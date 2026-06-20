"""Smart garage door controller.

Bridges a Raspberry Pi (relay + reed switch) to Home Assistant over MQTT,
using MQTT auto-discovery so the cover and a cooldown "number" entity appear
automatically.

Architecture:
  * A single asyncio event loop runs in the main thread and owns all the
    door logic (pulsing the relay, polling the reed switch).
  * paho-mqtt runs its network loop in its own thread; callbacks hand work
    to the asyncio loop via ``run_coroutine_threadsafe`` so the MQTT thread
    never blocks (keepalives keep flowing).
"""

import asyncio
import json
import logging
import os
import signal
import sys

import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
from dotenv import load_dotenv

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOGS_DIR = os.path.join(SCRIPT_DIR, "logs")
os.makedirs(LOGS_DIR, exist_ok=True)

logger = logging.getLogger("garage_door")
logger.setLevel(logging.INFO)
_formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")

_file_handler = logging.FileHandler(os.path.join(LOGS_DIR, "garage_door.log"))
_file_handler.setFormatter(_formatter)
logger.addHandler(_file_handler)

_console_handler = logging.StreamHandler()
_console_handler.setFormatter(_formatter)
logger.addHandler(_console_handler)


# ---------------------------------------------------------------------------
# Configuration (from .env)
# ---------------------------------------------------------------------------
if not load_dotenv():
    logger.warning("No .env file found; relying on existing environment variables.")


def _required(name: str) -> str:
    value = os.getenv(name)
    if value is None or value == "":
        logger.error("Missing required environment variable: %s", name)
        sys.exit(1)
    return value


def _required_int(name: str) -> int:
    try:
        return int(_required(name))
    except ValueError:
        logger.error("Environment variable %s must be an integer.", name)
        sys.exit(1)


# GPIO pins
RELAY_PIN = _required_int("RELAY_PIN")
REED_PIN = _required_int("REED_PIN")

# Relay polarity. Most cheap relay boards are active-LOW: driving the pin LOW
# energises the relay. Set RELAY_ACTIVE_LOW=false for active-HIGH boards.
RELAY_ACTIVE_LOW = os.getenv("RELAY_ACTIVE_LOW", "true").strip().lower() != "false"
RELAY_ACTIVE = GPIO.LOW if RELAY_ACTIVE_LOW else GPIO.HIGH
RELAY_IDLE = GPIO.HIGH if RELAY_ACTIVE_LOW else GPIO.LOW

# How long the relay is held to simulate a button press.
PULSE_SECONDS = float(os.getenv("PULSE_SECONDS", "0.5"))

# Cooldown: minimum seconds between accepted commands (anti double-trigger).
# Adjustable at runtime from Home Assistant.
COOLDOWN_PERIOD = int(os.getenv("COOLDOWN_PERIOD", "15"))
COOLDOWN_MIN = 5
COOLDOWN_MAX = 60

# Opening can't be confirmed by the single (closed-only) reed switch, so we
# just show "opening" for this fixed duration, then report "open".
OPEN_TRAVEL_TIME = float(os.getenv("OPEN_TRAVEL_TIME", "15"))

# Closing is confirmed by the reed switch; this is the max time we wait for
# that confirmation before flagging the move as failed. Keep it above the
# real close travel time (observed ~17s) plus margin.
CLOSE_TIMEOUT = float(os.getenv("CLOSE_TIMEOUT", "30"))

# MQTT
MQTT_BROKER = _required("MQTT_BROKER")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD")
MQTT_CLIENT_ID = "garage_door-pi"

TOPIC_STATE = "garage/door/state"
TOPIC_COMMAND = "garage/door/command"
TOPIC_AVAILABILITY = "garage/door/availability"

DISCOVERY_TOPIC = "homeassistant/cover/garage/config"
COOLDOWN_DISCOVERY_TOPIC = "homeassistant/number/garage_door_cooldown/config"
COOLDOWN_STATE_TOPIC = "garage/door/cooldown"
COOLDOWN_COMMAND_TOPIC = "garage/door/cooldown/set"

# Shared "device" block so both entities group under one device in HA.
DEVICE_INFO = {
    "identifiers": ["garage_door_pi"],
    "name": "Garage Door",
    "manufacturer": "DIY",
    "model": "Raspberry Pi Garage Controller",
}

DISCOVERY_PAYLOAD = {
    "name": None,  # use the device name
    "command_topic": TOPIC_COMMAND,
    "state_topic": TOPIC_STATE,
    "payload_open": "open",
    "payload_close": "close",
    "state_open": "open",
    "state_opening": "opening",
    "state_closed": "closed",
    "state_closing": "closing",
    "availability_topic": TOPIC_AVAILABILITY,
    "payload_available": "online",
    "payload_not_available": "offline",
    "device_class": "garage",
    "unique_id": MQTT_CLIENT_ID,
    "optimistic": False,
    "device": DEVICE_INFO,
}

COOLDOWN_DISCOVERY_PAYLOAD = {
    "name": "Cooldown",
    "unique_id": "garage_door_cooldown",
    "state_topic": COOLDOWN_STATE_TOPIC,
    "command_topic": COOLDOWN_COMMAND_TOPIC,
    "availability_topic": TOPIC_AVAILABILITY,
    "payload_available": "online",
    "payload_not_available": "offline",
    "unit_of_measurement": "seconds",
    "min": COOLDOWN_MIN,
    "max": COOLDOWN_MAX,
    "step": 1,
    "mode": "slider",
    "entity_category": "config",
    "device": DEVICE_INFO,
}


# ---------------------------------------------------------------------------
# GPIO setup
# ---------------------------------------------------------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Configure the relay pin's internal pull to its IDLE level BEFORE driving it
# as an output. This is critical: on the BCM283x the pull-up/down setting is
# *sticky* — it persists across warm reboots. By pinning the pull to idle we
# hold the relay line at its safe level during the boot window (after a reboot,
# before this service starts), so the door is NOT triggered on reboot.
#
# Without this, GPIO17 falls back to its power-on default pull (DOWN = LOW),
# which is the *active* level for an active-low relay — energising it for the
# whole boot and pulsing the opener. (The previous working version relied on
# exactly this PUD_UP line; the rewrite had dropped it.)
RELAY_IDLE_PULL = GPIO.PUD_UP if RELAY_IDLE == GPIO.HIGH else GPIO.PUD_DOWN
GPIO.setup(RELAY_PIN, GPIO.IN, pull_up_down=RELAY_IDLE_PULL)
GPIO.setup(RELAY_PIN, GPIO.OUT, initial=RELAY_IDLE)
GPIO.setup(REED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def reed_is_closed() -> bool:
    """Return True when the reed switch reports the door fully closed.

    With PUD_UP the input reads LOW when the (closed-position) reed contact
    shorts the pin to ground, i.e. when the door is closed.
    """
    return GPIO.input(REED_PIN) == GPIO.LOW


# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
last_activation_time = 0.0
current_door_state = "unknown"
in_motion = False  # True while a command-driven transition is running.
loop: asyncio.AbstractEventLoop = None  # set in main()
client: mqtt.Client = None  # set in main()


# ---------------------------------------------------------------------------
# Door logic (runs on the asyncio loop)
# ---------------------------------------------------------------------------
def _publish_state(state: str) -> None:
    global current_door_state
    current_door_state = state
    client.publish(TOPIC_STATE, state, retain=True)
    logger.info("State published: %s", state)


async def publish_sensor_state() -> None:
    """Publish the door state derived from the reed switch.

    Skipped while a command transition is in progress so the transient
    opening/closing states aren't clobbered by the reed reading.
    """
    if in_motion:
        return
    _publish_state("closed" if reed_is_closed() else "open")


async def _pulse_relay() -> None:
    GPIO.output(RELAY_PIN, RELAY_ACTIVE)
    await asyncio.sleep(PULSE_SECONDS)
    GPIO.output(RELAY_PIN, RELAY_IDLE)


async def _wait_until_reed_closed() -> bool:
    """Poll until the reed reports closed, or until CLOSE_TIMEOUT elapses.

    Returns True if the door reached the closed position, False on timeout.
    """
    deadline = asyncio.get_event_loop().time() + CLOSE_TIMEOUT
    while not reed_is_closed():
        if asyncio.get_event_loop().time() >= deadline:
            logger.warning("Timed out waiting for the door to close.")
            return False
        await asyncio.sleep(0.2)
    return True


async def handle_command(command: str) -> None:
    global last_activation_time, in_motion

    now = asyncio.get_event_loop().time()
    if in_motion:
        logger.info("Ignored '%s': door already in motion.", command)
        return
    if now - last_activation_time < COOLDOWN_PERIOD:
        remaining = int(COOLDOWN_PERIOD - (now - last_activation_time))
        logger.info("Ignored '%s': cooldown active (%ds remaining).", command, remaining)
        return

    if command == "open" and current_door_state == "open":
        logger.info("Ignored 'open': door already open.")
        return
    if command == "close" and current_door_state == "closed":
        logger.info("Ignored 'close': door already closed.")
        return

    last_activation_time = now
    in_motion = True
    try:
        if command == "open":
            # The reed only senses the *closed* position, so it can't confirm
            # "fully open". Show "opening" for the whole travel time, then trust
            # that the door is open (unless the reed still reads closed, meaning
            # it never moved).
            _publish_state("opening")
            await _pulse_relay()
            await asyncio.sleep(OPEN_TRAVEL_TIME)
            if reed_is_closed():
                logger.warning("Door still reads closed after 'open'; it may not have moved.")
                _publish_state("closed")
            else:
                _publish_state("open")
        else:  # close
            # Closing can be confirmed by the reed switch.
            _publish_state("closing")
            await _pulse_relay()
            if await _wait_until_reed_closed():
                _publish_state("closed")
            else:
                logger.warning("Door did not confirm closed within the travel time.")
                _publish_state("open")
    finally:
        in_motion = False


async def monitor_reed() -> None:
    """Publish door state whenever the reed switch changes (manual operation)."""
    last_raw = None
    while True:
        raw = GPIO.input(REED_PIN)
        if raw != last_raw:
            last_raw = raw
            await publish_sensor_state()
        await asyncio.sleep(0.5)


def set_cooldown(value: int) -> None:
    global COOLDOWN_PERIOD
    COOLDOWN_PERIOD = value
    logger.info("Cooldown updated to %d seconds.", value)
    client.publish(COOLDOWN_STATE_TOPIC, value, retain=True)


# ---------------------------------------------------------------------------
# MQTT callbacks (run on the paho network thread)
# ---------------------------------------------------------------------------
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code != 0:
        logger.error("MQTT connection failed: %s", reason_code)
        return
    logger.info("Connected to MQTT broker.")
    client.subscribe(TOPIC_COMMAND)
    client.subscribe(COOLDOWN_COMMAND_TOPIC)
    client.publish(TOPIC_AVAILABILITY, "online", retain=True)
    client.publish(DISCOVERY_TOPIC, json.dumps(DISCOVERY_PAYLOAD), retain=True)
    client.publish(COOLDOWN_DISCOVERY_TOPIC, json.dumps(COOLDOWN_DISCOVERY_PAYLOAD), retain=True)
    client.publish(COOLDOWN_STATE_TOPIC, COOLDOWN_PERIOD, retain=True)
    logger.info("Published availability and discovery payloads.")
    asyncio.run_coroutine_threadsafe(publish_sensor_state(), loop)


def on_disconnect(client, userdata, flags, reason_code, properties):
    logger.warning("Disconnected from MQTT broker (rc=%s). Auto-reconnect pending.", reason_code)


def on_message(client, userdata, msg):
    if msg.retain:
        # Ignore stale retained commands replayed on (re)subscribe.
        return

    payload = msg.payload.decode().strip()
    logger.info("Message on '%s': %s", msg.topic, payload)

    if msg.topic == TOPIC_COMMAND:
        command = payload.lower()
        if command in ("open", "close"):
            asyncio.run_coroutine_threadsafe(handle_command(command), loop)
        else:
            logger.warning("Invalid command: %s", payload)

    elif msg.topic == COOLDOWN_COMMAND_TOPIC:
        try:
            value = int(payload)
        except ValueError:
            logger.warning("Invalid cooldown value: %s", payload)
            return
        if COOLDOWN_MIN <= value <= COOLDOWN_MAX:
            loop.call_soon_threadsafe(set_cooldown, value)
        else:
            logger.warning("Cooldown out of range [%d-%d]: %d", COOLDOWN_MIN, COOLDOWN_MAX, value)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> None:
    global loop, client

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id=MQTT_CLIENT_ID,
    )
    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.will_set(TOPIC_AVAILABILITY, "offline", retain=True)

    # Stop cleanly on Ctrl+C and on `systemctl stop` (SIGTERM).
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, loop.stop)

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    except Exception as exc:  # noqa: BLE001 - let auto-reconnect take over.
        logger.error("Initial MQTT connect failed (%s); will keep retrying.", exc)
        client.connect_async(MQTT_BROKER, MQTT_PORT, keepalive=60)

    client.loop_start()
    loop.create_task(monitor_reed())

    try:
        loop.run_forever()
    finally:
        logger.info("Shutting down...")
        client.publish(TOPIC_AVAILABILITY, "offline", retain=True)
        client.loop_stop()
        client.disconnect()
        loop.close()
        # Keep the relay pin DRIVEN at its idle level across shutdown.
        # We intentionally do NOT GPIO.cleanup() the relay: cleanup() reverts
        # it to a floating input, and during a service restart that float is
        # read by the relay board as a button press, triggering the door on
        # every restart. Holding the output level keeps the relay idle while
        # the next process starts up.
        GPIO.output(RELAY_PIN, RELAY_IDLE)
        GPIO.cleanup(REED_PIN)
        logger.info("Disconnected from MQTT; relay held idle (not cleaned up).")


if __name__ == "__main__":
    main()
