import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import time
import json
import asyncio
from dotenv import load_dotenv
import os
import logging

# -------------------------------
# Logging Configuration
# -------------------------------
# Determine the directory where the script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
# Create the logs directory if it doesn't exist
logs_dir = os.path.join(script_dir, "logs")
if not os.path.exists(logs_dir):
    os.makedirs(logs_dir)

# Configure logger
LOG_FILE = os.path.join(logs_dir, "garage_door.log")
logger = logging.getLogger("garage_door")
logger.setLevel(logging.DEBUG)

# File handler for logging to a file
file_handler = logging.FileHandler(LOG_FILE)
file_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
file_handler.setFormatter(file_formatter)
logger.addHandler(file_handler)

# Also add a console handler if desired
console_handler = logging.StreamHandler()
console_handler.setFormatter(file_formatter)
logger.addHandler(console_handler)

# -------------------------------
# Global Variables and Setup
# -------------------------------
last_activation_time = 0
current_door_state = "UNKNOWN"
COOLDOWN_PERIOD = 15  # seconds
first_command = True
first_boot = True
loop = None  # Will set the asyncio event loop later

# Load environment variables
if not load_dotenv():
    logger.error("Error loading env variables")
    exit(1)

# GPIO Setup
RELAY_PIN = 27
REED_PIN = 17

GPIO.setmode(GPIO.BCM)

GPIO.setup(REED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.HIGH)
# MQTT Setup
MQTT_BROKER = os.getenv('MQTT_BROKER')
MQTT_USERNAME = os.getenv('MQTT_USERNAME')
MQTT_PASSWORD = os.getenv('MQTT_PASSWORD')
MQTT_CLIENT_ID = "garage_door-pi"

TOPIC_STATE = "garage/door/state"
TOPIC_COMMAND = "garage/door/command"
TOPIC_AVAILABILITY = "garage/door/availability"

DISCOVERY_TOPIC = "homeassistant/cover/garage/config"
DISCOVERY_PAYLOAD = {
    "name": "Garage Door",
    "command_topic": TOPIC_COMMAND,
    "state_topic": TOPIC_STATE,
    "payload_open": "open",
    "payload_close": "close",
    "state_open": "open",
    "state_closed": "closed",
    "availability_topic": TOPIC_AVAILABILITY,
    "payload_available": "online",
    "payload_not_available": "offline",
    "device_class": "garage",
    "unique_id": MQTT_CLIENT_ID,
    "optimistic": False,
    "retain": True,
    "supported_features": 0
}

COOLDOWN_DISCOVERY_TOPIC = "homeassistant/number/garage_door_cooldown/config"
COOLDOWN_STATE_TOPIC = "garage/door/cooldown"
COOLDOWN_COMMAND_TOPIC = "garage/door/cooldown/set"

COOLDOWN_DISCOVERY_PAYLOAD = {
    "name": "Garage Door Cooldown",
    "unique_id": "garage_door_cooldown",
    "state_topic": COOLDOWN_STATE_TOPIC,
    "command_topic": COOLDOWN_COMMAND_TOPIC,
    "unit_of_measurement": "seconds",
    "min": 5,
    "max": 60,
    "step": 1,
    "mode": "slider"
}


# -------------------------------
# Async Functions and MQTT Callbacks
# -------------------------------
async def publish_state(state=None):
    global current_door_state
    if state:
        current_door_state = state
    else:
        # Directly read the sensor for real-time state
        state = GPIO.input(REED_PIN)
        print(state)
        current_door_state = "open" if GPIO.input(REED_PIN) == GPIO.LOW else "closed"
    client.publish(TOPIC_STATE, current_door_state, retain=True)
    logger.info(f"State published: {current_door_state}")


async def handle_command(command):
    global last_activation_time, current_door_state, first_command, first_boot

    current_time = time.time()

    # if first_boot:
    #     first_boot = False
    #     first_command = True
    #     logger.debug("First boot detected. Skipping command processing.")
    #     return
    logger.info(f"Command: {command}")
    # if first_command:
    #     # Relay initialization - leaving this as-is per your request
    #     GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.HIGH)
    #     first_command = False
    #     logger.debug("First command initialization complete.")
    state = GPIO.input(RELAY_PIN)
    print("Relay state is:", state)
    if current_time - last_activation_time >= COOLDOWN_PERIOD:
        if command == "open" and current_door_state != "open":
            await publish_state("opening")
            # Activate relay via re-setup (keep as-is)
            # GPIO.setup(RELAY_PIN, GPIO.OUT, GPIO.LOW)
            GPIO.output(RELAY_PIN, GPIO.LOW)  
            state = GPIO.input(RELAY_PIN)
            print("Relay state is:", state)
            await asyncio.sleep(0.5)
            GPIO.output(RELAY_PIN, GPIO.HIGH)
            # GPIO.setup(RELAY_PIN, GPIO.OUT, GPIO.HIGH)
            state = GPIO.input(RELAY_PIN)
            print("Relay state is:", state)
            print("-----")
            last_activation_time = current_time
            await asyncio.sleep(COOLDOWN_PERIOD)
            await publish_state("open")

        elif command == "close" and current_door_state != "closed":
            await publish_state("closing")
            # GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.HIGH)
            # GPIO.output(RELAY_PIN, GPIO.LOW)
            GPIO.output(RELAY_PIN, GPIO.LOW)  
            state = GPIO.input(RELAY_PIN)
            print("Relay state is:", state)
            await asyncio.sleep(0.5)
            # GPIO.setup(RELAY_PIN, GPIO.OUT, GPIO.HIGH)
            GPIO.output(RELAY_PIN, GPIO.HIGH)
            state = GPIO.input(RELAY_PIN)
            print("Relay state is:", state)
            print("-----")
            last_activation_time = current_time
            start_time = time.time()
            # Wait up to COOLDOWN_PERIOD seconds for the door to close
            while GPIO.input(REED_PIN) == GPIO.LOW and (time.time() - start_time) < COOLDOWN_PERIOD:
                await asyncio.sleep(0.5)

            if GPIO.input(REED_PIN) == GPIO.HIGH:
                await publish_state("closed")
            else:
                logger.error("Door did not fully close.")
                await publish_state("closing_failed")
        else:
            logger.info("Command ignored: Door already in desired state.")
    else:
        remaining = int(COOLDOWN_PERIOD - (current_time - last_activation_time))
        logger.info(f"Ignored command '{command}': cooldown active ({remaining}s remaining).")


# MQTT Callbacks
def on_connect(client, userdata, flags, rc, properties):
    logger.info("Connected to MQTT Broker!")
    client.subscribe(TOPIC_COMMAND)
    client.subscribe(COOLDOWN_COMMAND_TOPIC)
    client.subscribe(COOLDOWN_STATE_TOPIC)
    client.publish(TOPIC_AVAILABILITY, "online", retain=True)
    publish_cooldown_discovery()
    publish_cover_discovery()
    # Schedule publish_state() in the already running event loop
    asyncio.run_coroutine_threadsafe(publish_state(), loop)


def on_message(client, userdata, msg):
    global COOLDOWN_PERIOD

    if msg.retain:
        logger.info(f"Ignored retained message on topic '{msg.topic}'")
        return

    logger.info(f"Received message on topic '{msg.topic}': {msg.payload.decode()}")

    if msg.topic == COOLDOWN_COMMAND_TOPIC:
        try:
            new_cooldown = int(msg.payload.decode())
            if 5 <= new_cooldown <= 60:
                COOLDOWN_PERIOD = new_cooldown
                logger.info(f"Cooldown updated to {COOLDOWN_PERIOD} seconds.")
                client.publish(COOLDOWN_STATE_TOPIC, COOLDOWN_PERIOD, retain=True)
            else:
                logger.warning("Invalid cooldown value.")
        except ValueError:
            logger.warning("Invalid cooldown format.")
    elif msg.topic == TOPIC_COMMAND:
        command = msg.payload.decode().strip().lower()
        if command in ["open", "close"]:
            logger.info(f"Processing command: {command}")
            # Submit the handle_command task to our global event loop
            asyncio.run_coroutine_threadsafe(handle_command(command), loop)
        else:
            logger.warning(f"Invalid command: {command}")


def publish_cover_discovery():
    client.publish(DISCOVERY_TOPIC, json.dumps(DISCOVERY_PAYLOAD), retain=True)
    logger.info("Published MQTT Discovery for Garage Door Cover.")


def publish_cooldown_discovery():
    client.publish(COOLDOWN_DISCOVERY_TOPIC, json.dumps(COOLDOWN_DISCOVERY_PAYLOAD), retain=True)
    logger.info("Published MQTT Discovery for Cooldown Slider.")


# -------------------------------
# MQTT Client Initialization
# -------------------------------
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, MQTT_CLIENT_ID)
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
client.on_connect = on_connect
client.on_message = on_message

client.will_set(TOPIC_AVAILABILITY, "offline", retain=True)
client.connect(MQTT_BROKER, 1883, 60)
client.loop_start()


# -------------------------------
# Asynchronous Reed Switch Monitor Task
# -------------------------------
async def monitor_reed():
    last_state = None
    while True:
        current_state = GPIO.input(REED_PIN)
        if current_state != last_state:
            await publish_state()
            last_state = current_state
        await asyncio.sleep(0.1)


# -------------------------------
# Main Entry Point
# -------------------------------
if __name__ == "__main__":
    try:
        # Create a new event loop and assign it to the global variable
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

        # Schedule the reed switch monitoring task
        loop.create_task(monitor_reed())

        # Run the asyncio event loop forever
        loop.run_forever()
    except KeyboardInterrupt:
        logger.info("Shutting down due to keyboard interrupt...")
    finally:
        GPIO.cleanup()
        client.loop_stop()
        client.disconnect()
        loop.stop()
        logger.info("Cleaned up GPIO and disconnected from MQTT.")
