import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import time
import json
import asyncio
from dotenv import load_dotenv
import os

# Initial State
last_activation_time = 0
current_door_state = "UNKNOWN"
COOLDOWN_PERIOD = 15  # seconds
first_command = False
first_boot = True

# Load environment variables
if not load_dotenv():
    print("Error loading env variables")
    exit(1)

# GPIO Setup
RELAY_PIN = 18
REED_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(REED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

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

# Async State Publishing
async def publish_state(state=None):
    global current_door_state
    if state:
        current_door_state = state
    else:
        current_door_state = "open" if GPIO.input(REED_PIN) == GPIO.HIGH else "closed"
    client.publish(TOPIC_STATE, current_door_state, retain=True)
    print(f"State published: {current_door_state}")

async def handle_command(command):
    global last_activation_time, current_door_state, first_command, first_boot

    current_time = time.time()

    if first_boot:
        first_boot = False
        first_command = True
        return

    if first_command:
        GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.HIGH)
        first_command = False

    if current_time - last_activation_time >= COOLDOWN_PERIOD:
        if command == "open" and current_door_state != "open":
            print("Command: open")
            await publish_state("opening")
            GPIO.setup(RELAY_PIN, GPIO.LOW)  # Activate relay
            time.sleep(0.5)
            GPIO.setup(RELAY_PIN, GPIO.HIGH)  # Deactivate relay
            last_activation_time = current_time
            await asyncio.sleep(COOLDOWN_PERIOD)
            await publish_state("open")

        elif command == "close" and current_door_state != "closed":
            print("Command: close")
            await publish_state("closing")
            GPIO.setup(RELAY_PIN, GPIO.LOW)
            time.sleep(0.5)
            GPIO.setup(RELAY_PIN, GPIO.HIGH)
            last_activation_time = current_time

            start_time = time.time()
            while GPIO.input(REED_PIN) == GPIO.HIGH and (time.time() - start_time) < COOLDOWN_PERIOD:
                await asyncio.sleep(0.5)

            if GPIO.input(REED_PIN) == GPIO.LOW:
                await publish_state("closed")
            else:
                print("Door did not fully close.")
                await publish_state("closing_failed")
        else:
            print("Command ignored: Door already in desired state.")
    else:
        remaining = int(COOLDOWN_PERIOD - (current_time - last_activation_time))
        print(f"Ignored command '{command}': cooldown active ({remaining}s remaining).")

# MQTT Callbacks
def on_connect(client, userdata, flags, rc, properties):
    print("Connected to MQTT Broker!")
    client.subscribe(TOPIC_COMMAND)
    client.subscribe(COOLDOWN_COMMAND_TOPIC)
    client.subscribe(COOLDOWN_STATE_TOPIC)
    client.publish(TOPIC_AVAILABILITY, "online", retain=True)
    publish_cooldown_discovery()
    publish_cover_discovery()
    asyncio.run(publish_state())  # Publish current state on connect

def on_message(client, userdata, msg):
    global COOLDOWN_PERIOD

    if msg.retain:
        print(f"Ignored retained message on topic '{msg.topic}'")
        return

    print(f"Received message on topic '{msg.topic}': {msg.payload.decode()}")

    if msg.topic == COOLDOWN_COMMAND_TOPIC:
        try:
            new_cooldown = int(msg.payload.decode())
            if 5 <= new_cooldown <= 60:
                COOLDOWN_PERIOD = new_cooldown
                print(f"Cooldown updated to {COOLDOWN_PERIOD} seconds.")
                client.publish(COOLDOWN_STATE_TOPIC, COOLDOWN_PERIOD, retain=True)
            else:
                print("Invalid cooldown value.")
        except ValueError:
            print("Invalid cooldown format.")
    elif msg.topic == TOPIC_COMMAND:
        command = msg.payload.decode().strip().lower()
        if command in ["open", "close"]:
            print(f"Processing command: {command}")
            asyncio.run(handle_command(command))
        else:
            print(f"Invalid command: {command}")

def publish_cover_discovery():
    client.publish(DISCOVERY_TOPIC, json.dumps(DISCOVERY_PAYLOAD), retain=True)
    print("Published MQTT Discovery for Garage Door Cover.")

def publish_cooldown_discovery():
    client.publish(COOLDOWN_DISCOVERY_TOPIC, json.dumps(COOLDOWN_DISCOVERY_PAYLOAD), retain=True)
    print("Published MQTT Discovery for Cooldown Slider.")

# Initialize MQTT
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, MQTT_CLIENT_ID)
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
client.on_connect = on_connect
client.on_message = on_message

client.will_set(TOPIC_AVAILABILITY, "offline", retain=True)
client.connect(MQTT_BROKER, 1883, 60)
client.loop_start()

# Main Loop
try:
    last_state = None
    while True:
        current_state = GPIO.input(REED_PIN)
        if current_state != last_state:
            asyncio.run(publish_state())
            last_state = current_state
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Shutting down...")
    GPIO.cleanup()
    client.loop_stop()
    client.disconnect()
