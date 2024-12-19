import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import time
import json
from dotenv import load_dotenv
import os

last_activation_time = 0
current_door_state = "UNKNOWN"
COOLDOWN_PERIOD = 15  # Default value in seconds
first_signal = False
first_boot = True

if not load_dotenv():
    print("error loading env variables")
    exit(1)

# GPIO Setup
RELAY_PIN = 18  # GPIO pin for the relay
REED_PIN = 17   # GPIO pin for the reed switch

GPIO.setmode(GPIO.BCM)

GPIO.setup(REED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Cooldown Configuration Discovery
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



# MQTT Garage Setup
MQTT_BROKER = os.getenv('MQTT_BROKER')  # Replace with your MQTT broker's IP address
MQTT_USERNAME = os.getenv('MQTT_USERNAME')  # Replace with your MQTT broker's IP address
MQTT_PASSWORD = os.getenv('MQTT_PASSWORD')  # Replace with your MQTT broker's IP address
MQTT_CLIENT_ID = "garage_door-pi"
TOPIC_STATE = "garage/door/state"
TOPIC_COMMAND = "garage/door/command"
TOPIC_AVAILABILITY = "garage/availability"
DISCOVERY_TOPIC = "homeassistant/cover/garage/config"

DISCOVERY_PAYLOAD = {
    "name": "Garage Door",
    "command_topic": TOPIC_COMMAND,
    "state_topic": TOPIC_STATE,
    "payload_open": "OPEN",
    "payload_close": "CLOSE",
    "state_open": "OPEN",
    "state_closed": "CLOSED",
    "availability_topic": TOPIC_AVAILABILITY,
    "payload_available": "online",
    "payload_not_available": "offline",
    "device_class": "garage",
    "unique_id": MQTT_CLIENT_ID,
    "optimistic": False,  # Optional: Disable optimistic mode (requires feedback)
    "retain": True        # Optional: Retain last known state
    
}

# Helper Functions
def publish_state(state=None):
    """Publish the state of the garage door."""
    global current_door_state
    if state:
        current_door_state = state
    else:
        # Determine state from reed switch
        current_door_state = "OPEN" if GPIO.input(REED_PIN) == GPIO.HIGH else "CLOSED"
    client.publish(TOPIC_STATE, current_door_state, retain=True)
    print(f"State published: {current_door_state}")

# def handle_command(command):
#     """Handle open/close commands."""
#     if command in ["OPEN", "CLOSE"]:
#         GPIO.setup(RELAY_PIN, GPIO.LOW)  # Activate relay
#         time.sleep(.5)                   # Simulate button press
#         GPIO.setup(RELAY_PIN, GPIO.HIGH)  # Deactivate relay

def handle_command(command):
    """Handle open/close commands with cooldown logic and intermediate states."""
    
    global last_activation_time, current_door_state, first_signal, first_boot

    # Cooldown logic
    current_time = time.time()
    if first_boot:
        first_boot = False
        return
    first_signal = True
    if first_signal:
        GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.HIGH)
        first_signal = False
        
    # Check if the cooldown has expired
    if current_time - last_activation_time >= COOLDOWN_PERIOD:
        if command == "OPEN" and current_door_state != "OPEN":
            print("Command: OPEN")
            publish_state("OPENING")  # Set intermediate state
            GPIO.setup(RELAY_PIN, GPIO.LOW)  # Activate relay
            time.sleep(0.5)  # Simulate button press
            GPIO.setup(RELAY_PIN, GPIO.HIGH)  # Deactivate relay
            last_activation_time = current_time

            # Wait 10 seconds for the door to open
            time.sleep(COOLDOWN_PERIOD)
            publish_state("OPEN")  # Set state to OPEN after timer

        elif command == "CLOSE" and current_door_state != "CLOSED":
            print("Command: CLOSE")
            publish_state("CLOSING")  # Set intermediate state
            GPIO.setup(RELAY_PIN, GPIO.LOW)  # Activate relay
            time.sleep(0.5)  # Simulate button press
            GPIO.setup(RELAY_PIN, GPIO.HIGH)  # Deactivate relay
            last_activation_time = current_time

            # Wait 15 seconds or check sensor to confirm closed
            start_time = time.time()
            while GPIO.input(REED_PIN) == GPIO.HIGH and (time.time() - start_time) < COOLDOWN_PERIOD:
                time.sleep(0.5)  # Check reed switch every 0.5 seconds
            if GPIO.input(REED_PIN) == GPIO.LOW:
                publish_state("CLOSED")  # Sensor confirmed closed
            else:
                print(f"Door did not fully close after {COOLDOWN_PERIOD} seconds.")
                publish_state("CLOSING_FAILED")  # Optional failed state

        else:
            print("Command ignored: Door already in the desired state.")
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

# Handle MQTT messages
def on_message(client, userdata, msg):
    global COOLDOWN_PERIOD

    print(f"Received message on topic '{msg.topic}': {msg.payload.decode()}")  # Debug log

    # Handle Cooldown Topic
    if msg.topic == COOLDOWN_COMMAND_TOPIC:
        try:
            new_cooldown = int(msg.payload.decode())
            if 5 <= new_cooldown <= 60:
                COOLDOWN_PERIOD = new_cooldown
                print(f"Cooldown period updated to {COOLDOWN_PERIOD} seconds.")
                client.publish(COOLDOWN_STATE_TOPIC, COOLDOWN_PERIOD, retain=True)
            else:
                print("Received invalid cooldown value (out of range).")
        except ValueError:
            print("Invalid cooldown format received.")

    # Handle Command Topic (OPEN/CLOSE)
    elif msg.topic == TOPIC_COMMAND:
        command = msg.payload.decode().strip().upper()  # Clean and ensure uppercase
        if command in ["OPEN", "CLOSE"]:
            print(f"Processing command: {command}")
            handle_command(command)
        else:
            print(f"Invalid command received: {command}")

# Publish the discovery message
def publish_cooldown_discovery():
    client.publish(COOLDOWN_DISCOVERY_TOPIC, json.dumps(COOLDOWN_DISCOVERY_PAYLOAD), retain=True)
    print("Published MQTT Discovery for Garage Door Cooldown.")

# Initialize MQTT Client
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
        if current_state != last_state:  # State changed
            publish_state()
            last_state = current_state
        time.sleep(0.1)
except KeyboardInterrupt:
    GPIO.cleanup()
    client.loop_stop()
    client.disconnect()