import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import time
import json
from dotenv import load_dotenv
import os

glob_state = "CLOSED"

if not load_dotenv():
    print("error loading env variables")
    exit(1)

# GPIO Setup
RELAY_PIN = 18  # GPIO pin for the relay
REED_PIN = 17   # GPIO pin for the reed switch

GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(REED_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)



# MQTT Setup
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
def publish_state():
    """Publish the state of the garage door."""
    state = "OPEN" if GPIO.input(REED_PIN) == GPIO.HIGH else "CLOSED"
    client.publish(TOPIC_STATE, state, retain=True)

def handle_command(command):
    """Handle open/close commands."""
    print("hehehe", command)
    if command in ["OPEN", "CLOSE"]:
        GPIO.output(RELAY_PIN, GPIO.LOW)  # Activate relay
        time.sleep(.5)                   # Simulate button press
        GPIO.output(RELAY_PIN, GPIO.HIGH)  # Deactivate relay

# MQTT Callbacks
def on_connect(client, userdata, flags, rc, properties):
    print("Connected to MQTT Broker!")
    client.subscribe(TOPIC_COMMAND)
    client.publish(TOPIC_AVAILABILITY, "online", retain=True)
    client.publish(DISCOVERY_TOPIC, json.dumps(DISCOVERY_PAYLOAD), qos=0, retain=True)

def on_message(client, userdata, msg):
    if msg.topic == TOPIC_COMMAND:
        handle_command(msg.payload.decode())

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