# mqtt_garage_door

Smart garage-door opener for a Raspberry Pi using a relay and a reed switch,
integrated with Home Assistant over MQTT (auto-discovery).

## How it works

- A **relay** is pulsed to emulate the physical garage button.
- A **reed switch** reports whether the door is fully closed.
- The Pi publishes MQTT discovery so Home Assistant auto-creates a `cover`
  entity plus a `number` slider to tune the command cooldown.
- All door logic runs on a single asyncio loop; paho-mqtt runs its network
  loop in a separate thread, so MQTT keepalives are never blocked by a move.

## Setup

1. Create and populate the environment file:
   ```bash
   cp .env_template .env
   # edit .env with your pins and MQTT broker details
   ```
2. Install dependencies (a virtualenv is recommended):
   ```bash
   python3 -m venv venv
   ./venv/bin/pip install -r requirements.txt
   ```
3. Run it:
   ```bash
   ./venv/bin/python3 script.py
   ```

## Configuration (.env)

| Variable           | Description                                               | Default |
|--------------------|-----------------------------------------------------------|---------|
| `RELAY_PIN`        | BCM pin wired to the relay                                | —       |
| `REED_PIN`         | BCM pin wired to the reed switch                          | —       |
| `RELAY_ACTIVE_LOW` | `true` for active-LOW relay boards, `false` for active-HIGH | `true`  |
| `PULSE_SECONDS`    | Relay hold time to emulate a button press                 | `0.5`   |
| `COOLDOWN_PERIOD`  | Min seconds between accepted commands (5–60)              | `15`    |
| `DOOR_TRAVEL_TIME` | Seconds to wait for the reed switch to confirm a move     | `20`    |
| `MQTT_BROKER`      | Broker hostname/IP                                        | —       |
| `MQTT_PORT`        | Broker port                                               | `1883`  |
| `MQTT_USERNAME`    | Broker username (optional)                                | —       |
| `MQTT_PASSWORD`    | Broker password (optional)                                | —       |

## Run as a service

See [`garage-pi.service`](garage-pi.service) for a systemd unit. Install with:

```bash
sudo cp garage-pi.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now garage-pi.service
```

Logs are written to `logs/garage_door.log` and to the systemd journal.
