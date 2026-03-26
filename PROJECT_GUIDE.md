# RFx-4 Project Guide

RFx-4 is an autonomous multi-kite aerial power system. Up to 4 kites fly coordinated flight patterns, each controlled by an Arduino Nano ESP32 that manages line length via a Moteus brushless motor controller. A wireless 4-joystick remote control sends commands to individual kites, and flight telemetry is logged at 30 Hz and automatically uploaded to Supabase cloud storage.

---

## Table of Contents

1. [Tech Stack](#tech-stack)
2. [Architecture & Data Flow](#architecture--data-flow)
3. [Build Environments](#build-environments)
4. [Setup & Getting Started](#setup--getting-started)
5. [Over-The-Air (OTA) Updates](#over-the-air-ota-updates)
6. [Web API Reference](#web-api-reference)
7. [Dashboards](#dashboards)
8. [Fleet & Multi-Kite Operation](#fleet--multi-kite-operation)
9. [Flight Logging & Supabase](#flight-logging--supabase)
10. [Autopilot](#autopilot)
11. [Troubleshooting](#troubleshooting)

---

## Tech Stack

### Hardware

| Component | Details |
|-----------|---------|
| **Main MCU** | Arduino Nano ESP32 (ESP32-S3), dual-core, USB-C |
| **Motor Controller** | Moteus (CAN bus, ID=1), velocity/position/torque modes |
| **CAN Interface** | MCP2518FD (SPI) at 1 Mbps, 40 MHz oscillator |
| **IMU** | Adafruit ICM-20948 (9-DOF), I2C address 0x69/0x68 |
| **Flash Storage** | SPIFFS filesystem (4 MB) for flight logs |
| **Remote Control** | ESP32-S3 DevKit with 4 independent joysticks (ADC + button) |
| **Status LEDs** | RGB LED (active LOW) |

### Firmware Frameworks & Libraries

| Library | Purpose | Version |
|---------|---------|---------|
| Arduino / ESP-IDF | Core framework | — |
| FreeRTOS | Dual-core task scheduling, queues | built-in |
| ESPAsyncWebServer | Non-blocking HTTP server | 3.9.4 |
| ArduinoJson | JSON serialization | 6.21.3 |
| ACAN2517FD | CAN FD controller driver | 2.1.16 |
| Moteus | Motor control commands | 1.0.2 |
| Adafruit ICM20X | 9-DOF IMU driver | 2.0.7 |
| WiFi / ESP-NOW | WiFi + peer-to-peer mesh networking | built-in |
| SPIFFS | Flash file system | built-in |
| HTTPClient | Cloud uploads | built-in |

### Communication Protocols

| Protocol | Purpose |
|----------|---------|
| **WiFi (AP_STA)** | Dashboards, OTA updates, Supabase uploads |
| **ESP-NOW** | Low-latency peer-to-peer fleet mesh and remote control |
| **CAN Bus** | Motor velocity/position commands to Moteus |
| **I2C** | IMU (SDA=A4, SCL=A5) |

### Third-Party Integrations

#### Supabase

Flight logs are auto-uploaded to Supabase Storage after each flight.

- **Project URL**: `https://iigtcugucufmrosrkgjv.supabase.co`
- **Storage Bucket**: `flight-logs`
- **Auth**: Anon key via `apikey` and `Authorization: Bearer` headers
- **Credentials**: Stored in `include/flight_logger_config.h`
- **Trigger**: Upload fires automatically on landing when WiFi is connected

#### NTP

Device clock is synced from `pool.ntp.org` on WiFi connect to generate accurate `YYYY-MM-DDThh-mm-ss` log filenames.

### Web Interfaces

Two HTML dashboards are served from SPIFFS (`data/` folder):

| File | URL | Purpose |
|------|-----|---------|
| `data/dashboard.html` | `http://<ip>/` | Single-kite real-time telemetry |
| `data/fleet_dashboard.html` | `http://<ip>/fleet` | Multi-kite fleet overview |

Dashboards use **Three.js** (3D orientation visualization) and **Chart.js** (telemetry charts). CORS is enabled on all `/status` endpoints so a browser on one kite's IP can monitor any other kite.

---

## Architecture & Data Flow

```
┌─ Hardware Sensors ──────────────────┐
│  Moteus Motor (CAN)                 │
│    • Velocity, torque, position     │
│  IMU ICM-20948 (I2C)               │
│    • Pitch, yaw, roll, gyro rates   │
│  Joystick Remote (ESP-NOW)          │
│    • Velocity command ±1000         │
└──────────────┬──────────────────────┘
               ↓
┌─ Autopilot (Core 1) ────────────────┐
│  • Line length target seeking       │
│  • Slack detection & reel-in        │
│  • Dive / AWW / Active-flight       │
│    confidence detectors (0–1)       │
└──────────────┬──────────────────────┘
               ↓
┌─ CAN Output ────────────────────────┐
│  • Motor velocity command           │
└──────────────┬──────────────────────┘
               ↓
┌─ Flight Logger (Core 0) ────────────┐
│  • 30 Hz CSV to SPIFFS              │
│  • ~21 columns of telemetry         │
│  • FreeRTOS queue (non-blocking)    │
└──────────────┬──────────────────────┘
               ↓
┌─ Web API ───────────────────────────┐
│  • GET /status → live JSON          │
│  • Dashboard polls at 1–2 Hz        │
└──────────────┬──────────────────────┘
               ↓
┌─ Supabase Upload ───────────────────┐
│  • HTTPClient POST on landing       │
│  • CSV → flight-logs bucket         │
└─────────────────────────────────────┘
```

**FreeRTOS task split:**
- **Core 1**: Autopilot control loop, CAN comms, ESP-NOW, web server
- **Core 0**: Flight logger (SPIFFS writes, Supabase upload)

---

## Build Environments

Four environments are defined in `platformio.ini`:

| Environment | Board | Purpose |
|-------------|-------|---------|
| `rfx-4-main` | `arduino_nano_esp32` | Main kite controller (production firmware) |
| `remote-control` | `esp32-s3-devkitc-1` | 4-joystick remote transmitter |
| `remote-calibration` | `esp32-s3-devkitc-1` | Joystick ADC calibration utility |
| `i2c-scanner` | `arduino_nano_esp32` | I2C device diagnostic tool |

### Flash Partition Layout

```
Address Range         | Partition    | Size
0x00000000–0x00008000 | Bootloader   | 32 KB
0x00008000–0x00009000 | NVS          | 4 KB
0x00009000–0x00010000 | OTA Data     | 28 KB
0x00010000–0x00200000 | OTA0 (app)   | 2 MB
0x00200000–0x00400000 | OTA1 (app)   | 2 MB
0x00400000–0x00FFB000 | SPIFFS       | ~4 MB
```

Total: 16 MB flash (configured via `board_build.partitions = partitions.csv` or `large_spiffs_16M`).

### Building

```bash
# Build main kite firmware
platformio run --environment rfx-4-main

# Build remote control firmware
platformio run --environment remote-control

# Upload filesystem (dashboards)
platformio run --target uploadfs --environment rfx-4-main --upload-port COM15
```

---

## Setup & Getting Started

### 1. WiFi Configuration

WiFi credentials are set in `platformio.ini` as build flags (or directly in source):

```ini
build_flags =
  -DWIFI_SSID=\"YourNetwork\"
  -DWIFI_PASS=\"YourPassword\"
```

### 2. Supabase Configuration

Keys are stored in `include/flight_logger_config.h`. Do not commit this file with real keys:

```cpp
#define SUPABASE_URL  "https://iigtcugucufmrosrkgjv.supabase.co"
#define SUPABASE_KEY  "<anon-key>"
#define SUPABASE_BUCKET "flight-logs"
```

### 3. Initial Flash (USB required)

The first flash must be done over USB to install the OTA-enabled partition scheme:

```bash
platformio run --target upload --environment rfx-4-main --upload-port COM15
```

Then upload the SPIFFS filesystem (dashboards):

```bash
platformio run --target uploadfs --environment rfx-4-main --upload-port COM15
```

### 4. Verify Connection

Open the serial monitor and confirm:

```
✓ WiFi connected | IP: 192.168.x.x
✓ IMU initialised
✓ Moteus CAN connected
```

Navigate to `http://<ip>/` to confirm the dashboard loads.

---

## Over-The-Air (OTA) Updates

After the initial USB flash, all subsequent firmware updates can be done wirelessly.

### How OTA Works

The ESP32 flash is split into two application partitions (OTA0 and OTA1). The device writes new firmware to the inactive partition. On success, the bootloader switches to the new partition and reboots. If the new firmware fails to boot, the bootloader rolls back to the previous partition automatically.

### Option A: Web Interface

1. Build the firmware: `platformio run --environment rfx-4-main`
2. Open `http://<ip>/update-page` in your browser
3. Click **Select File** and choose `.pio/build/rfx-4-main/firmware.bin`
4. Click **Upload Firmware**
5. Device automatically reboots with the new firmware

### Option B: PlatformIO Upload

Temporarily set `upload_protocol` in `platformio.ini`:

```ini
[env:rfx-4-main-ota]
extends = env:rfx-4-main
upload_protocol = espota
upload_port = 192.168.x.x
```

Then:

```bash
platformio run --target upload --environment rfx-4-main-ota
```

### Option C: cURL

```bash
curl -F "file=@.pio/build/rfx-4-main/firmware.bin" http://<ip>/update
```

### Finding the Device IP

- **Serial monitor**: look for `✓ WiFi connected | IP: xxx.xxx.xxx.xxx`
- **Router admin panel**: look for a device named `esp32` or similar
- **OTA info endpoint**: `curl http://<ip>/ota/info`

### OTA Info Endpoint

```bash
curl http://<ip>/ota/info
```

```json
{
  "current_partition": "OTA0",
  "available_space": 1835008,
  "flash_size": 16777216,
  "version": "1.0.0",
  "build_date": "Mar 26 2026 10:30:45"
}
```

### Performance

- Upload time: ~30–60 seconds for a 2 MB firmware image
- Reboot time: ~5–10 seconds
- No impact on CAN bus or motor control during upload (async server)

---

## Web API Reference

All endpoints are served by ESPAsyncWebServer. CORS headers are enabled on all routes.

| Method | Path | Description |
|--------|------|-------------|
| GET | `/` | Main single-kite dashboard (HTML) |
| GET | `/fleet` | Fleet overview dashboard (HTML) |
| GET | `/status` | Full telemetry JSON snapshot |
| GET | `/fleet/kites` | Fleet roster (MAC, IP, kite_id) |
| GET | `/fleet/status` | Live telemetry from all fleet kites |
| GET | `/logs` | List SPIFFS CSV files + recording status |
| GET | `/motor/velocity?value=f` | Set motor velocity (rev/s) |
| GET | `/motor/stop` | Emergency motor stop |
| GET | `/update-page` | OTA firmware upload UI (HTML) |
| POST | `/update` | OTA firmware upload endpoint |
| GET | `/ota/info` | Partition info + firmware version |

### Example `/status` Response

```json
{
  "velocity": 2.5,
  "torque": 0.18,
  "tension": 1.2,
  "line_length": 45.3,
  "pitch": 22.1,
  "yaw": 5.4,
  "roll": -1.2,
  "autopilot_enabled": true,
  "active_flight_confidence": 0.87,
  "dive_confidence": 0.02,
  "aww_confidence": 0.11,
  "target_line_length": 50.0,
  "slack_detected": false
}
```

---

## Dashboards

### Single-Kite Dashboard (`/`)

Served from `data/dashboard.html`. Features:

- 3D orientation visualiser (Three.js)
- Real-time motor telemetry cards (velocity, torque, tension, line length)
- IMU readings (pitch, yaw, roll)
- Autopilot status (enabled, slack, target length)
- Dive / Away-from-Wind / Active-flight confidence meters
- Tension vs. pitch time-series chart (Chart.js)
- CSV recording controls and download
- OTA firmware update button
- Remote kite monitoring (add kite by IP)

### Fleet Dashboard (`/fleet`)

Served from `data/fleet_dashboard.html`. Features:

- Grid of cards, one per kite in the fleet
- Online/offline status with colour coding
- Live velocity, tension, line length, pitch, yaw per kite
- Motor fault codes
- Polls `/fleet/kites` for roster, then each kite's `/status` directly
- 1.5 s refresh interval

---

## Fleet & Multi-Kite Operation

The fleet uses **ESP-NOW** (2.4 GHz, no router required) for low-latency coordination.

### Host Election

1. On power-up, each kite listens for 3 seconds for an existing host announcement
2. If a host is heard, the kite registers with it
3. If no host is found, the kite self-elects as host
4. Host broadcasts a roster announcement every 2 seconds
5. Kites that miss 3 consecutive announcements (6 s timeout) trigger re-election

### Fleet Message Types

| Message | Direction | Purpose |
|---------|-----------|---------|
| `MSG_FLEET_ANNOUNCE` | Host → all | Heartbeat + roster |
| `MSG_FLEET_REGISTER` | Kite → host | Join request |
| `MSG_FLEET_ACK` | Host → kite | Assigns kite_id (1–4) |
| `MSG_CONTROL` | Remote → kite | Joystick velocity command |
| `MSG_REMOTE_DISCOVER` | Remote → broadcast | Request roster |
| `MSG_FLEET_ROSTER` | Host → remote | Full kite list with MACs |

### Remote Control

The remote transmits joystick commands at 50 Hz via ESP-NOW. On startup it broadcasts `MSG_REMOTE_DISCOVER`, receives the fleet roster, and maps each joystick to a kite by index. Triple-clicking a joystick button activates respool mode override on that kite.

---

## Flight Logging & Supabase

### Logging

- **Trigger**: Starts when `active_flight_confidence ≥ 0.5`
- **Rate**: 30 Hz (one row every 33 ms)
- **Storage**: SPIFFS as `/f0000.csv`, `/f0001.csv`, etc.
- **Columns** (~21): `commanded_vel`, `actual_vel`, `torque`, `tension`, `line_length`, `target`, pitch/yaw/roll, gyro rates, detector confidences, joystick input

### Supabase Upload

After landing (`active_flight_confidence → 0`), the logger waits 10 seconds, then uploads the completed CSV to Supabase Storage:

```
POST https://iigtcugucufmrosrkgjv.supabase.co/storage/v1/object/flight-logs/<filename>
Headers:
  apikey: <anon-key>
  Authorization: Bearer <anon-key>
  Content-Type: text/csv
```

The upload runs on FreeRTOS Core 0 (non-blocking). If WiFi is not available at upload time, the log remains on SPIFFS and can be downloaded manually via the dashboard.

### Manual Download

From the dashboard, click the log filename in the **Logs** panel to download the CSV directly from SPIFFS.

---

## Autopilot

The autopilot runs on Core 1 and targets a set line length using velocity control.

### Line Length Targeting

- Deadband: 1 cm (no command issued within ±1 cm of target)
- Max velocity: 8 rev/s
- Acceleration limit: 1 rev/s²

### Slack Detection

If motor torque drops below **0.015 Nm**, the line is considered slack. The autopilot ramps reel-in speed from 0.5 to 4 rev/s over 2 seconds. Slack clears when torque exceeds **0.025 Nm**.

### Confidence Detectors

Three detectors produce a 0–1 confidence score each cycle. Thresholds are tuneable in `include/detector_params.h`.

| Detector | Active When |
|----------|------------|
| **Dive** | Pitch rate < −5°/s AND tension > 0.8 N |
| **Away-from-Wind (AWW)** | Yaw offset > 90° from estimated wind direction |
| **Active Flight** | Pitch > 5°, line > 1 m, tension > 0.3 N, variation score satisfied |

The `active_flight_confidence` value gates the flight logger (start/stop recording) and the Supabase upload trigger.

---

## Troubleshooting

### OTA Upload Fails — "Not enough space"

The firmware binary is too large for the available partition. Reduce code size or remove debug symbols (`-Os`, `-DCORE_DEBUG_LEVEL=0`).

### Device Disconnects During OTA Upload

WiFi signal dropped mid-transfer. Move closer to the access point and retry. The bootloader will remain on the previous partition since the new one was incomplete.

### Device Won't Boot After OTA

Rare partition corruption. Flash via USB with the known-good firmware:

```bash
platformio run --target upload --environment rfx-4-main --upload-port <COM port>
```

### Can't Find Device IP

After a WiFi reconnect the IP may change. Check the serial monitor or your router's DHCP table. Assigning a static DHCP lease by MAC address prevents this.

### Supabase Upload Fails

- Confirm WiFi is connected at time of landing
- Verify `SUPABASE_KEY` in `include/flight_logger_config.h` is valid and has Storage write permission
- Check SPIFFS has free space (`GET /logs` shows usage)
- Logs remain on SPIFFS if upload fails and can be downloaded manually

### IMU Not Detected

Run the `i2c-scanner` environment to verify the ICM-20948 is on bus address 0x68 or 0x69. Check SDA/SCL wiring (A4/A5).

### Motor Not Responding

- Confirm Moteus is powered and CAN termination is present
- Check MCP2518FD SPI wiring
- Review CAN fault codes in the `/status` JSON (`motor_fault` field)

### Fleet Kites Not Discovering Each Other

- All kites must be on the same WiFi channel (ESP-NOW uses the same channel as the AP)
- Ensure no kite MAC is blocked in the router
- Power-cycle all kites to trigger fresh host election
