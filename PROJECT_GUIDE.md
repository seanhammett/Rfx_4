# RFx-4 Project Guide (Current Iteration)

Last updated: 2026-04-23

This guide reflects the current firmware and dashboard behavior in this repository.

## 1) Project Summary

RFx-4 is a multi-kite control system with:

- A main kite controller on Arduino Nano ESP32 (ESP32-S3)
- A multi-joystick remote on ESP32-S3 (ESP-NOW control)
- Optional kite-mounted IMU transmitter on ESP32-C6 + CodeCell BNO085
- Moteus motor control over CAN (MCP2518FD)
- Live dashboards from SPIFFS
- Flight CSV logging to SPIFFS with optional Supabase upload
- OTA firmware and SPIFFS file updates over WiFi

## 2) Repository Layout (Key Files)

- `src/main.cpp`: Main kite firmware, web API, fleet logic, autopilot integration
- `src/autopilot.h` / `src/autopilot.cpp`: Target-seeking and detector logic
- `src/flight_logger.h` / `src/flight_logger.cpp`: Non-blocking flight logging + upload
- `src/fleet_protocol.h`: Shared ESP-NOW message types
- `src/remote_control.cpp`: 4-joystick remote firmware
- `src/kite_imu.cpp`: CodeCell IMU sender firmware
- `src/imu_handler.h` / `src/imu_handler.cpp`: ICM-20948 filtering
- `include/detector_params.h`: Detector thresholds and rates (single source of truth)
- `include/flight_logger_config.h`: Supabase/logging configuration
- `include/ota_handler.h`: OTA and SPIFFS upload endpoints
- `platformio.ini`: Build environments and dependencies
- `partitions.csv`: Active partition table for `rfx-4-main`
- `data/dashboard.html`: Single-kite dashboard UI
- `data/fleet_dashboard.html`: Fleet dashboard UI

## 3) Build Environments

Defined in `platformio.ini`:

| Environment | Board | Purpose | Source Filter |
|---|---|---|---|
| `rfx-4-main` | `arduino_nano_esp32` | Main kite controller | `main.cpp`, `imu_handler.cpp`, `autopilot.cpp`, `flight_logger.cpp` |
| `remote-control` | `esp32-s3-devkitc-1` | Multi-joystick remote | `remote_control.cpp`, `joystick_handler.cpp` |
| `remote-calibration` | `esp32-s3-devkitc-1` | Joystick calibration utility | `remote_control_calibration.cpp` |
| `i2c-scanner` | `arduino_nano_esp32` | I2C diagnostic scanner | `i2c_scanner.cpp` |
| `kite-imu` | `esp32-c6-devkitc-1` | Kite IMU sender (CodeCell/BNO085) | `kite_imu.cpp` |

Main environment dependencies:

- `pierremolinaro/ACAN2517FD@^2.1.16`
- `mjbots/Moteus@^1.0.2`
- `esp32async/ESPAsyncWebServer@^3.9.4`
- `bblanchon/ArduinoJson@^6.21.3`
- `adafruit/Adafruit ICM20X@^2.0.7`
- `adafruit/Adafruit Unified Sensor@^1.1.15`
- `adafruit/Adafruit BusIO@^1.17.4`

`kite-imu` uses the pioarduino ESP32 platform zip and `CodeCell` from GitHub.

## 4) Partition Layout (Current)

From `partitions.csv` used by `rfx-4-main`:

| Name | Offset | Size | Notes |
|---|---:|---:|---|
| `nvs` | `0x9000` | `0x4000` (16 KB) | NVS storage |
| `otadata` | `0xD000` | `0x2000` (8 KB) | OTA metadata |
| `app0` | `0x10000` | `0x140000` (1.25 MB) | OTA slot 0 |
| `app1` | `0x150000` | `0x140000` (1.25 MB) | OTA slot 1 |
| `spiffs` | `0x290000` | `0xD70000` (~13.44 MB) | Logs and dashboards |

## 5) Runtime Architecture

### Main control loop

- Motion/control cadence: every 8 ms (125 Hz)
- IMU update runs each loop iteration
- Remote commands received by ESP-NOW callback and applied in loop
- Autopilot and detector updates run in the motion block

### Task split

- Main loop/web server/control: Arduino runtime task (main firmware flow)
- Flight logger writer task: pinned to Core 0 via FreeRTOS queue
- SPIFFS writes and Supabase upload logic execute in logger task context

### Data path

1. IMU + Moteus telemetry sampled in main loop
2. Autopilot computes commanded velocity when target-seeking is enabled
3. Safety limits clamp unspool/retract behavior
4. Motor command sent via Moteus over CAN
5. `/status` publishes nested JSON snapshot
6. Flight logger records CSV samples to SPIFFS
7. Completed logs are uploaded when WiFi and Supabase config are available

## 6) Autopilot and Safety (Current Defaults)

Configured in `setup()` from `src/main.cpp` and `src/autopilot.cpp`:

- Max velocity: `8.0 rev/s`
- Accel/decel: `1.0 / 5.0 rev/s^2`
- Deadband: `0.01 m`
- Slack detect hysteresis: enter `< 0.015 Nm`, exit `>= 0.025 Nm`
- Slack reel speed ramp: `0.1 -> 8.0 rev/s` over `4.0 s`

Additional safety limits in `applySafetyLimits()`:

- Unspool blocked when measured torque `< 0.016 Nm`
- Unspool speed ramps with tension until full speed at `0.2 Nm`
- Retraction blocked at or below zero line length unless respool mode is enabled

### Detector thresholds

Defined in `include/detector_params.h` and exposed in `/status`:

- Dive: pitch-rate `< -5 deg/s` and tension `> 0.8 N`
- Away-from-wind: yaw offset `> 90 deg` from estimated wind direction
- Active-flight: pitch `> 5 deg`, line `> 1 m`, tension `> 0.3 N`, variation score threshold

## 7) Fleet and ESP-NOW Protocol

`src/fleet_protocol.h` defines these message types:

- `MSG_CONTROL`
- `MSG_FLEET_ANNOUNCE`
- `MSG_FLEET_REGISTER`
- `MSG_FLEET_ACK`
- `MSG_REMOTE_DISCOVER`
- `MSG_FLEET_ROSTER`
- `MSG_KITE_IMU`

Key behavior:

- Max fleet size: 4 kites
- Host heartbeat interval: 2000 ms
- Host timeout/re-election: 6000 ms
- Discovery window: 3000 ms
- Remote sends per-joystick commands at ~50 Hz (`COMMAND_INTERVAL = 20 ms`)

## 8) Web API Reference

Routes registered in `src/main.cpp` plus OTA routes from `include/ota_handler.h`.

| Method | Path | Purpose |
|---|---|---|
| GET | `/` | Single-kite dashboard HTML (`/dashboard.html`) |
| GET | `/status` | Full nested telemetry JSON |
| GET | `/motor/velocity?value=<float>` | Set direct velocity command |
| GET | `/motor/stop` | Stop motor |
| GET | `/fleet` | Fleet dashboard HTML (`/fleet_dashboard.html`) |
| GET | `/fleet/kites` | Fleet roster (ID/MAC/IP/color) |
| GET | `/fleet/status` | Host + fleet summary JSON |
| GET | `/logs` | Log files and logger status |
| GET | `/logs/upload` | Trigger upload of pending logs |
| GET | `/update-page` | OTA + SPIFFS upload web UI |
| POST | `/update` | Firmware OTA upload |
| POST | `/update-spiffs` | SPIFFS file upload (no reboot) |
| GET | `/ota/info` | OTA partition and build metadata |

Notes:

- CORS header is globally enabled: `Access-Control-Allow-Origin: *`
- `data/fleet_dashboard.html` polls each kite directly at 1.5 s intervals

## 9) Flight Logging and Supabase Upload

### Recording behavior

- Start recording when `active_flight_confidence >= 0.5`
- Record at `30 Hz`
- On confidence drop to `<= 0`, enter cooldown and stop after `10 s`
- File naming on SPIFFS: `/f0000.csv`, `/f0001.csv`, ...

### CSV schema (24 columns)

`Date,Time,Timestamp_ms,Kite_ID,Commanded_Vel_rps,Actual_Vel_rps,Torque_Nm,Tension_N,Line_Length_m,Target_Enabled,Target_Length_m,Remote_Joy,Remote_Active,Pitch_deg,Pitch_Vel_dps,Yaw_deg,Yaw_Vel_dps,Roll_deg,Dive_Conf,AWW_Conf,AF_Conf,Kite_Pitch_deg,Kite_Roll_deg,Kite_IMU_Battery`

### Cloud upload

- Uses HTTPS POST to Supabase Storage bucket from `include/flight_logger_config.h`
- Upload trigger paths:
  - Automatically after recording stops
  - Manually via `GET /logs/upload`
- On upload success, local CSV is deleted; on failure, file is retained on SPIFFS

## 10) Dashboards

### Single-kite dashboard (`data/dashboard.html`)

- Polls `/status`
- 3D orientation view (Three.js)
- Tension/pitch/line-length charts (Chart.js)
- Detector confidence display
- Kite-mounted IMU status (connected, pitch, roll, battery)
- Log listing and upload control
- Firmware update entry point (`/update-page`)

### Fleet dashboard (`data/fleet_dashboard.html`)

- Fetches roster from `/fleet/kites`
- Polls each kite directly at `http://<kite-ip>/status`
- Shows online/offline, velocity, tension, line length, mode, fault, and remote/target-seek status

## 11) Setup Checklist

1. Install PlatformIO in VS Code.
2. Select target environment from `platformio.ini`.
3. Confirm credentials and secrets:
   - WiFi credentials are currently hardcoded in:
     - `src/main.cpp`
     - `src/remote_control.cpp`
     - `src/kite_imu.cpp`
   - Supabase config is in `include/flight_logger_config.h`.
4. Build:

```bash
platformio run -e rfx-4-main
```

5. First flash (USB):

```bash
platformio run -e rfx-4-main -t upload
```

6. Upload dashboard files to SPIFFS:

```bash
platformio run -e rfx-4-main -t uploadfs
```

7. Open serial monitor:

```bash
platformio device monitor -b 115200
```

8. Open dashboard at `http://<device-ip>/`.

## 12) OTA Workflow

After first USB flash, update firmware over WiFi:

1. Build `rfx-4-main`.
2. Open `http://<device-ip>/update-page`.
3. Upload `.pio/build/rfx-4-main/firmware.bin`.
4. Device reboots automatically on success.

You can also update dashboard/static files live from the same page using the SPIFFS uploader (`/update-spiffs`) without reboot.

## 13) Current Risks and Notes

- WiFi credentials and Supabase key are stored in plaintext in source/config.
- OTA endpoints are not authenticated.
- Supabase upload uses `WiFiClientSecure::setInsecure()` (certificate validation disabled).
- `src/remote_control.cpp` header comments still mention `Rfx_3`, but functionality is for RFx-4 fleet mode.

## 14) Quick Troubleshooting

- No IMU detected:
  - Build/upload `i2c-scanner` and verify address `0x68` or `0x69`.
- No remote response:
  - Check ESP-NOW channel alignment (same WiFi channel) and host election logs.
- Motor not responding:
  - Verify MCP2518FD wiring, Moteus power, termination, and fault fields in `/status`.
- OTA fails with size error:
  - Ensure firmware fits in `app0/app1` partition size (`0x140000`).
- Logs not uploading:
  - Confirm WiFi connected and Supabase settings valid; pending files remain in SPIFFS.
