# RFx4 OTA (Over-The-Air) Firmware Updates

This project now supports wireless firmware updates via WiFi! No more USB cable needed after the initial flash.

## How It Works

The ESP32 has a built-in OTA (Over-The-Air) update capability that allows you to upload new firmware wirelessly. The board partitions its flash memory into two application areas, allowing safe updates:

- **OTA0 (Boot partition)**: Currently running firmware
- **OTA1 (Update partition)**: New firmware being uploaded
- When upload completes, the bootloader switches to the new partition and reboots

## Getting Started

### 1. **Initial Flash (USB Connection Required)**

First time only, you must flash via USB to set up the OTA partition scheme:

```bash
# Terminal or VS Code task
platformio run --target upload --environment rfx-4-main --upload-port COM10
```

This flashes the firmware with the `large_spiffs_16M` partition scheme that includes OTA support.

### 2. **Connect to WiFi**

The board automatically connects to the WiFi network:
```cpp
const char* WIFI_SSID = "iPhone 123";
const char* WIFI_PASS = "sonoma1991";
```

Check the serial monitor to confirm WiFi connection:
```
✓ WiFi connected | IP: 192.168.1.100
```

### 3. **Upload Firmware Wirelessly**

Once WiFi is connected, you have three options:

#### **Option A: Web Interface (Easiest)**

1. Open your browser and navigate to:
   ```
   http://192.168.1.100/update-page
   ```
   (Replace with your actual board IP from serial monitor)

2. Click **"Select File"** and choose a `.bin` file from:
   ```
   .pio/build/rfx-4-main/firmware.bin
   ```

3. Click **"Upload Firmware"** and wait for the progress bar to complete

4. Device automatically reboots with new firmware

#### **Option B: PlatformIO Upload**

Build the firmware and upload via OTA instead of USB:

```bash
platformio run --target upload --environment rfx-4-main --upload-port 192.168.1.100
```

First, add to `platformio.ini` for this environment (if using OTA):
```ini
[env:rfx-4-main]
upload_protocol = espota
upload_port = 192.168.1.100
upload_flags = --auth=password  # Optional password protection
```

#### **Option C: cURL / Command Line**

```bash
curl -F "file=@.pio/build/rfx-4-main/firmware.bin" http://192.168.1.100/update
```

## Building Firmware for OTA

### Standard Build
```bash
platformio run --environment rfx-4-main
```
Creates: `.pio/build/rfx-4-main/firmware.bin`

### Build Without Upload
```bash
platformio run --environment rfx-4-main
```

The `.bin` file is located at:
```
.pio/build/rfx-4-main/firmware.bin
```

## Finding Your Device IP

### Method 1: Serial Monitor
```
Monitor (rfx-4-main) terminal → Check for "✓ WiFi connected | IP: XXX.XXX.XXX.XXX"
```

### Method 2: Your Router
- Log into your WiFi router's admin page
- Look for connected devices named "esp32-nano" or similar

### Method 3: Scan Network (Linux/Mac)
```bash
arp-scan --localnet | grep -i esp
```

## OTA Information Endpoint

Check device status and update info:

```bash
curl http://192.168.1.100/ota/info
```

Response Example:
```json
{
  "current_partition": "OTA0",
  "available_space": 1835008,
  "flash_size": 16777216,
  "version": "1.0.0",
  "build_date": "Mar 16 2026 10:30:45"
}
```

## Troubleshooting

### Upload Fails - "Not enough space"
- The `.bin` file is too large for the available partition
- Solution: Remove debug symbols or optimize code size

### Device Disconnects During Update
- WiFi connection interrupted
- Solution: Ensure strong WiFi signal, try again

### Device Won't Boot After OTA
- Rare: Corruption during upload
- Solution: Flash via USB with original firmware and retry

### Can't Find Device Web Interface
- IP changed due to WiFi reconnect
- Solution: Check serial monitor for current IP

### Upload Shows Progress But Hangs
- Server-side issue
- Solution: Power cycle the device and retry

## Safety Features

✓ **Automatic Safe Mode**: If new firmware fails to start, bootloader rolls back to previous version  
✓ **Progress Tracking**: Serial output shows upload progress  
✓ **Checksum Verification**: Firmware integrity checked before switching partitions  
✓ **Partition Validation**: Device validates both app partitions before boot  

## Partition Layout

```
Address Range       | Partition      | Size
0x00000000-0x00008000 | Bootloader     | 32 KB
0x00008000-0x00009000 | NVS            | 4 KB
0x00009000-0x00010000 | OTA Data       | 28 KB
0x00010000-0x00200000 | OTA0 (app)     | 2 MB
0x00200000-0x00400000 | OTA1 (app)     | 2 MB
0x00400000-0x00FFB000 | SPIFFS (data)  | 4032 KB
```

Total: 16 MB flash memory

## Reverting to Previous Firmware

There's no automatic rollback button, but you can use:

```bash
# Upload your previous .bin file
curl -F "file=@previous_firmware.bin" http://192.168.1.100/update
```

Keep backup `.bin` files in your project directory.

## Environment Variable for Versioning

Update version in code (currently `1.0.0`):

Edit `include/ota_handler.h`:
```cpp
doc["version"] = "1.0.1";  // Update version string
```

Or use build flags in `platformio.ini`:
```ini
build_flags = 
  -DFIRMWARE_VERSION=\"1.0.1\"
```

## Remote Control OTA Updates

Both `remote-control` and `exhibition-remote` environments can be configured for OTA as well:

1. Update their `platformio.ini` entries to include `board_build.partitions = large_spiffs_16M`
2. They'll automatically inherit OTA capability if they have WiFi enabled

## Performance Notes

- **Upload Time**: ~30-60 seconds for full 2MB firmware (depends on WiFi speed)
- **Reboot Time**: ~5-10 seconds after upload completes
- **Network Overhead**: Minimal impact on CAN bus or motor control

## Security (Optional)

To add password protection to OTA uploads, set in Arduino sketch:

```cpp
ArduinoOTA.setPassword("yourpassword");
```

Or via HTTP auth in web interface (currently unrestricted).

---

**Now you can update firmware wirelessly! No more USB cables required.** 🎉
