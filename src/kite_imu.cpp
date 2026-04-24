// Kite IMU — standalone CodeCell C6 sketch
// Streams BNO085 IMU data from the kite to a paired kite controller via ESP-NOW
// Follows the same pattern as remote_control.cpp

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_pm.h>
#include <CodeCell.h>
#include "fleet_protocol.h"

// =================== CONFIG ===================
const char* WIFI_SSID = "iPhone 123";
const char* WIFI_PASS = "sonoma1991";

// Target kite controller MAC address (unicast)
// Set this to the MAC of the kite controller this CodeCell is paired with.
// Find it from the kite controller's serial output on boot.
static const uint8_t TARGET_KITE_MAC[6] = { 0x3C, 0x84, 0x27, 0xFC, 0xC8, 0x9C }; //  3C:84:27:FC:C8:9C - bodgy board in purple tether

// Transmission rate
static const uint8_t IMU_RATE_HZ = 125;

// =================== GLOBALS ===================
CodeCell myCodeCell;
uint8_t packetSequence = 0;
bool peerRegistered = false;

// Send status tracking
volatile bool lastSendOk = false;
unsigned long lastSendTime = 0;
unsigned long successCount = 0;
unsigned long failCount = 0;

// =================== ESP-NOW CALLBACKS ===================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  lastSendOk = (status == ESP_NOW_SEND_SUCCESS);
}

// ESP32-C6 uses newer ESP-NOW API with esp_now_recv_info
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data, int data_len) {
  // Currently no inbound messages expected, but handler registered for future use
}

// =================== SETUP ===================
void setup() {
  // Reduce CPU from 160 MHz default to 80 MHz — sufficient for IMU + ESP-NOW
  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n=== Kite IMU — CodeCell C6 + BNO085 ===");
  Serial.printf("[OK] CPU frequency: %d MHz\n", getCpuFrequencyMhz());

  // Initialize CodeCell with game rotation only (no mag, no gyro, no accel)
  myCodeCell.Init(MOTION_ROTATION_NO_MAG);
  Serial.println("[OK] CodeCell initialized (BNO085: rotation only)");

  // Connect to WiFi briefly to sync channel, then disconnect.
  // ESP-NOW requires the radio to be on the same channel as the peer;
  // staying associated costs ~40-80 mA continuously — disconnect saves it.
  uint8_t wifiChannel = 1;  // default; overwritten if WiFi connects
  Serial.println("\nInitializing WiFi (channel sync only)...");
  WiFi.mode(WIFI_STA);
  // Reduce TX power to 11 dBm — sufficient for <100 m kite range (default is 19.5 dBm)
  esp_wifi_set_max_tx_power(44);  // units: 0.25 dBm steps → 44 × 0.25 = 11 dBm
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStartTime < 10000)) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifiChannel = (uint8_t)WiFi.channel();
    Serial.printf("[OK] WiFi connected | Channel: %d — disconnecting (channel locked)\n", wifiChannel);
    WiFi.disconnect(false);  // drop AP association; STA mode stays active for ESP-NOW
  } else {
    Serial.printf("[WARNING] WiFi timeout — using default channel %d\n", wifiChannel);
  }
  // Lock radio to the negotiated channel so ESP-NOW peers can find us
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  Serial.printf("MAC Address: %s\n", WiFi.macAddress().c_str());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed!");
    return;
  }
  Serial.println("[OK] ESP-NOW initialized");
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);

  // Register target kite controller as unicast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, TARGET_KITE_MAC, 6);
  peerInfo.channel = wifiChannel;  // explicit channel — required after WiFi.disconnect()
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    peerRegistered = true;
    Serial.printf("[OK] Peer registered: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  TARGET_KITE_MAC[0], TARGET_KITE_MAC[1], TARGET_KITE_MAC[2],
                  TARGET_KITE_MAC[3], TARGET_KITE_MAC[4], TARGET_KITE_MAC[5]);
  } else {
    Serial.println("[ERROR] Failed to register peer");
  }

  // Brief blink to confirm setup complete, then LED off.
  // The kite is airborne — visual feedback is useless and costs 5-15 mA.
  myCodeCell.LED_SetBrightness(10);
  delay(500);
  myCodeCell.LED_SetBrightness(0);
  Serial.printf("\n[OK] Kite IMU ready — streaming at %d Hz\n", IMU_RATE_HZ);

  // Enable dynamic frequency scaling (10-80 MHz) + auto light-sleep.
  // The CPU scales down when idle between myCodeCell.Run() ticks.
  // If ESP-NOW delivery degrades, set light_sleep_enable = false to keep
  // DFS only (still significant saving without modem sleep risk).
  esp_pm_config_t pm_config = {};
  pm_config.max_freq_mhz = 80;
  pm_config.min_freq_mhz = 10;
  pm_config.light_sleep_enable = true;
  if (esp_pm_configure(&pm_config) == ESP_OK) {
    Serial.println("[OK] Power management: DFS 10-80 MHz + light sleep enabled");
  } else {
    Serial.println("[WARNING] Power management config failed — running at fixed 80 MHz");
  }
}

// =================== MAIN LOOP ===================
void loop() {
  if (myCodeCell.Run(IMU_RATE_HZ)) {
    // Read rotation only (pitch & roll)
    float roll, pitch, yaw;
    myCodeCell.Motion_RotationNoMagRead(roll, pitch, yaw);

    // Correct for inverted mounting: rest position reads ±180° instead of 0°.
    // Shift by 180 and wrap to [-180, 180]. If tilting direction is also
    // backwards after this correction, negate the affected axis as well.
    auto wrapTo180 = [](float v) -> float {
      if (v >  180.0f) v -= 360.0f;
      if (v < -180.0f) v += 360.0f;
      return v;
    };
    pitch = wrapTo180(180.0f - pitch);
    roll  = wrapTo180(180.0f - roll);

    // Build packet (pitch + roll only)
    KiteImuMsg msg = {};
    msg.msg_type = MSG_KITE_IMU;
    msg.pitch = pitch;
    msg.roll = roll;
    msg.battery_pct = (uint8_t)myCodeCell.BatteryLevelRead();
    msg.sequence = packetSequence++;

    // Send via ESP-NOW
    if (peerRegistered) {
      esp_now_send(TARGET_KITE_MAC, (uint8_t*)&msg, sizeof(msg));
      lastSendTime = millis();

      if (lastSendOk) {
        successCount++;
      } else {
        failCount++;
      }
    }

    // Periodic serial status (every 5 seconds)
    static unsigned long lastStatusPrint = 0;
    if (millis() - lastStatusPrint >= 5000) {
      lastStatusPrint = millis();
      Serial.printf("[IMU] P%+6.1f R%+6.1f | Batt:%d | OK:%lu Fail:%lu\n",
                    pitch, roll,
                    (int)msg.battery_pct, successCount, failCount);
    }
  } else {
    delay(1);  // yield to FreeRTOS scheduler; pairs with light sleep for idle-time savings
  }
}
