// Kite IMU — standalone CodeCell C6 sketch
// Streams BNO085 IMU data from the kite to a paired kite controller via ESP-NOW
// Follows the same pattern as remote_control.cpp

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <CodeCell.h>
#include "fleet_protocol.h"

// =================== CONFIG ===================
const char* WIFI_SSID = "iPhone 123";
const char* WIFI_PASS = "sonoma1991";

// Target kite controller MAC address (unicast)
// Set this to the MAC of the kite controller this CodeCell is paired with.
// Find it from the kite controller's serial output on boot.
static const uint8_t TARGET_KITE_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Transmission rate
static const uint8_t IMU_RATE_HZ = 50;

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
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\n=== Kite IMU — CodeCell C6 + BNO085 ===");

  // Initialize CodeCell with game rotation (no mag) + gyro + accelerometer
  myCodeCell.Init(MOTION_ROTATION_NO_MAG + MOTION_GYRO + MOTION_ACCELEROMETER);
  Serial.println("[OK] CodeCell initialized (BNO085: rotation + gyro + accel)");

  // Initialize WiFi (STA mode for channel sync with kite controller)
  Serial.println("\nInitializing WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wifiStartTime = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStartTime < 10000)) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[OK] WiFi connected | Channel: %d\n", WiFi.channel());
  } else {
    Serial.println("[WARNING] WiFi connection timeout (ESP-NOW will use default channel)");
  }
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
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    peerRegistered = true;
    Serial.printf("[OK] Peer registered: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  TARGET_KITE_MAC[0], TARGET_KITE_MAC[1], TARGET_KITE_MAC[2],
                  TARGET_KITE_MAC[3], TARGET_KITE_MAC[4], TARGET_KITE_MAC[5]);
  } else {
    Serial.println("[ERROR] Failed to register peer");
  }

  // LED: blue = initializing complete
  myCodeCell.LED_SetBrightness(5);
  Serial.printf("\n[OK] Kite IMU ready — streaming at %d Hz\n", IMU_RATE_HZ);
}

// =================== MAIN LOOP ===================
void loop() {
  if (myCodeCell.Run(IMU_RATE_HZ)) {
    // Read sensor data
    float roll, pitch, yaw;
    float gx, gy, gz;
    float ax, ay, az;

    myCodeCell.Motion_RotationNoMagRead(roll, pitch, yaw);
    myCodeCell.Motion_GyroRead(gx, gy, gz);
    myCodeCell.Motion_AccelerometerRead(ax, ay, az);

    // Build packet
    KiteImuMsg msg = {};
    msg.msg_type = MSG_KITE_IMU;
    msg.roll = roll;
    msg.pitch = pitch;
    msg.yaw = yaw;
    msg.gyro_x = gx;
    msg.gyro_y = gy;
    msg.gyro_z = gz;
    msg.accel_x = ax;
    msg.accel_y = ay;
    msg.accel_z = az;
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

    // LED status: green = streaming OK, red = send failures
    if (lastSendOk && peerRegistered) {
      myCodeCell.LED_SetBrightness(3);  // dim green via Run() default
    }

    // Serial readout every cycle (temporary debug)
    Serial.printf("[IMU] P%+6.1f R%+6.1f Y%+6.1f | G(%+6.1f,%+6.1f,%+6.1f) | A(%+5.2f,%+5.2f,%+5.2f) | Batt:%d | OK:%lu Fail:%lu\n",
                  pitch, roll, yaw, gx, gy, gz, ax, ay, az,
                  (int)msg.battery_pct, successCount, failCount);
  }
}
