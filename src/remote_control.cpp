// Remote Control ESP32 - Wireless Joystick Controller for Rfx_3
// This program runs on a second ESP32 with a joystick attached
// It sends control commands to the main Rfx_3 system via ESP-NOW (very low latency)
// Compatible with moteus motor controller velocity commands

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "joystick_handler.h"

// =================== PINS ===================
// Set ACTIVE_JOYSTICK to 1, 2, 3, or 4 to select which unit is connected.
// Pin options and calibrated deadbands are defined below.
#define ACTIVE_JOYSTICK  1

#if ACTIVE_JOYSTICK == 1
  #define JOYSTICK_X_PIN    6
  #define JOYSTICK_Y_PIN    7
  #define JOYSTICK_SW_PIN   39
  #define DZ_Y_LOWER  2079
  #define DZ_Y_UPPER  2143
  #define DZ_X_LOWER  2063
  #define DZ_X_UPPER  2124
#elif ACTIVE_JOYSTICK == 2
  #define JOYSTICK_X_PIN    4
  #define JOYSTICK_Y_PIN    5
  #define JOYSTICK_SW_PIN   40
  #define DZ_Y_LOWER  2055
  #define DZ_Y_UPPER  2113
  #define DZ_X_LOWER  2078
  #define DZ_X_UPPER  2137
#elif ACTIVE_JOYSTICK == 3
  #define JOYSTICK_X_PIN    12
  #define JOYSTICK_Y_PIN    10
  #define JOYSTICK_SW_PIN   41
  #define DZ_Y_LOWER  2104
  #define DZ_Y_UPPER  2160
  #define DZ_X_LOWER  2053
  #define DZ_X_UPPER  2111
#elif ACTIVE_JOYSTICK == 4
  #define JOYSTICK_X_PIN    13
  #define JOYSTICK_Y_PIN    11
  #define JOYSTICK_SW_PIN   42
  #define DZ_Y_LOWER  2097
  #define DZ_Y_UPPER  2161
  #define DZ_X_LOWER  2046
  #define DZ_X_UPPER  2101
#else
  #error "ACTIVE_JOYSTICK must be 1, 2, 3, or 4"
#endif

// =================== CONFIG ===================
const char* WIFI_SSID = "iPhone 123";
const char* WIFI_PASS = "sonoma1991";

// MAC address of the main Rfx_3 ESP32-C3 (UPDATE THIS!)
// You'll see this printed in the main system's serial output on boot
// Example format: 34:B4:72:EA:48:3C
uint8_t receiverMAC[] = {0x3C, 0x84, 0x27, 0xFC, 0xC8, 0x9C}; //  MAC: 3C:84:27:FC:C8:9C

// =================== ESP-NOW DATA STRUCTURE ===================
typedef struct {
  int16_t motor_speed;  // Joystick command: -1000 to +1000 range
  uint8_t command;      // 0=speed, 1=unused, 2=stop
  uint8_t button;       // Button state (0=released, 1=pressed)
} ControlMessage;

// =================== STATE ====================
JoystickController joystick(JOYSTICK_X_PIN, JOYSTICK_Y_PIN, JOYSTICK_SW_PIN);

unsigned long lastCommandSent = 0;
const unsigned long COMMAND_INTERVAL = 20;  // Send commands every 20ms (50 Hz) - ESP-NOW is fast!

ControlMessage outgoingMsg;
esp_now_peer_info_t peerInfo;

// =================== ESP-NOW FUNCTIONS ===================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Callback when data is sent
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  
  // Print status every 500ms to avoid spam
  // if (now - lastPrint > 500) {
  //   if (status == ESP_NOW_SEND_SUCCESS) {
  //     Serial.println("TX: SUCCESS");
  //   } else {
  //     Serial.println("TX: FAILED!");
  //   }
  //   lastPrint = now;
  // }
}

void sendMotorCommand(int speed, bool button_pressed) {
  outgoingMsg.motor_speed = speed;
  outgoingMsg.command = 0;  // Velocity command
  outgoingMsg.button = button_pressed ? 1 : 0;
  esp_now_send(receiverMAC, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
}

void sendStopCommand() {
  outgoingMsg.motor_speed = 0;
  outgoingMsg.command = 2;  // Stop
  outgoingMsg.button = 0;
  esp_now_send(receiverMAC, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
}

// =================== SETUP ===================
void setup() {
  // ESP32-S3 USB CDC serial setup
  Serial.begin(115200);
  delay(2000);  // Give time for USB CDC to initialize
  
  Serial.println("\n=== Remote Control Startup ===");
  
  // Initialize built-in RGB LED on GPIO2 (off)
  neopixelWrite(2, 0, 0, 0);
  
  // Configure ADC (ESP32-S3 compatible)
  analogSetPinAttenuation(JOYSTICK_X_PIN, ADC_11db);
  analogSetPinAttenuation(JOYSTICK_Y_PIN, ADC_11db);
  
  // Initialize joystick
  joystick.begin();
  joystick.setDeadzone(DZ_Y_LOWER, DZ_Y_UPPER, DZ_X_LOWER, DZ_X_UPPER);
  joystick.setInvert(true, true);  // Board has X and Y sense inverted
  joystick.setSpeedLimits(-1000, 1000);  // Match main system: ±1000 maps to ±MAX_VELOCITY_RPS
  joystick.setFilterAlpha(0.7);
  Serial.println("Joystick configured!");
  Serial.println("  Range: ±1000 units (matches main system JOYSTICK_MAX)");
  Serial.println("  Button: Toggle target seeking mode on main system");
  
  // Initialize WiFi in STA mode 
  Serial.println("\nInitializing WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  // Wait for WiFi connection (up to 10 seconds)
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
  
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  // Initialize ESP-NOW after WiFi is set up
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("[OK] ESP-NOW initialized!");
  
  // Register send callback
  esp_now_register_send_cb(onDataSent);
  
  // Use WiFi channel if connected, else default to 0 (auto)
  int channel = 0;  // 0 = use primary channel (let ESP handle it)
  if (WiFi.status() == WL_CONNECTED) {
    channel = WiFi.channel();
  }
  
  // Register peer (receiver)
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = channel;  // Sync to WiFi channel if available
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    // Still try to proceed
  } else {
    Serial.println("Peer added successfully!");
  }
  Serial.print("Sending to MAC: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", receiverMAC[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  
  Serial.println("[OK] Remote control ready!");
  Serial.println("  - WiFi and ESP-NOW synced to same channel");
  Serial.println("  - Commands: motor_speed=±1000, button toggles target seeking mode");
}

// =================== MAIN LOOP ===================
void loop() {
  unsigned long loopStart = millis();
  
  joystick.update();
  
  // Get button state and track changes
  static bool prev_button_state = false;
  bool current_switch = joystick.getSwitch();
  bool button_changed = (current_switch != prev_button_state);
  prev_button_state = current_switch;
  
  // Send motor commands with 100ms zero-command period after release
  static unsigned long lastJoystickActiveTime = 0;
  static unsigned long lastButtonChangeTime = 0;
  unsigned long now = millis();
  bool commandSentThisLoop = false;
  int commandSent = 0;
  
  // Default LED off; will be turned on if we send data
  neopixelWrite(2, 0, 0, 0);
  
  // Track button changes
  if (button_changed) {
    lastButtonChangeTime = now;
  }
  
  if (joystick.isInUse()) {
    // Joystick is active - send commands at regular intervals
    lastJoystickActiveTime = now;
    if (now - lastCommandSent >= COMMAND_INTERVAL) {
      lastCommandSent = now;
      commandSent = joystick.getMotorCommand();
      sendMotorCommand(commandSent, current_switch);
      { // LED: brightness scaled by command magnitude, white=positive, red=negative
        uint8_t brightness = map(abs(commandSent), 0, 1000, 0, 255);
        if (commandSent >= 0)
          neopixelWrite(2, brightness, brightness, brightness);  // White
        else
          neopixelWrite(2, brightness, 0, 0);                    // Red
      }
      Serial.printf("TX: %5d  rev/s: %+6.2f  X: %5d  Y: %5d  Btn: %d\n", 
                    commandSent, (commandSent / 1000.0) * 2.0,
                    joystick.getX(), joystick.getY(), current_switch);
      commandSentThisLoop = true;
    }
  } else if (now - lastJoystickActiveTime < 100) {
    // Joystick released less than 100ms ago - keep sending zero
    if (now - lastCommandSent >= COMMAND_INTERVAL) {
      lastCommandSent = now;
      commandSent = 0;
      sendMotorCommand(0, current_switch);
      neopixelWrite(2, 5, 5, 5);  // Dim white for idle send
      Serial.printf("TX:     0  rev/s:  +0.00  idle: %3lums          Btn: %d\n", now - lastJoystickActiveTime, current_switch);
      commandSentThisLoop = true;
    }
  } else if (now - lastButtonChangeTime < 100) {
    // Button changed recently - keep sending to ensure button state is transmitted
    if (now - lastCommandSent >= COMMAND_INTERVAL) {
      lastCommandSent = now;
      commandSent = 0;
      sendMotorCommand(0, current_switch);
      neopixelWrite(2, 5, 5, 5);  // Dim white for button update
      Serial.printf("TX:     0  rev/s:  +0.00  btn update           Btn: %d\n", current_switch);
      commandSentThisLoop = true;
    }
  }
  // After 100ms idle AND no recent button changes, send nothing so main board's local joystick can take over
  
  // Periodically check WiFi status (every 10s)
  static unsigned long lastWiFiCheckTime = 0;
  if (now - lastWiFiCheckTime > 10000) {
    lastWiFiCheckTime = now;
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("WiFi connected | Channel: %d\n", WiFi.channel());
    } else {
      Serial.println("WiFi disconnected (ESP-NOW on default channel 1)");
    }
  }
  
  // Debug output every loop
  static unsigned long lastDebugPrint = 0;
  // if (now - lastDebugPrint >= 50) {  // Print every 50ms
  //   if (joystick.isInUse()) {
  //     Serial.printf("[ACTIVE] X=%d Y=%d Cmd=%d | Sent=%d\n", 
  //                   joystick.getX(), joystick.getY(), joystick.getMotorCommand(), commandSentThisLoop);
  //   } else if (now - lastJoystickActiveTime < 100) {
  //     Serial.printf("[RELEASE] Idle=%lums | Sent=%d\n", 
  //                   now - lastJoystickActiveTime, commandSentThisLoop);
  //   } else {
  //     Serial.printf("[IDLE] Idle=%lums | No TX\n", now - lastJoystickActiveTime);
  //   }
  //   lastDebugPrint = now;
  // }
  
  unsigned long loopElapsed = millis() - loopStart;
  // Serial.printf("Loop: %lums\n", loopElapsed);
  
  delay(5);  // 5ms loop time
}
