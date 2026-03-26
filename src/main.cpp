#include <Arduino.h>
#include <ACAN2517FD.h>
#include <SPI.h>
#include <Moteus.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <esp_partition.h>
#include <SPIFFS.h>
#include "imu_handler.h"
#include "autopilot.h"
#include "ota_handler.h"
#include "fleet_protocol.h"
#include "flight_logger.h"

// ===== Arduino Nano ESP32 Pin Definitions for MCP2518FD (SPI) =====
// Nano ESP32 uses ESP32-S3 with Arduino pin mapping (Dx/Ax macros)
#define MCP2518_SCK     D13  // SPI Clock (GPIO48)    check
#define MCP2518_MOSI    D11  // SPI MOSI (GPIO38)     check
#define MCP2518_MISO    D12  // SPI MISO (GPIO47)     check
#define MCP2518_CS      D10  // Chip Select (GPIO21)  check
#define MCP2518_INT     D2   // Interrupt pin (GPIO5) 

// ===== Built-in RGB LED (active LOW on Nano ESP32) =====
#ifndef LED_BUILTIN
#define LED_BUILTIN     LED_RED   // Fallback to red channel of RGB LED
#endif

// ===== IMU (I2C) Pin Definitions =====
#define I2C_SDA         A4   // I2C Data (GPIO11)
#define I2C_SCL         A5   // I2C Clock (GPIO12)

// ===== CAN Configuration =====
ACAN2517FD can(MCP2518_CS, SPI, MCP2518_INT);
static const uint32_t CAN_BITRATE = 1000000;  // 1 Mbps (adjust as needed)

// ===== Moteus Configuration =====
const int MOTEUS_ID = 1;  // Change this to match your Moteus controller ID

Moteus moteus(can, []() {
  Moteus::Options options;
  options.id = MOTEUS_ID;
  return options;
}());

// ===== IMU =====
IMUHandler imu;

// ===== Autopilot =====
Autopilot autopilot;

// ===== Flight Logger =====
FlightLogger flightLogger;

// ===== Kite Identity (auto-assigned by fleet protocol) =====
char kiteIdStr[16] = "Kite-?";
const char* KITE_ID    = kiteIdStr;
const char* KITE_COLOR = "#ffffff";

// ===== WiFi Configuration =====
const char* WIFI_SSID = "iPhone 123";
const char* WIFI_PASS = "sonoma1991";
AsyncWebServer server(80);
OTAHandler otaHandler;

// ===== ESP-NOW Remote Control =====
LegacyControlMsg remoteControlMsg;
volatile bool newRemoteCommand = false;     // Flag set in ISR
unsigned long lastRemoteCommandTime = 0;    // Track when we last received remote command

// ===== Fleet State =====
bool isFleetHost = false;
uint8_t myKiteId = 0;                       // 0 = unassigned
KiteSlot fleet[MAX_KITES];
uint8_t fleetSize = 0;
unsigned long lastHostAnnounce = 0;          // Timestamp of last announce heard/sent
uint8_t myMAC[6];

// Fleet message buffers (set in ISR, processed in loop)
volatile bool newFleetRegister = false;
FleetRegisterMsg pendingRegister;
volatile bool newRemoteDiscover = false;
FleetDiscoverMsg pendingDiscover;
uint8_t discoverSenderMAC[6];               // MAC of remote that sent discover

// ===== Timing & Control Constants =====
const unsigned long MOTION_INTERVAL = 10;        // Update every 10ms (100Hz)
const unsigned long REMOTE_TIMEOUT_MS = 300;      // Remote considered active for 300ms after last command
const int DEBUG_PRINT_CYCLES = 25;                // Print debug every 25 cycles (500ms at 50Hz)
const float LINE_LENGTH_DEADBAND = 0.01;          // 1cm deadband for target seeking

// ===== Motion Control Variables =====
unsigned long lastMotionUpdate = 0;
const float MAX_VELOCITY_RPS = 8.0;  // Maximum velocity: 8.0 revolutions per second
const float MAX_TORQUE = 2.0;  // Max torque in Nm (adjust as needed)
const float MIN_TENSION_TORQUE = 0.015;  // Minimum torque (Nm) required to allow line extension
const float FULL_TENSION_TORQUE = 0.08;  // Torque (Nm) above which full unspool speed is allowed
const float MIN_UNSPOOL_SPEED = 0.5;     // Minimum unspool speed (rev/s) at MIN_TENSION_TORQUE threshold

// Line length tracking
const float SPOOL_DIAMETER = 68.0;  // Spool diameter in mm
const float SPOOL_RADIUS_M = SPOOL_DIAMETER / 2.0 / 1000.0;  // Spool radius in meters
float line_length = 0.0;  // Unspooled line length in meters

// Line length target (return to this length when no commands)
float line_length_target = 1.0;  // Target line length in meters (positive = extended)
const float line_length_target_velocity = 0.5;  // Max velocity to target in m/s
const float line_length_target_acceleration = 0.25;  // Acceleration to target in m/s^2
float target_velocity_current = 0.0;  // Current velocity for gradual acceleration to target
bool target_seeking_enabled = false;  // Toggle with joystick button

// Joystick range for velocity calculation
const int JOYSTICK_MAX = 1000;  // ±1000 maps to ±MAX_VELOCITY_RPS
float commanded_velocity = 0.0;  // Track commanded velocity for display

// Motor communication state
bool motorResponseReceived = false;  // Track if we've ever received a valid response
float lastReportedTorque = 0.0;      // Track torque to detect sudden loss
unsigned long lastTorqueLossTime = 0;  // Track last torque loss event
uint8_t lastMotorFault = 0;          // Track last fault code to detect changes
unsigned long lastFaultTime = 0;     // Timestamp of last fault

// Respool mode: triple-click remote button to bypass zero line-length safety
bool respool_mode = false;
float motor_position_offset = 0.0;  // Offset applied when respool resets zero point

// ===== Motor Fault Code Descriptions =====
const char* getMotorFaultDescription(uint8_t fault) {
  if (fault == 0) return "No Fault";
  
  // Moteus fault codes (from Moteus documentation)
  static char buffer[128];
  buffer[0] = '\0';
  
  // Build fault string from bit flags with safe concatenation
  if (fault & 0x01) strncat(buffer, "OverVoltage|", sizeof(buffer)-strlen(buffer)-1);
  if (fault & 0x02) strncat(buffer, "Brownout|", sizeof(buffer)-strlen(buffer)-1);
  if (fault & 0x04) strncat(buffer, "OverCurrent|", sizeof(buffer)-strlen(buffer)-1);
  if (fault & 0x08) strncat(buffer, "FET_Temp|", sizeof(buffer)-strlen(buffer)-1);
  if (fault & 0x10) strncat(buffer, "Motor_Temp|", sizeof(buffer)-strlen(buffer)-1);
  if (fault & 0x20) strncat(buffer, "Encoder|", sizeof(buffer)-strlen(buffer)-1);
  if (fault & 0x40) strncat(buffer, "Comm|", sizeof(buffer)-strlen(buffer)-1);
  if (fault & 0x80) strncat(buffer, "Unknown|", sizeof(buffer)-strlen(buffer)-1);
  
  // Remove trailing pipe
  if (buffer[0] != '\0') {
    size_t len = strlen(buffer);
    if (buffer[len-1] == '|') buffer[len-1] = '\0';
  }
  
  if (buffer[0] == '\0') snprintf(buffer, sizeof(buffer), "Unknown(0x%02X)", fault);
  return buffer;
}

// ===== Fleet Helper Functions =====

// Store our own MAC address
void getMyMacAddress() {
  esp_read_mac(myMAC, ESP_MAC_WIFI_STA);
}

// Register broadcast peer for fleet messages
void registerBroadcastPeer() {
  esp_now_peer_info_t peer = {};
  memset(peer.peer_addr, 0xFF, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (!esp_now_is_peer_exist(peer.peer_addr)) {
    esp_now_add_peer(&peer);
  }
}

// Register unicast peer (for sending ACK/roster to a specific MAC)
void registerUnicastPeer(const uint8_t* mac) {
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (!esp_now_is_peer_exist(peer.peer_addr)) {
    esp_now_add_peer(&peer);
  }
}

// Add a kite to the fleet roster (host only). Returns assigned kite_id, 0 if full.
uint8_t addKiteToFleet(const uint8_t* mac, uint32_t ip) {
  // Check if already registered
  for (uint8_t i = 0; i < fleetSize; i++) {
    if (memcmp(fleet[i].mac, mac, 6) == 0) {
      fleet[i].ip_addr = ip;  // update IP
      return fleet[i].kite_id;
    }
  }
  if (fleetSize >= MAX_KITES) return 0;
  KiteSlot& slot = fleet[fleetSize];
  memcpy(slot.mac, mac, 6);
  slot.kite_id = fleetSize + 1;
  slot.ip_addr = ip;
  fleetSize++;
  return slot.kite_id;
}

// Broadcast fleet announce (host heartbeat with roster)
void broadcastFleetAnnounce() {
  FleetAnnounceMsg msg = {};
  msg.msg_type = MSG_FLEET_ANNOUNCE;
  memcpy(msg.host_mac, myMAC, 6);
  msg.fleet_size = fleetSize;
  memcpy(msg.kites, fleet, sizeof(KiteSlot) * fleetSize);
  uint8_t broadcast[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  esp_now_send(broadcast, (uint8_t*)&msg, sizeof(msg));
}

// Handle a pending registration request (host only, called from loop)
void handleFleetRegister() {
  uint8_t id = addKiteToFleet(pendingRegister.requester_mac, pendingRegister.requester_ip);
  if (id == 0) {
    Serial.println("[FLEET] Fleet full, registration rejected");
    return;
  }
  Serial.printf("[FLEET] Registered kite %d (MAC %02X:%02X:%02X:%02X:%02X:%02X)\n",
                id,
                pendingRegister.requester_mac[0], pendingRegister.requester_mac[1],
                pendingRegister.requester_mac[2], pendingRegister.requester_mac[3],
                pendingRegister.requester_mac[4], pendingRegister.requester_mac[5]);

  // Send ACK to the requester
  registerUnicastPeer(pendingRegister.requester_mac);
  FleetAckMsg ack = {};
  ack.msg_type = MSG_FLEET_ACK;
  ack.assigned_id = id;
  ack.fleet_size = fleetSize;
  memcpy(ack.kites, fleet, sizeof(KiteSlot) * fleetSize);
  esp_now_send(pendingRegister.requester_mac, (uint8_t*)&ack, sizeof(ack));

  // Also broadcast updated roster
  broadcastFleetAnnounce();
}

// Handle a remote discover request (host only, called from loop)
void handleRemoteDiscover() {
  Serial.printf("[FLEET] Remote discover from %02X:%02X:%02X:%02X:%02X:%02X (joystick %d)\n",
                discoverSenderMAC[0], discoverSenderMAC[1],
                discoverSenderMAC[2], discoverSenderMAC[3],
                discoverSenderMAC[4], discoverSenderMAC[5],
                pendingDiscover.joystick_id);

  registerUnicastPeer(discoverSenderMAC);
  FleetRosterMsg roster = {};
  roster.msg_type = MSG_FLEET_ROSTER;
  roster.fleet_size = fleetSize;
  memcpy(roster.kites, fleet, sizeof(KiteSlot) * fleetSize);
  esp_now_send(discoverSenderMAC, (uint8_t*)&roster, sizeof(roster));
}

// ===== Motor Mode Descriptions =====
const char* getMotorModeDescription(int mode) {
  switch(mode) {
    case 0: return "Stopped";
    case 1: return "Position";
    case 2: return "Velocity";
    case 3: return "Fault";
    case 4: return "PWM";
    default: return "Unknown";
  }
}
float calculateLineLength(float motor_position) {
  return -((motor_position - motor_position_offset) * PI * SPOOL_DIAMETER / 1000.0);
}

// Check if remote control is currently active
bool isRemoteActive() {
  return (millis() - lastRemoteCommandTime < REMOTE_TIMEOUT_MS);
}

// Process joystick input for either local or remote source
// raw_command: joystick value in range [-JOYSTICK_MAX, +JOYSTICK_MAX]
// source_label: "REMOTE" or "LOCAL" for debug output
void processJoystickInput(int raw_command, const char* source_label) {
  if (target_seeking_enabled) {
    // Adjust target instead of direct velocity control
    float dt = MOTION_INTERVAL / 1000.0;
    float target_adjustment_rate = (raw_command / (float)JOYSTICK_MAX) * line_length_target_velocity;
    // Delegate target adjustment to autopilot
    autopilot.adjustTarget(target_adjustment_rate, dt);
    line_length_target = autopilot.getTargetLineLength();
    static int targetDebugCount = 0;
    if (++targetDebugCount >= DEBUG_PRINT_CYCLES) {
      Serial.printf("%s adjusting target: rate=%.2fm/s new_target=%.2fm\n", source_label, target_adjustment_rate, line_length_target);
      targetDebugCount = 0;
    }
  } else {
    // Direct velocity control
    commanded_velocity = (raw_command / (float)JOYSTICK_MAX) * MAX_VELOCITY_RPS;
    target_velocity_current = 0.0;
    static int directDebugCount = 0;
    if (++directDebugCount >= DEBUG_PRINT_CYCLES) {
      Serial.printf("%s control: cmd=%d vel=%.2f\n", source_label, raw_command, commanded_velocity);
      directDebugCount = 0;
    }
  }
}

// ===== ESP-NOW Callback (fleet-aware dispatcher) =====
void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  // ISR context — keep FAST! Copy data and set flags only.
  if (data_len == 0) return;

  // Legacy 4-byte ControlMessage (backward compat with old remotes)
  if (data_len == sizeof(LegacyControlMsg)) {
    if (!newRemoteCommand) {
      memcpy(&remoteControlMsg, data, sizeof(LegacyControlMsg));
      newRemoteCommand = true;
    }
    return;
  }

  uint8_t msg_type = data[0];

  switch (msg_type) {
    case MSG_CONTROL:
      if (data_len >= sizeof(FleetControlMsg) && !newRemoteCommand) {
        const FleetControlMsg* msg = (const FleetControlMsg*)data;
        remoteControlMsg.motor_speed = msg->motor_speed;
        remoteControlMsg.command = msg->command;
        remoteControlMsg.button = msg->button;
        newRemoteCommand = true;
      }
      break;

    case MSG_FLEET_ANNOUNCE:
      if (data_len >= sizeof(FleetAnnounceMsg) && !isFleetHost) {
        const FleetAnnounceMsg* msg = (const FleetAnnounceMsg*)data;
        lastHostAnnounce = millis();
        fleetSize = msg->fleet_size;
        if (fleetSize > MAX_KITES) fleetSize = MAX_KITES;
        memcpy(fleet, msg->kites, sizeof(KiteSlot) * fleetSize);
      }
      break;

    case MSG_FLEET_REGISTER:
      if (data_len >= sizeof(FleetRegisterMsg) && isFleetHost && !newFleetRegister) {
        memcpy(&pendingRegister, data, sizeof(FleetRegisterMsg));
        newFleetRegister = true;
      }
      break;

    case MSG_REMOTE_DISCOVER:
      if (data_len >= sizeof(FleetDiscoverMsg) && isFleetHost && !newRemoteDiscover) {
        memcpy(&pendingDiscover, data, sizeof(FleetDiscoverMsg));
        memcpy(discoverSenderMAC, mac_addr, 6);
        newRemoteDiscover = true;
      }
      break;

    default:
      break;
  }
}

void setup() {
  // Blink LED 5 times quickly so we know the board is alive
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, LOW);   // ON (active LOW on Nano ESP32)
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);  // OFF
    delay(100);
  }
  
  // Initialize Serial (USB CDC)
  Serial.begin(115200);
  // Wait for USB CDC connection (with timeout so board runs even without monitor)
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 3000)) {
    delay(10);
  }
  delay(3000);  // Brief extra delay for stable connection
  
  Serial.println("\n\n=== Arduino Nano ESP32 + MCP2518FD CAN + IMU System ===");
  
  // Initialize IMU (ICM-20948) — try 0x69 first, fall back to 0x68
  if (!imu.begin(I2C_SDA, I2C_SCL, 0x69)) {
    Serial.println("[WARNING] IMU not found at 0x69, trying 0x68...");
    if (!imu.begin(I2C_SDA, I2C_SCL, 0x68)) {
      Serial.println("[ERROR] IMU not found at 0x68 either. Check wiring and power. System halted.");
      while (1) { delay(1000); }  // Halt execution
    }
  }
  
  // Configure SPI pins
  SPI.begin(MCP2518_SCK, MCP2518_MISO, MCP2518_MOSI);
  Serial.println("SPI initialized");
  
  // Configure CAN controller settings
  // Using 1Mbps for both arbitration and data rate (conservative Arduino-compatible setting)
  // Oscillator: 40MHz (matching hardware)
  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_40MHz, CAN_BITRATE, DataBitRateFactor::x1);
  
  settings.mDriverTransmitFIFOSize = 5;
  settings.mDriverReceiveFIFOSize = 10;
  
  // Configure receive filters to accept ALL frames (promiscuous mode)
  // Without this, MCP2518FD rejects incoming frames!
  ACAN2517FDFilters filters;
  filters.appendPassAllFilter(NULL);  // Accept all incoming CAN frames
  
  // Hold INT pin HIGH (via pull-up) before can.begin() to prevent the MCP2518FD's
  // open-drain INT line from floating LOW on power-up and triggering spurious interrupts
  // during attachInterrupt()'s IPC call, which overflows the IPC task stack.
  pinMode(MCP2518_INT, INPUT_PULLUP);
  delay(10);  // Let the pin settle and chip reach a defined state

  // Initialize CAN controller
  Serial.println("Initializing MCP2518FD...");
  const uint32_t errorCode = can.begin(settings, [] { can.isr(); }, filters);
  
  if (errorCode == 0) {
    Serial.println("[OK] MCP2518FD initialized successfully!");
    Serial.printf("  Bitrate: %u bps\n", CAN_BITRATE);
  } else {
    Serial.print("[ERROR] CAN initialization failed, error code: 0x");
    Serial.println(errorCode, HEX);
    Serial.println("Check wiring and connections! Continuing without CAN...");
  }
  
  Serial.println("\n=== System Initialized ===\n");
  
  // Stop the motor initially and clear any faults
  Serial.println("Attempting Moteus library stop command...");
  if (moteus.SetStop()) {
    motorResponseReceived = true;  // Mark that we have valid motor data
    Serial.println("[OK] Motor responded to stop command");
    const auto& result = moteus.last_result().values;
    Serial.printf("  Mode: %d, Position: %.3f, Fault: 0x%02X\n", 
                  static_cast<int>(result.mode), result.position, result.fault);
  } else {
    Serial.println("[ERROR] No response from motor via Moteus library");
  }
  delay(100);
  
  // ===== WiFi & ESP-NOW Setup =====
  // Initialize WiFi in AP_STA mode (required for both WiFi and ESP-NOW to coexist)
  WiFi.mode(WIFI_AP_STA);
  
  // Initialize ESP-NOW immediately (doesn't require WiFi connection)
  Serial.println("\nInitializing ESP-NOW...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] Error initializing ESP-NOW");
  } else {
    Serial.println("[OK] ESP-NOW initialized!");
    esp_now_register_recv_cb(onDataReceived);
    Serial.println("  ESP-NOW receiver ready!");
  }
  
  // Start WiFi connection attempt in background (non-blocking)
  // This allows ESP-NOW to be ready immediately, regardless of WiFi status
  Serial.println("\nAttempting WiFi connection (in background)...");
  unsigned long wifiStartTime = millis();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  
  // Wait up to 10 seconds for WiFi connection
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStartTime < 10000)) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  // Print WiFi status
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[OK] WiFi connected | IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  MAC: %s\n", WiFi.macAddress().c_str());
  } else {
    Serial.println("[WARNING] WiFi connection timeout (ESP-NOW active, will retry)");
  }
  
  Serial.println("  - ESP-NOW is ready for remote control");

  // ===== Fleet Discovery & Host Election =====
  getMyMacAddress();
  registerBroadcastPeer();
  Serial.printf("[FLEET] My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                myMAC[0], myMAC[1], myMAC[2], myMAC[3], myMAC[4], myMAC[5]);

  // Listen for an existing host announce for DISCOVER_TIMEOUT_MS
  Serial.println("[FLEET] Listening for existing host...");
  unsigned long discoverStart = millis();
  while (millis() - discoverStart < DISCOVER_TIMEOUT_MS) {
    if (lastHostAnnounce != 0) break;  // heard an announce
    delay(50);
  }

  if (lastHostAnnounce != 0) {
    // An existing host was found — register with it
    Serial.println("[FLEET] Host found, sending register request...");
    FleetRegisterMsg reg = {};
    reg.msg_type = MSG_FLEET_REGISTER;
    memcpy(reg.requester_mac, myMAC, 6);
    reg.requester_ip = WiFi.status() == WL_CONNECTED ? (uint32_t)WiFi.localIP() : 0;
    uint8_t broadcast[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    esp_now_send(broadcast, (uint8_t*)&reg, sizeof(reg));

    // Wait briefly for ACK (the ISR updates fleet[] via MSG_FLEET_ANNOUNCE)
    delay(500);

    // Find our kite_id in the roster
    for (uint8_t i = 0; i < fleetSize; i++) {
      if (memcmp(fleet[i].mac, myMAC, 6) == 0) {
        myKiteId = fleet[i].kite_id;
        break;
      }
    }
    if (myKiteId == 0) {
      // Host didn't ACK in time — fall back to self-election
      Serial.println("[FLEET] No ACK received, self-electing as host");
    } else {
      Serial.printf("[FLEET] Joined fleet as kite %d\n", myKiteId);
    }
  }

  if (myKiteId == 0) {
    // No host heard, or ACK failed — become host
    isFleetHost = true;
    myKiteId = addKiteToFleet(myMAC,
                              WiFi.status() == WL_CONNECTED ? (uint32_t)WiFi.localIP() : 0);
    lastHostAnnounce = millis();
    broadcastFleetAnnounce();
    Serial.printf("[FLEET] Self-elected as HOST, kite_id=%d\n", myKiteId);
  }

  // Set display identity from kite_id
  snprintf(kiteIdStr, sizeof(kiteIdStr), "Kite-%d", myKiteId);
  KITE_COLOR = (myKiteId < 5) ? KITE_COLORS[myKiteId] : KITE_COLORS[0];
  Serial.printf("[FLEET] Identity: %s  Color: %s  Host: %s\n",
                kiteIdStr, KITE_COLOR, isFleetHost ? "YES" : "NO");

  
  // ===== SPIFFS Setup =====
  if (!SPIFFS.begin(false)) {
    Serial.println("[WARNING] SPIFFS Mount Failed, attempting format...");
    if (SPIFFS.format()) {
      Serial.println("[OK] SPIFFS formatted successfully");
      if (SPIFFS.begin()) {
        Serial.println("[OK] SPIFFS mounted after format");
      } else {
        Serial.println("[ERROR] SPIFFS still failed after format");
      }
    } else {
      Serial.println("[ERROR] SPIFFS format failed");
    }
  } else {
    Serial.println("[OK] SPIFFS mounted successfully");
  }

  // ===== Flight Logger Setup =====
  flightLogger.begin(myKiteId);
  Serial.println("[OK] Flight logger initialized");
  
  // ===== Web Server Setup =====
  // CORS headers so dashboard served from one kite can reach the others
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/dashboard.html", "text/html");
  });
  
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(3072);
    bool remoteActive = isRemoteActive();
    
    if (motorResponseReceived) {
      const auto& result = moteus.last_result().values;
      
      doc["motor"]["commanded_velocity"] = commanded_velocity;
      doc["motor"]["actual_velocity"] = result.velocity;
      doc["motor"]["position"] = result.position;
      doc["motor"]["torque"] = result.torque;
      doc["motor"]["tension"] = abs(result.torque) / SPOOL_RADIUS_M;
      doc["motor"]["mode"] = static_cast<int>(result.mode);
      doc["motor"]["mode_name"] = getMotorModeDescription(static_cast<int>(result.mode));
      doc["motor"]["fault"] = result.fault;
      doc["motor"]["fault_desc"] = getMotorFaultDescription(result.fault);
    } else {
      doc["motor"]["commanded_velocity"] = commanded_velocity;
      doc["motor"]["actual_velocity"] = 0;
      doc["motor"]["position"] = 0;
      doc["motor"]["torque"] = 0;
      doc["motor"]["tension"] = 0;
      doc["motor"]["mode"] = 0;
      doc["motor"]["mode_name"] = "Unknown";
      doc["motor"]["fault"] = 0;
      doc["motor"]["fault_desc"] = "No Connection";
    }
    
    doc["kite_id"]    = KITE_ID;
    doc["kite_color"]  = KITE_COLOR;
    doc["line_length"] = line_length;
    doc["target_seeking_enabled"] = target_seeking_enabled;
    doc["line_length_target"] = line_length_target;
    doc["autopilot"]["enabled"] = autopilot.isEnabled();
    doc["autopilot"]["slack"] = autopilot.isSlackDetected();
    doc["autopilot"]["target"] = autopilot.getTargetLineLength();
    doc["autopilot"]["max_velocity"] = MAX_VELOCITY_RPS;
    doc["autopilot"]["accel"] = 1.0;
    doc["autopilot"]["decel"] = 5.0;
    doc["autopilot"]["deadband"] = LINE_LENGTH_DEADBAND;
    doc["autopilot"]["slack_min_torque"] = 0.015;
    doc["autopilot"]["slack_max_torque"] = 0.025;
    
    doc["joystick"]["remote_command"] = remoteControlMsg.motor_speed;
    doc["joystick"]["remote_active"] = remoteActive;
    
    doc["imu"]["yaw"] = imu.yaw;
    doc["imu"]["pitch"] = imu.pitch;
    doc["imu"]["roll"] = imu.roll;
    doc["imu"]["yaw_velocity"] = imu.yaw_velocity;
    doc["imu"]["pitch_velocity"] = imu.pitch_velocity;
    
    // Detectors
    const DetectorState& detectors = autopilot.getDetectors();
    doc["detectors"]["dive_confidence"] = detectors.dive_confidence;
    doc["detectors"]["dive_pitch_rate_threshold"] = DIVE_PITCH_RATE_THRESHOLD;
    doc["detectors"]["dive_tension_threshold"] = DIVE_TENSION_THRESHOLD;
    doc["detectors"]["dive_attack_rate"] = DIVE_ATTACK_RATE;
    doc["detectors"]["dive_decay_rate"] = DIVE_DECAY_RATE;
    doc["detectors"]["dive_pitch_velocity_alpha"] = DIVE_PITCH_VELOCITY_ALPHA;
    doc["detectors"]["aww_confidence"] = detectors.aww_confidence;
    doc["detectors"]["aww_wind_direction"] = detectors.wind_direction_deg;
    doc["detectors"]["aww_angle_offset"] = detectors.aww_angle_offset_deg;
    doc["detectors"]["aww_angle_threshold"] = AWW_ANGLE_THRESHOLD;
    doc["detectors"]["aww_attack_rate"] = AWW_ATTACK_RATE;
    doc["detectors"]["aww_decay_rate"] = AWW_DECAY_RATE;
    doc["detectors"]["aww_wind_alpha"] = AWW_WIND_ALPHA;
    doc["detectors"]["active_flight_confidence"] = detectors.active_flight_confidence;
    doc["detectors"]["af_pitch_min"] = AF_PITCH_MIN;
    doc["detectors"]["af_line_length_min"] = AF_LINE_LENGTH_MIN;
    doc["detectors"]["af_tension_min"] = AF_TENSION_MIN;
    doc["detectors"]["af_variation_threshold"] = AF_VARIATION_THRESHOLD;
    doc["detectors"]["af_attack_rate"] = AF_ATTACK_RATE;
    doc["detectors"]["af_decay_rate"] = AF_DECAY_RATE;

    doc["flight_logger"]["recording"] = flightLogger.isRecording();
    doc["flight_logger"]["uploading"] = flightLogger.isUploading();
    doc["flight_logger"]["samples"] = flightLogger.getSampleCount();
    doc["flight_logger"]["bytes"] = flightLogger.getFileBytes();
    
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });
  
  // Motor velocity control endpoint
  server.on("/motor/velocity", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      float velocity = request->getParam("value")->value().toFloat();
      
      Moteus::PositionMode::Command cmd;
      cmd.position = NaN;
      cmd.velocity = velocity;
      cmd.maximum_torque = MAX_TORQUE;
      
      Moteus::PositionMode::Format fmt;
      fmt.position = Moteus::kFloat;
      fmt.velocity = Moteus::kFloat;
      fmt.maximum_torque = Moteus::kFloat;
      
      moteus.SetPosition(cmd, &fmt);
      
      request->send(200, "application/json", "{\"status\":\"ok\",\"velocity\":" + String(velocity) + "}");
    } else {
      request->send(400, "application/json", "{\"error\":\"missing value parameter\"}");
    }
  });
  
  // Stop command
  server.on("/motor/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    moteus.SetStop();
    request->send(200, "application/json", "{\"status\":\"stopped\"}");
  });

  // ===== Fleet Endpoints =====
  // /fleet — fleet dashboard HTML (served from SPIFFS)
  server.on("/fleet", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/fleet_dashboard.html", "text/html");
  });

  // /fleet/kites — simple roster list
  server.on("/fleet/kites", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(512);
    doc["fleet_size"] = fleetSize;
    JsonArray arr = doc.createNestedArray("kites");
    char mac_str[18];
    for (uint8_t i = 0; i < fleetSize; i++) {
      JsonObject k = arr.createNestedObject();
      k["kite_id"] = fleet[i].kite_id;
      snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
               fleet[i].mac[0], fleet[i].mac[1], fleet[i].mac[2],
               fleet[i].mac[3], fleet[i].mac[4], fleet[i].mac[5]);
      k["mac"] = mac_str;
      IPAddress ip(fleet[i].ip_addr);
      k["ip"] = ip.toString();
      k["color"] = (fleet[i].kite_id < 5) ? KITE_COLORS[fleet[i].kite_id] : "#ffffff";
    }
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  // /fleet/status — roster + local telemetry (browser fetches remote kites directly)
  server.on("/fleet/status", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(2048);
    doc["is_host"] = isFleetHost;
    doc["fleet_size"] = fleetSize;
    JsonArray kitesArr = doc.createNestedArray("kites");

    for (uint8_t i = 0; i < fleetSize; i++) {
      JsonObject kObj = kitesArr.createNestedObject();
      kObj["kite_id"] = fleet[i].kite_id;
      char mac_str[18];
      snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
               fleet[i].mac[0], fleet[i].mac[1], fleet[i].mac[2],
               fleet[i].mac[3], fleet[i].mac[4], fleet[i].mac[5]);
      kObj["mac"] = mac_str;
      IPAddress ip(fleet[i].ip_addr);
      kObj["ip"] = ip.toString();
      kObj["color"] = (fleet[i].kite_id < 5) ? KITE_COLORS[fleet[i].kite_id] : "#ffffff";

      // Local kite — inline live telemetry
      if (memcmp(fleet[i].mac, myMAC, 6) == 0) {
        kObj["online"] = true;
        JsonObject t = kObj.createNestedObject("telemetry");
        if (motorResponseReceived) {
          const auto& r = moteus.last_result().values;
          t["velocity"] = r.velocity;
          t["torque"] = r.torque;
          t["tension"] = abs(r.torque) / SPOOL_RADIUS_M;
          t["position"] = r.position;
          t["mode"] = getMotorModeDescription(static_cast<int>(r.mode));
          t["fault"] = r.fault;
        }
        t["line_length"] = line_length;
        t["target_seeking"] = target_seeking_enabled;
        t["line_length_target"] = line_length_target;
        t["pitch"] = imu.pitch;
        t["yaw"] = imu.yaw;
        t["remote_active"] = isRemoteActive();
      } else {
        // Remote kite — browser polls directly; mark online if IP is known
        kObj["online"] = (fleet[i].ip_addr != 0);
      }
    }

    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });
  
  // ===== Flight Log Endpoints =====
  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request){
    DynamicJsonDocument doc(2048);
    doc["recording"] = flightLogger.isRecording();
    doc["uploading"] = flightLogger.isUploading();
    doc["samples"] = flightLogger.getSampleCount();
    doc["bytes"] = flightLogger.getFileBytes();
    JsonArray filesArr = doc.createNestedArray("files");
    File root = SPIFFS.open("/");
    if (root) {
      File f = root.openNextFile();
      while (f) {
        String name = f.name();
        size_t sz = f.size();
        f.close();
        if (name.endsWith(".csv") && (name.startsWith("/f") || name.startsWith("f"))) {
          JsonObject fo = filesArr.createNestedObject();
          fo["name"] = name;
          fo["size"] = sz;
        }
        f = root.openNextFile();
      }
    }
    String response;
    serializeJson(doc, response);
    request->send(200, "application/json", response);
  });

  server.on("/logs/upload", HTTP_GET, [](AsyncWebServerRequest *request){
    flightLogger.uploadPending();
    request->send(200, "application/json", "{\"status\":\"upload_started\"}");
  });

  // Catch-all 404 handler
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "NOT FOUND: " + String(request->url()));
  });
  
  // ===== OTA Firmware Update Setup =====
  // Must be registered BEFORE server.begin() so routes are available immediately
  otaHandler.begin(server);
  Serial.println("[OK] OTA update system initialized");

  server.begin();
  Serial.println("[OK] Web server started");

  // ===== Autopilot Setup (all speeds in rev/s, accelerations in rev/s²) =====
  autopilot.setSpoolDiameter(SPOOL_DIAMETER);
  autopilot.setTargetLineLength(line_length_target);
  autopilot.setSlackTorqueThreshold(0.015, 0.025);  // Enter slack below 0.015 Nm, exit above 0.025 Nm
  autopilot.setSlackReelSpeed(0.1, MAX_VELOCITY_RPS, 4.0);       // min 0.5 rev/s → max 8.0 rev/s over 0.5 seconds
  autopilot.setMaxVelocity(MAX_VELOCITY_RPS);        // Same max as manual control (8.0 rev/s)
  autopilot.setAcceleration(1.0, 5.0);               // accel 1.0 rev/s², decel 5.0 rev/s²
  autopilot.setDeadband(LINE_LENGTH_DEADBAND);
  autopilot.setTorqueFilterAlpha(0.3);
  Serial.println("[OK] Autopilot configured (disabled by default)");

  Serial.println("\n=== System Ready ===");
}

// ===== Toggle target seeking (shared by local and remote buttons) =====
void toggleTargetSeeking(const char* source) {
  target_seeking_enabled = !target_seeking_enabled;
  if (target_seeking_enabled) {
    // Use current line length as the autopilot target
    line_length_target = line_length;
    autopilot.setTargetLineLength(line_length_target);
    autopilot.enable();
    Serial.printf("[OK] Target seeking ENABLED via %s (target: %.2f m = current length)\n", source, line_length_target);
  } else {
    autopilot.disable();
    Serial.printf("[ERROR] Target seeking DISABLED via %s\n", source);
    target_velocity_current = 0.0;
  }
}

// ===== Process incoming ESP-NOW remote commands =====
// Multi-click state lives outside the function so resolution runs every loop iteration
static bool prev_remote_button = false;
static uint8_t remote_click_count = 0;
static unsigned long remote_last_click_time = 0;
static const unsigned long REMOTE_MULTI_CLICK_WINDOW = 400;  // ms between clicks

void processRemoteCommands() {
  // --- Process new ESP-NOW packet (edge detection) ---
  if (newRemoteCommand) {
    newRemoteCommand = false;
    lastRemoteCommandTime = millis();
    
    Serial.printf("ESP-NOW RX: cmd=%d speed=%d btn=%d\n",
                  remoteControlMsg.command, remoteControlMsg.motor_speed, remoteControlMsg.button);
    
    bool current_remote_button = (remoteControlMsg.button != 0);
    // Detect rising edge (button just pressed)
    if (current_remote_button && !prev_remote_button) {
      remote_click_count++;
      remote_last_click_time = millis();
    }
    prev_remote_button = current_remote_button;
    
    // Handle stop command
    if (remoteControlMsg.command == 2) {
      Serial.println("Remote STOP command");
      moteus.SetStop();
    }
  }

  // --- Resolve click sequence (runs every loop, not gated on new packet) ---
  if (remote_click_count > 0 && !prev_remote_button &&
      (millis() - remote_last_click_time > REMOTE_MULTI_CLICK_WINDOW)) {
    uint8_t clicks = min((uint8_t)3, remote_click_count);
    remote_click_count = 0;

    if (clicks == 1) {
      toggleTargetSeeking("REMOTE");
    } else if (clicks == 3) {
      respool_mode = !respool_mode;
      if (!respool_mode && motorResponseReceived) {
        // Exiting respool: save current motor position as new zero
        const auto& r = moteus.last_result().values;
        motor_position_offset = r.position;
        line_length = 0.0;
        line_length_target = 0.0;
        autopilot.setTargetLineLength(0.0);
        Serial.printf("[OK] New zero set at motor position %.3f rev\n", motor_position_offset);
      }
      Serial.printf("%s Respool mode %s (zero-length safety %s)\n",
                    respool_mode ? "[CHARGING]" : "[OK]",
                    respool_mode ? "ENABLED" : "DISABLED",
                    respool_mode ? "BYPASSED" : "active");
    }
  }
}

// ===== Target seeking: compute velocity to reach line_length_target =====
void updateTargetSeeking() {
  if (line_length_target < 0.0) {
    line_length_target = 0.0;
  }
  
  const auto& result_temp = moteus.last_result().values;
  float current_line_length = calculateLineLength(result_temp.position);
  float line_length_error = line_length_target - current_line_length;
  
  // Calculate desired velocity in m/s (positive = retract, negative = unspool)
  float desired_velocity_mps = 0.0;
  if (abs(line_length_error) > LINE_LENGTH_DEADBAND) {
    // v = sqrt(2*a*d) — maximum safe velocity to avoid overshoot
    float distance_to_target = abs(line_length_error);
    float max_safe_velocity = sqrt(2.0 * line_length_target_acceleration * distance_to_target);
    float max_vel = min(max_safe_velocity, line_length_target_velocity);
    
    // Direction: positive error → extend more (negative velocity)
    desired_velocity_mps = (line_length_error > 0) ? -max_vel : max_vel;
  }
  
  // Apply acceleration limit (gradual change)
  float dt = MOTION_INTERVAL / 1000.0;
  float max_velocity_change = line_length_target_acceleration * dt;
  float velocity_error = desired_velocity_mps - target_velocity_current;
  
  if (abs(velocity_error) > max_velocity_change) {
    target_velocity_current += (velocity_error > 0) ? max_velocity_change : -max_velocity_change;
  } else {
    target_velocity_current = desired_velocity_mps;
  }
  
  // Convert from m/s to rev/s
  float meters_per_revolution = PI * SPOOL_DIAMETER / 1000.0;
  commanded_velocity = target_velocity_current / meters_per_revolution;
}

// ===== Safety checks: tension and over-retraction =====
void applySafetyLimits() {
  if (!motorResponseReceived) return;
  
  const auto& result = moteus.last_result().values;
  
  // Tension-based unspool speed limiting:
  // Below MIN_TENSION_TORQUE: block completely
  // Between MIN and FULL_TENSION_TORQUE: ramp max allowed speed from MIN_UNSPOOL_SPEED to full
  // Above FULL_TENSION_TORQUE: allow full commanded speed
  if (commanded_velocity < 0) {
    float torque = abs(result.torque);
    if (torque < MIN_TENSION_TORQUE) {
      static unsigned long lastTensionWarning = 0;
      if (millis() - lastTensionWarning > 1000) {
        Serial.printf("[WARNING] BLOCKED unspool: torque too low (%.3f Nm < %.3f Nm)\n",
                      torque, MIN_TENSION_TORQUE);
        lastTensionWarning = millis();
      }
      commanded_velocity = 0.0;
    } else if (torque < FULL_TENSION_TORQUE) {
      // Linearly scale max allowed unspool speed with tension
      float t = (torque - MIN_TENSION_TORQUE) / (FULL_TENSION_TORQUE - MIN_TENSION_TORQUE);
      float max_speed = MIN_UNSPOOL_SPEED + t * (MAX_VELOCITY_RPS - MIN_UNSPOOL_SPEED);
      if (abs(commanded_velocity) > max_speed) {
        commanded_velocity = -max_speed;
      }
      static unsigned long lastRampInfo = 0;
      if (millis() - lastRampInfo > 1000) {
        Serial.printf("[SPEED] Unspool speed limited: torque=%.3f Nm, max_speed=%.1f rev/s (%.0f%%)\\n",
                      torque, max_speed, t * 100.0);
        lastRampInfo = millis();
      }
    }
    // else torque >= FULL_TENSION_TORQUE: allow full commanded speed
  }
  
  // Block retraction if line length is at or below zero (bypassed in respool mode)
  float current_line_length = calculateLineLength(result.position);
  if (commanded_velocity > 0 && current_line_length <= 0.0 && !respool_mode) {
    static unsigned long lastRetractionWarning = 0;
    if (millis() - lastRetractionWarning > 1000) {
      Serial.printf("[WARNING] BLOCKED over-retraction: line length at minimum (%.3f m)\n", current_line_length);
      lastRetractionWarning = millis();
    }
    commanded_velocity = 0.0;
  }
}

// ===== Send velocity command to motor and handle response =====
void sendMotorCommand() {
  Moteus::PositionMode::Command cmd;
  cmd.position = NaN;
  cmd.velocity = commanded_velocity;
  cmd.maximum_torque = MAX_TORQUE;
  cmd.watchdog_timeout = 0.5;  // 500ms timeout — survives SPIFFS reads during page refresh
  
  Moteus::PositionMode::Format fmt;
  fmt.position = Moteus::kFloat;
  fmt.velocity = Moteus::kFloat;
  fmt.maximum_torque = Moteus::kFloat;
  fmt.watchdog_timeout = Moteus::kFloat;
  
  bool gotResponse = moteus.SetPosition(cmd, &fmt);
  
  if (gotResponse) {
    motorResponseReceived = true;
    const auto& result = moteus.last_result().values;
    
    // Detect fault changes
    if (result.fault != lastMotorFault) {
      lastMotorFault = result.fault;
      lastFaultTime = millis();
      if (result.fault != 0) {
        Serial.printf("[ERROR] MOTOR FAULT DETECTED: 0x%02X (%s)\n", result.fault, getMotorFaultDescription(result.fault));
        Serial.printf("  Mode: %s, Torque: %.3f Nm, Velocity: %.2f rev/s\n",
                      getMotorModeDescription(static_cast<int>(result.mode)), result.torque, result.velocity);
        if (result.fault & 0x02) {
          Serial.println("  [POSSIBLE CAUSE] Brownout - Power supply may be sagging. Check USB power or motor supply voltage.");
        }
        if (result.fault & 0x10) {
          Serial.println("  [POSSIBLE CAUSE] Motor overheating. Check thermal conditions.");
        }
        if (result.fault & 0x04) {
          Serial.println("  [POSSIBLE CAUSE] Over-current. Check for motor jam or high load.");
        }
      }
    }
    
    // Detect torque loss: we commanded motion but got no torque
    if (commanded_velocity != 0.0 && result.torque == 0.0 && lastReportedTorque != 0.0) {
      if (millis() - lastTorqueLossTime > 1000) {
        Serial.printf("[ALERT] TORQUE LOSS DETECTED: cmd_vel=%.2f rev/s, torque=0.0 Nm, mode=%s, fault=0x%02X\n",
                      commanded_velocity, getMotorModeDescription(static_cast<int>(result.mode)), result.fault);
        lastTorqueLossTime = millis();
      }
    }
    lastReportedTorque = result.torque;
    
  } else {
    static unsigned long lastWarning = 0;
    if (millis() - lastWarning > 1000) {
      lastWarning = millis();
      Serial.println("[WARNING] No response from motor - check CAN connection");
    }
  }
}

// ===== Print status line to serial =====
void printStatus() {
  if (!motorResponseReceived) {
    Serial.printf("Waiting for motor response... (cmd: %+5.2f rev/s)\n", commanded_velocity);
    return;
  }
  
  const auto& result = moteus.last_result().values;
  line_length = calculateLineLength(result.position);
  float tension = abs(result.torque) / SPOOL_RADIUS_M;
  bool remoteActive = isRemoteActive();
  
  // Highlight torque loss condition
  const char* torque_status = "";
  if (commanded_velocity != 0.0 && result.torque == 0.0) {
    torque_status = " ⚠️TORQUE_LOSS";
  }
  
  char status_line[512];
  int pos = snprintf(status_line, sizeof(status_line),
                    "Vel%+5.2f/%+5.2f | Tq%+.3f | Tn%.1fN%s | L%6.2fm | Joy:R%+4d%s",
                    commanded_velocity, result.velocity, result.torque, tension, torque_status, line_length,
                    remoteControlMsg.motor_speed,
                    remoteActive ? "*" : "");
  
  if (target_seeking_enabled) {
    float target_error = line_length_target - line_length;
    pos += snprintf(status_line + pos, sizeof(status_line) - pos, " | Tgt%+.2fm(Δ%+.2f)%s",
                   line_length_target, target_error,
                   autopilot.isSlackDetected() ? " SLACK" : "");
  }
  if (respool_mode) {
    pos += snprintf(status_line + pos, sizeof(status_line) - pos, " | RESPOOL");
  }
  
  if (result.fault != 0) {
    pos += snprintf(status_line + pos, sizeof(status_line) - pos, " | FAULT=0x%02X", result.fault);
  }
  
  snprintf(status_line + pos, sizeof(status_line) - pos,
          " | P%+5.1f*(%+4.1f*/s) Y%+5.1f*(%+4.1f*/s)",
          imu.pitch, imu.pitch_velocity, imu.yaw, imu.yaw_velocity);
  
  Serial.println(status_line);
}

void loop() {
  // ===== Fleet Host Duties =====
  if (isFleetHost) {
    // Periodic heartbeat broadcast
    if (millis() - lastHostAnnounce >= FLEET_ANNOUNCE_INTERVAL) {
      broadcastFleetAnnounce();
      lastHostAnnounce = millis();
    }
    // Process pending kite registration
    if (newFleetRegister) {
      newFleetRegister = false;
      handleFleetRegister();
    }
    // Process pending remote discover
    if (newRemoteDiscover) {
      newRemoteDiscover = false;
      handleRemoteDiscover();
    }
  } else {
    // Non-host: detect host timeout and re-elect
    if (lastHostAnnounce != 0 && millis() - lastHostAnnounce > HOST_TIMEOUT_MS) {
      Serial.println("[FLEET] Host timeout — self-electing as new host");
      isFleetHost = true;
      fleetSize = 0;
      myKiteId = addKiteToFleet(myMAC,
                                WiFi.status() == WL_CONNECTED ? (uint32_t)WiFi.localIP() : 0);
      snprintf(kiteIdStr, sizeof(kiteIdStr), "Kite-%d", myKiteId);
      KITE_COLOR = (myKiteId < 5) ? KITE_COLORS[myKiteId] : KITE_COLORS[0];
      lastHostAnnounce = millis();
      broadcastFleetAnnounce();
    }
  }

  processRemoteCommands();
  imu.update();
  
  // ===== Motion Control (50Hz) =====
  if (millis() - lastMotionUpdate >= MOTION_INTERVAL) {
    lastMotionUpdate = millis();
    
    bool remoteActive = isRemoteActive();
    
    // Process remote joystick command
    if (remoteActive && remoteControlMsg.command == 0) {
      processJoystickInput(remoteControlMsg.motor_speed, "REMOTE");
    }
    
    // Target seeking / autopilot
    if (target_seeking_enabled && motorResponseReceived) {
      const auto& ap_result = moteus.last_result().values;
      float ap_line = calculateLineLength(ap_result.position);
      float ap_torque = fabs(ap_result.torque);
      float ap_dt = MOTION_INTERVAL / 1000.0;
      commanded_velocity = autopilot.update(ap_line, ap_torque, ap_dt);
      // Keep shared state in sync
      line_length_target = autopilot.getTargetLineLength();
    } else if (remoteActive && remoteControlMsg.command == 0) {
      // Remote input is active - process joystick (already done above, just continue with this velocity)
      // Do nothing here - velocity already set by processJoystickInput()
    }
    // Otherwise: keep last commanded_velocity (don't zero it out!)
    
    // Update detectors (always run regardless of autopilot state)
    if (motorResponseReceived) {
      const auto& det_result = moteus.last_result().values;
      float det_tension = fabs(det_result.torque) / SPOOL_RADIUS_M;
      float det_dt = MOTION_INTERVAL / 1000.0;
      autopilot.updateDetectors(imu.pitch, imu.pitch_velocity, imu.yaw, det_tension, line_length, det_dt);

      // Update flight logger
      const DetectorState& det = autopilot.getDetectors();
      FlightSample fs = {};
      fs.commanded_vel_rps = commanded_velocity;
      fs.actual_vel_rps = det_result.velocity;
      fs.torque_nm = det_result.torque;
      fs.tension_n = det_tension;
      fs.line_length_m = line_length;
      fs.target_enabled = target_seeking_enabled;
      fs.target_length_m = line_length_target;
      fs.remote_joy = remoteControlMsg.motor_speed;
      fs.remote_active = isRemoteActive();
      fs.pitch_deg = imu.pitch;
      fs.pitch_vel_dps = imu.pitch_velocity;
      fs.yaw_deg = imu.yaw;
      fs.yaw_vel_dps = imu.yaw_velocity;
      fs.roll_deg = imu.roll;
      fs.dive_conf = det.dive_confidence;
      fs.aww_conf = det.aww_confidence;
      fs.af_conf = det.active_flight_confidence;
      flightLogger.update(det.active_flight_confidence, fs);
    }
    
    applySafetyLimits();
    sendMotorCommand();
    printStatus();
  }
  
  delay(1);
}