// Remote Control ESP32 - Wireless Joystick Controller for Rfx_3
// This program runs on a second ESP32 with a joystick attached
// It sends control commands to the main Rfx_3 system via ESP-NOW (very low latency)
// Compatible with moteus motor controller velocity commands

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "joystick_handler.h"
#include "fleet_protocol.h"

// =================== JOYSTICK CONFIGURATION ===================
// All 4 joysticks — pins and calibrated deadbands.
// Joystick index 0-3 maps to kite_id 1-4.
struct JoystickConfig {
  uint8_t x_pin, y_pin, sw_pin;
  int dz_y_lower, dz_y_upper, dz_x_lower, dz_x_upper;
};

static const JoystickConfig JS_CONFIG[MAX_KITES] = {
  {  6,  7, 39,  2068, 2156, 2051, 2144 },  // Joystick 1
  {  4,  5, 40,  2037, 2136, 2066, 2157 },  // Joystick 2
  { 12, 10, 41,  2087, 2176, 2044, 2119 },  // Joystick 3
  { 13, 11, 42,  2087, 2166, 2036, 2115 },  // Joystick 4
};

// =================== KNOWN KITES ===================
// Fixed MAC → joystick slot mapping. Slot 0 = first entry, slot 1 = second, etc.
// Add or reorder entries here to change which kite each joystick controls.
struct KnownKite {
  uint8_t mac[6];
  const char* name;
  uint8_t r, g, b;   // full-brightness LED color for this kite
};

static const KnownKite KNOWN_KITES[] = {
  { {0x3C, 0x84, 0x27, 0xFC, 0xC8, 0x9C}, "purple", 160,   0, 200 },  // Joystick 1
  { {0xE4, 0xB0, 0x63, 0xAE, 0x7B, 0x28}, "blue",     0,   0, 255 },  // Joystick 2
  { {0xE4, 0xB0, 0x63, 0xAE, 0xBA, 0xF8}, "red",    255,   0,   0 },  // Joystick 3
};
static const int NUM_KNOWN_KITES = sizeof(KNOWN_KITES) / sizeof(KNOWN_KITES[0]);

// =================== CONFIG ===================
const char* WIFI_SSID = "iPhone 123";
const char* WIFI_PASS = "sonoma1991";

// =================== PER-JOYSTICK STATE ===================
struct JoystickSlot {
  JoystickController* controller;
  uint8_t targetMAC[6];       // kite this joystick sends to
  bool assigned;               // true if a kite was matched for this joystick
  unsigned long lastCommandSent;
  unsigned long lastActiveTime;
  unsigned long lastButtonChangeTime;
  bool prevButton;
};

JoystickSlot slots[MAX_KITES];   // index 0 = joystick 1 → KNOWN_KITES[0], etc.

// =================== FLEET STATE ====================
bool fleetDiscovered = false;
KiteSlot knownKites[MAX_KITES];
uint8_t knownFleetSize = 0;

// Roster response received in ISR
volatile bool rosterReceived = false;
FleetRosterMsg pendingRoster;

// =================== ESP-NOW DATA STRUCTURE ===================
FleetControlMsg outgoingMsg;         // 5-byte fleet control message

const unsigned long COMMAND_INTERVAL = 20;  // 50 Hz per joystick

// =================== ESP-NOW FUNCTIONS ===================
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Callback when data is sent (debug only)
}

// Receive callback — handles fleet roster responses from host
void onDataReceived(const esp_now_recv_info *recv_info, const uint8_t *data, int data_len) {
  if (data_len == 0) return;
  uint8_t msg_type = data[0];
  if (msg_type == MSG_FLEET_ROSTER && data_len >= sizeof(FleetRosterMsg) && !rosterReceived) {
    memcpy(&pendingRoster, data, sizeof(FleetRosterMsg));
    rosterReceived = true;
  }
}

void sendMotorCommand(const uint8_t* mac, int speed, bool button_pressed) {
  outgoingMsg.msg_type = MSG_CONTROL;
  outgoingMsg.motor_speed = speed;
  outgoingMsg.command = 0;  // Velocity command
  outgoingMsg.button = button_pressed ? 1 : 0;
  esp_now_send(mac, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
}

void sendStopCommand(const uint8_t* mac) {
  outgoingMsg.msg_type = MSG_CONTROL;
  outgoingMsg.motor_speed = 0;
  outgoingMsg.command = 2;  // Stop
  outgoingMsg.button = 0;
  esp_now_send(mac, (uint8_t *)&outgoingMsg, sizeof(outgoingMsg));
}

// =================== FLEET DISCOVERY ===================
// Sends a discover broadcast and assigns any newly-found kites to their slots.
// Returns the number of newly assigned slots.
int doFleetDiscovery(int attempts, unsigned long timeout_ms) {
  rosterReceived = false;

  for (int attempt = 0; attempt < attempts; attempt++) {
    FleetDiscoverMsg disc = {};
    disc.msg_type = MSG_REMOTE_DISCOVER;
    disc.joystick_id = 0;
    uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(broadcast, (uint8_t*)&disc, sizeof(disc));
    Serial.printf("[FLEET] Discover attempt %d/%d...\n", attempt + 1, attempts);

    unsigned long waitStart = millis();
    while (millis() - waitStart < timeout_ms) {
      if (rosterReceived) break;
      delay(50);
    }
    if (rosterReceived) break;
  }

  if (!rosterReceived) {
    Serial.println("[FLEET] No host responded");
    return 0;
  }

  rosterReceived = false;
  knownFleetSize = pendingRoster.fleet_size;
  if (knownFleetSize > MAX_KITES) knownFleetSize = MAX_KITES;
  memcpy(knownKites, pendingRoster.kites, sizeof(KiteSlot) * knownFleetSize);
  fleetDiscovered = true;

  Serial.printf("[FLEET] Roster: %d kite(s) online", knownFleetSize);
  for (uint8_t k = 0; k < knownFleetSize; k++) {
    Serial.printf("  K%d(%02X:%02X:%02X:%02X:%02X:%02X)",
                  knownKites[k].kite_id,
                  knownKites[k].mac[0], knownKites[k].mac[1],
                  knownKites[k].mac[2], knownKites[k].mac[3],
                  knownKites[k].mac[4], knownKites[k].mac[5]);
  }
  Serial.println();

  // Assign each unassigned slot to its known kite by MAC address
  int newlyAssigned = 0;
  for (int j = 0; j < NUM_KNOWN_KITES; j++) {
    if (slots[j].assigned) continue;
    for (uint8_t k = 0; k < knownFleetSize; k++) {
      if (memcmp(knownKites[k].mac, KNOWN_KITES[j].mac, 6) == 0) {
        memcpy(slots[j].targetMAC, knownKites[k].mac, 6);
        slots[j].assigned = true;
        newlyAssigned++;
        if (!esp_now_is_peer_exist(slots[j].targetMAC)) {
          esp_now_peer_info_t p = {};
          memcpy(p.peer_addr, slots[j].targetMAC, 6);
          p.channel = 0;
          p.encrypt = false;
          esp_now_add_peer(&p);
        }
        Serial.printf("[FLEET] Joystick %d \u2192 %s  %02X:%02X:%02X:%02X:%02X:%02X\n",
                      j + 1, KNOWN_KITES[j].name,
                      slots[j].targetMAC[0], slots[j].targetMAC[1],
                      slots[j].targetMAC[2], slots[j].targetMAC[3],
                      slots[j].targetMAC[4], slots[j].targetMAC[5]);
        break;
      }
    }
  }
  return newlyAssigned;
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n=== Remote Control Startup (Multi-Kite) ===");
  rgbLedWrite(2, 0, 0, 0);
  
  // Initialize all 4 joysticks
  for (int i = 0; i < MAX_KITES; i++) {
    const JoystickConfig& cfg = JS_CONFIG[i];
    analogSetPinAttenuation(cfg.x_pin, ADC_11db);
    analogSetPinAttenuation(cfg.y_pin, ADC_11db);

    slots[i].controller = new JoystickController(cfg.x_pin, cfg.y_pin, cfg.sw_pin);
    slots[i].controller->begin();
    slots[i].controller->setDeadzone(cfg.dz_y_lower, cfg.dz_y_upper, cfg.dz_x_lower, cfg.dz_x_upper);
    slots[i].controller->setInvert(true, true);
    slots[i].controller->setSpeedLimits(-1000, 1000);
    slots[i].controller->setFilterAlpha(0.7);

    memset(slots[i].targetMAC, 0, 6);
    slots[i].assigned = false;
    slots[i].lastCommandSent = 0;
    slots[i].lastActiveTime = 0;
    slots[i].lastButtonChangeTime = 0;
    slots[i].prevButton = false;

    Serial.printf("  Joystick %d: X=%d Y=%d SW=%d\n", i + 1, cfg.x_pin, cfg.y_pin, cfg.sw_pin);
  }
  
  // Initialize WiFi
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
    Serial.println("[ERROR] ESP-NOW init failed");
    return;
  }
  Serial.println("[OK] ESP-NOW initialized!");
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);

  // Register broadcast peer for fleet discovery
  {
    esp_now_peer_info_t bcast = {};
    memset(bcast.peer_addr, 0xFF, 6);
    bcast.channel = 0;
    bcast.encrypt = false;
    esp_now_add_peer(&bcast);
  }

  // ===== Fleet Discovery =====
  Serial.println("\n[FLEET] Starting fleet discovery...");
  rgbLedWrite(2, 0, 0, 50);  // Blue = discovering
  int assignedCount = doFleetDiscovery(DISCOVER_RETRIES + 1, DISCOVER_TIMEOUT_MS);
  if (assignedCount > 0) {
    rgbLedWrite(2, 0, 50, 0);   // Green = at least 1 kite found
  } else {
    rgbLedWrite(2, 50, 20, 0);  // Orange = no kites found
  }
  Serial.println("[OK] Remote control ready!");
}

// =================== MAIN LOOP ===================
void loop() {
  unsigned long now = millis();

  // Update and send for each joystick (only slots with a connected kite)
  for (int i = 0; i < MAX_KITES; i++) {
    JoystickSlot& s = slots[i];
    if (!s.assigned) continue;
    JoystickController& js = *s.controller;
    js.update();

    bool current_switch = js.getSwitch();
    bool button_changed = (current_switch != s.prevButton);
    s.prevButton = current_switch;
    if (button_changed) s.lastButtonChangeTime = now;

    bool shouldSend = false;
    int cmd = 0;

    if (js.isInUse()) {
      s.lastActiveTime = now;
      cmd = js.getMotorCommand();
      shouldSend = true;
    } else if (now - s.lastActiveTime < 100) {
      // Just released — keep sending zero briefly
      cmd = 0;
      shouldSend = true;
    } else if (now - s.lastButtonChangeTime < 100) {
      // Button changed recently — ensure state is transmitted
      cmd = 0;
      shouldSend = true;
    }

    if (shouldSend && (now - s.lastCommandSent >= COMMAND_INTERVAL)) {
      s.lastCommandSent = now;
      sendMotorCommand(s.targetMAC, cmd, current_switch);
      Serial.printf("J%d→%-6s TX:%+5d btn:%d\n", i + 1, i < NUM_KNOWN_KITES ? KNOWN_KITES[i].name : "?", cmd, current_switch);
    }
  }

  // LED: reflect dominant joystick command — brightness = speed, colour = direction
  int dominantCmd = 0;
  for (int i = 0; i < MAX_KITES; i++) {
    if (!slots[i].assigned || !slots[i].controller) continue;
    int c = slots[i].controller->getMotorCommand();
    if (abs(c) > abs(dominantCmd)) dominantCmd = c;
  }
  if (dominantCmd > 0) {
    int brightness = map(dominantCmd, 0, 1000, 10, 255);
    rgbLedWrite(2, brightness, brightness, brightness);  // White = extending
  } else if (dominantCmd < 0) {
    int brightness = map(-dominantCmd, 0, 1000, 10, 255);
    rgbLedWrite(2, brightness, 0, 0);                    // Red = retracting
  } else {
    // Idle — heartbeat: N dim blue blips every ~2s (N = assigned kites)
    static int  hbBlip = 0;         // 0 = initial pause, 1..N = current blip
    static bool hbOn   = false;
    static unsigned long hbTime = 0;
    int connectedKites = 0;
    for (int i = 0; i < MAX_KITES; i++) if (slots[i].assigned) connectedKites++;

    if (connectedKites == 0) {
      rgbLedWrite(2, 0, 0, 0);                           // No fleet — stay off
      hbBlip = 0; hbTime = now;
    } else if (hbBlip == 0) {
      rgbLedWrite(2, 0, 0, 0);                           // Pre-sequence pause
      if (now - hbTime >= 2000) { hbBlip = 1; hbOn = true; hbTime = now; }
    } else if (hbOn) {
      // Color = dim version of the hbBlip-th connected kite's color
      uint8_t hr = 8, hg = 8, hb = 8;  // default: dim white
      int cnt = 0;
      for (int i = 0; i < NUM_KNOWN_KITES; i++) {
        if (slots[i].assigned && ++cnt == hbBlip) {
          hr = KNOWN_KITES[i].r / 20;
          hg = KNOWN_KITES[i].g / 20;
          hb = KNOWN_KITES[i].b / 20;
          if (hr == 0 && hg == 0 && hb == 0) { hr = hg = hb = 8; }  // fallback white
          break;
        }
      }
      rgbLedWrite(2, hr, hg, hb);
      if (now - hbTime >= 80) { hbOn = false; hbTime = now; }
    } else {
      rgbLedWrite(2, 0, 0, 0);
      if (hbBlip < connectedKites) {
        if (now - hbTime >= 150) { hbBlip++; hbOn = true; hbTime = now; }  // Next blip
      } else {
        if (now - hbTime >= 2000) { hbBlip = 1; hbOn = true; hbTime = now; }  // New cycle
      }
    }
  }

  // Periodic fleet re-discovery — every 10s if any slot unassigned, else every 30s
  static unsigned long lastDiscoveryTime = 0;
  {
    bool anyUnassigned = false;
    for (int i = 0; i < MAX_KITES; i++) if (!slots[i].assigned) { anyUnassigned = true; break; }
    unsigned long interval = anyUnassigned ? 10000UL : 30000UL;
    if (now - lastDiscoveryTime >= interval) {
      lastDiscoveryTime = now;
      int found = doFleetDiscovery(1, 800);  // Quick single attempt
      // Print connection status
      int conn = 0;
      for (int i = 0; i < NUM_KNOWN_KITES; i++) if (slots[i].assigned) conn++;
      Serial.printf("[STATUS] %d/%d kite(s) connected", conn, NUM_KNOWN_KITES);
      for (int i = 0; i < NUM_KNOWN_KITES; i++) {
        if (slots[i].assigned) {
          Serial.printf("  J%d→%s", i + 1, KNOWN_KITES[i].name);
        } else {
          Serial.printf("  J%d→(none)", i + 1);
        }
      }
      Serial.println();
      (void)found;
    }
  }

  // Periodically check WiFi (every 30s)
  static unsigned long lastWiFiCheckTime = 0;
  if (now - lastWiFiCheckTime > 30000) {
    lastWiFiCheckTime = now;
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] Connected | Channel: %d\n", WiFi.channel());
    } else {
      Serial.println("[WiFi] Disconnected");
    }
  }

  delay(5);
}
