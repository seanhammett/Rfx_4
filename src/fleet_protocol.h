// Fleet Protocol — shared message types for multi-kite ESP-NOW coordination
// Used by both main.cpp (kite) and remote_control.cpp (remote)

#ifndef FLEET_PROTOCOL_H
#define FLEET_PROTOCOL_H

#include <stdint.h>

// ===== Constants =====
#define MAX_KITES               4
#define FLEET_ANNOUNCE_INTERVAL 2000   // ms between host heartbeat broadcasts
#define HOST_TIMEOUT_MS         6000   // ms without announce before re-election
#define DISCOVER_TIMEOUT_MS     3000   // ms remote waits for roster reply
#define DISCOVER_RETRIES        2      // number of retry attempts for remote discovery

// Kite display colours (indexed by kite_id 1-4)
static const char* KITE_COLORS[] = { "#ffffff", "#ff4444", "#ffa807", "#44bb44", "#4488ff" };

// ===== Message Types =====
enum FleetMsgType : uint8_t {
  MSG_CONTROL        = 0,  // Joystick command (from remote to kite)
  MSG_FLEET_ANNOUNCE = 1,  // Host heartbeat broadcast (host → all kites)
  MSG_FLEET_REGISTER = 2,  // New kite requests to join (kite → host)
  MSG_FLEET_ACK      = 3,  // Host assigns kite_id (host → kite)
  MSG_REMOTE_DISCOVER= 4,  // Remote requests fleet roster (remote → broadcast)
  MSG_FLEET_ROSTER   = 5,  // Host responds with roster (host → remote)
  MSG_KITE_IMU       = 6,  // Kite-mounted IMU data (CodeCell → kite controller)
};

// ===== Kite Identity Slot =====
typedef struct __attribute__((packed)) {
  uint8_t mac[6];
  uint8_t kite_id;       // 1-based (0 = unassigned)
  uint32_t ip_addr;      // IPv4 address (network byte order)
} KiteSlot;

// ===== Control Message (remote → kite) =====
// Prefixed with msg_type so kites can distinguish fleet vs control packets.
// For backward compat: old 4-byte ControlMessage (no prefix) is detected by length.
typedef struct __attribute__((packed)) {
  uint8_t msg_type;      // MSG_CONTROL
  int16_t motor_speed;   // -1000 to +1000
  uint8_t command;       // 0=speed, 1=unused, 2=stop
  uint8_t button;        // 0=released, 1=pressed
} FleetControlMsg;       // 5 bytes

// ===== Fleet Announce (host → broadcast) =====
typedef struct __attribute__((packed)) {
  uint8_t msg_type;      // MSG_FLEET_ANNOUNCE
  uint8_t host_mac[6];
  uint8_t fleet_size;
  KiteSlot kites[MAX_KITES];  // roster (only fleet_size entries valid)
} FleetAnnounceMsg;

// ===== Fleet Register (new kite → host) =====
typedef struct __attribute__((packed)) {
  uint8_t msg_type;      // MSG_FLEET_REGISTER
  uint8_t requester_mac[6];
  uint32_t requester_ip;
} FleetRegisterMsg;

// ===== Fleet Ack (host → kite) =====
typedef struct __attribute__((packed)) {
  uint8_t msg_type;      // MSG_FLEET_ACK
  uint8_t assigned_id;   // kite_id assigned by host
  uint8_t fleet_size;
  KiteSlot kites[MAX_KITES];  // current roster
} FleetAckMsg;

// ===== Remote Discover (remote → broadcast) =====
typedef struct __attribute__((packed)) {
  uint8_t msg_type;      // MSG_REMOTE_DISCOVER
  uint8_t joystick_id;   // which ACTIVE_JOYSTICK this remote is (1-4)
} FleetDiscoverMsg;

// ===== Fleet Roster (host → remote) =====
typedef struct __attribute__((packed)) {
  uint8_t msg_type;      // MSG_FLEET_ROSTER
  uint8_t fleet_size;
  KiteSlot kites[MAX_KITES];
} FleetRosterMsg;

// ===== Legacy ControlMessage (backward compat — no type prefix, 4 bytes) =====
typedef struct __attribute__((packed)) {
  int16_t motor_speed;
  uint8_t command;
  uint8_t button;
} LegacyControlMsg;

// ===== Kite IMU Data (CodeCell on kite → kite controller) =====
typedef struct __attribute__((packed)) {
  uint8_t msg_type;      // MSG_KITE_IMU
  float pitch;           // degrees (game rotation, no magnetometer)
  float roll;            // degrees
  uint8_t battery_pct;   // 0-100 battery level, 101=charging, 102=USB
  uint8_t sequence;      // rolling packet counter for drop detection
} KiteImuMsg;            // 11 bytes

#endif // FLEET_PROTOCOL_H
