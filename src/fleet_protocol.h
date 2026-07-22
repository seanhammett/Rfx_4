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
#define REMOTE_EVENT_REPEAT_MS  120    // remote resends a one-shot control event this long (drop tolerance)

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
};

// ===== Kite Identity Slot =====
typedef struct __attribute__((packed)) {
  uint8_t mac[6];
  uint8_t kite_id;       // 1-based (0 = unassigned)
  uint32_t ip_addr;      // IPv4 address (network byte order)
} KiteSlot;

// ===== Control Events (remote → kite) =====
// The remote parses raw switch input into semantic, one-shot actions and sends
// the resolved event — the kite no longer does click detection itself. Events are
// edge-triggered, so the remote repeats each one for a short window (see
// REMOTE_EVENT_REPEAT_MS) and stamps a rolling event_seq; the kite acts once per
// new seq and ignores repeats. (Level-triggered "held" states will use a separate
// flags field when group-fly lands — see docs/joint_flight_features_2026-07-22.md.)
enum FleetControlEvent : uint8_t {
  EVENT_NONE               = 0,  // no pending action
  EVENT_TOGGLE_TARGET_SEEK = 1,  // was: single-click
  EVENT_TOGGLE_RESPOOL     = 2,  // was: triple-click
};

// ===== Control Message (remote → kite) =====
// Prefixed with msg_type so kites can distinguish fleet vs control packets.
// For backward compat: old 4-byte ControlMessage (no prefix) is detected by length.
typedef struct __attribute__((packed)) {
  uint8_t msg_type;      // MSG_CONTROL
  int16_t motor_speed;   // -1000 to +1000
  uint8_t command;       // 0=speed, 1=unused, 2=stop
  uint8_t event;         // FleetControlEvent — one-shot semantic action (0 = none)
  uint8_t event_seq;     // rolling id; kite de-dups so a repeated event fires once
} FleetControlMsg;       // 6 bytes

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

#endif // FLEET_PROTOCOL_H
