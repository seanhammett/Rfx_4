# Remote command "sticking" — fix notes & follow-up work

**Date:** 2026-07-22
**Branch:** `fix/remote-command-sticking`

## Background — the bug

With three kites connected to one remote, a commanded fast retract would sometimes
**keep running after the joystick was released to neutral**. Sending any fresh
command released the stick; simply centring the joystick did not. It was
intermittent and got worse with more kites connected.

### Root cause

Two compounding issues:

1. **Fail-dangerous hold (primary).** In the kite motion loop, when the remote
   link went quiet (no packet for `REMOTE_TIMEOUT_MS` = 300 ms) and autopilot was
   off, the code deliberately kept the last `commanded_velocity`
   (`main.cpp`, "…keep last commanded_velocity (don't zero it out!)"). Because
   `sendMotorCommand()` re-streams that velocity every 10 ms with a refreshed
   500 ms Moteus watchdog, a stale retract velocity persisted **indefinitely** and
   the motor's own watchdog never tripped.

2. **Fragile neutral signalling (trigger).** The remote only emits release-zeros
   for a short (~350 ms) window after release, then goes silent. If that burst was
   lost — far more likely with three concurrent 50 Hz streams — the last value the
   kite latched stayed non-zero, and issue (1) held it forever. The kite's ISR
   dispatcher made this worse by **dropping the newest control packet** whenever the
   previous one was unconsumed (`&& !newRemoteCommand`), so the meaningful
   release-zero could be the one discarded.

"Sending another command releases it" because any fresh packet re-latches
`remoteControlMsg`, makes `isRemoteActive()` true again, and lets
`processJoystickInput()` overwrite the velocity.

## What was implemented on this branch

### A — Kite fail-safe on timeout (`src/main.cpp`, motion loop)
Replaced the "keep last velocity" fallthrough with an explicit fail-safe: when
neither autopilot nor an active remote link is driving the motor, command
`commanded_velocity = 0.0` (and reset `target_velocity_current`). Velocity 0 — not
`SetStop()` — is used so the spool **holds line length under tension**; `SetStop()`
would drop torque and let the line pay out.

- Worst-case over-travel after a lost release-zero is now bounded to
  ~`REMOTE_TIMEOUT_MS` (300 ms) instead of running away indefinitely.
- Brief packet gaps < 300 ms during active control still do **not** cause a
  stutter, because `isRemoteActive()` stays true across them.

### C — Latest-wins ISR dispatcher (`src/main.cpp`, `onDataReceived` / `MSG_CONTROL`)
Removed the `&& !newRemoteCommand` guard so the freshest stick state always wins.
This prevents a newer release-zero from being dropped in favour of an older,
unconsumed packet. The only cost is a possible torn read of the small
`remoteControlMsg` struct (one field stale for a single 10 ms cycle) — harmless,
and button-click edge detection is unaffected at the 100 Hz loop rate.

### Verify after flashing
- [ ] Fast retract on one kite, release to neutral → motor stops within ~300 ms,
      every time, no stick.
- [ ] Repeat with all three kites connected and driven simultaneously.
- [ ] Confirm holding a retract through a brief (<300 ms) RF dropout does not
      stutter/stop mid-gesture.
- [ ] Confirm autopilot / target-seeking still drives velocity normally (the
      fail-safe `else` must not fire while `target_seeking_enabled`).
- [ ] Confirm the `/motor/velocity` web test endpoint is not relied on during
      flight — the motion loop now authoritatively zeros an idle spool.

---

## Build environment — Moteus library pinned (Mac migration)

Moving the project from the original Windows PC to a Mac triggered a fresh
dependency resolve, which surfaced a build break unrelated to the sticking fix:

```
src/main.cpp:42:1: error: 'Moteus' does not name a type
```

**Cause:** `platformio.ini` requested `mjbots/Moteus@^1.0.2` (i.e. `>=1.0.2 <2.0.0`).
The Windows build predated the 1.1.x releases, so it resolved to **1.0.2**. The
fresh Mac resolve pulled **1.1.1**, which had refactored the API:

| Version | Date       | API                                                        |
|---------|------------|------------------------------------------------------------|
| 1.0.2   | 2024-05-09 | Concrete `Moteus` class defined in `<Moteus.h>` (used here) |
| 1.1.0   | 2026-03-19 | Refactored to template `MoteusController<CanBus>`; `using Moteus = ...` alias moved to `<MoteusAcan2517fd.h>` |
| 1.1.1   | 2026-04-02 | (what the Mac auto-resolved — breaks `#include <Moteus.h>`) |
| 1.1.2   | 2026-07-15 | latest                                                     |

**Fix applied:** pinned `mjbots/Moteus@1.0.2` (exact) in `platformio.ini` — the
latest version compatible with the unmodified, field-tested firmware. All three
ESP32 envs (`rfx-4-main`, `remote-control`, `remote-calibration`) build clean.

**Follow-up option (not done):** migrating to Moteus 1.1.x looks low-effort —
change `#include <Moteus.h>` → `#include <MoteusAcan2517fd.h>` (which supplies
`using Moteus = MoteusController<ACAN2517FD>`), then re-verify the
`Options` / `PositionMode` / `kFloat` / `SetStop` / `SetPosition` / `last_result`
API is unchanged. Not worth doing until there's a reason to want a newer library.

**Broader hygiene:** the remaining `^` dependency ranges (e.g.
`pierremolinaro/ACAN2517FD@^2.1.16`, the ESPAsyncWebServer / ArduinoJson / Adafruit
deps) can drift the same way on the next clean checkout. Consider pinning them to
the exact versions currently building, for reproducible flight firmware.

## Follow-up work (not yet implemented)

### B — Remote neutral heartbeat (robustness + lower latency)
Today an idle-but-assigned joystick sends nothing. Add a low-rate neutral
heartbeat in the remote loop (`src/remote_control.cpp`) so the kite is continuously
fed zeros whenever the remote is powered:

```cpp
// after the existing shouldSend logic, per joystick slot:
if (!shouldSend && s.assigned && (now - s.lastCommandSent >= NEUTRAL_HEARTBEAT_MS)) {
  cmd = 0; shouldSend = true;   // idle neutral heartbeat keeps the kite fed with zeros
}
```

- **Effect:** normal-release over-travel drops from ~300 ms to ~one command
  interval; the kite only enters the Option-A timeout path when the remote is
  actually powered off.
- **Cost:** extra baseline airtime (heartbeat rate × number of kites). Since RF
  contention is what causes the loss in the first place, keep the rate modest and
  re-measure loss with all three kites before/after.
- **Suggested starting point:** `NEUTRAL_HEARTBEAT_MS = 100` (10 Hz), well below the
  active 50 Hz stream. Tune up/down against measured packet loss.

### Tuning parameters to revisit once A + C (+ B) are in
- `REMOTE_TIMEOUT_MS` (currently 300 ms, `src/main.cpp`) — the fail-safe stop
  latency. With heartbeat (B) in place this could likely be shortened for a
  crisper stop without risking mid-gesture stutter.
- Remote release window — joystick `idle_timeout` (250 ms,
  `src/joystick_handler.cpp`) + the 100 ms grace in `src/remote_control.cpp`.
  Largely superseded by the heartbeat if B is adopted.
- `filter_alpha` (0.7, set in `remote_control.cpp setup`) — trades stick smoothing
  against how long a fast release takes to decay into the deadzone before zeros
  begin.
- `COMMAND_INTERVAL` (20 ms / 50 Hz per joystick, `src/remote_control.cpp`) vs.
  total RF airtime across all kites — the core contention budget.
- Consider measuring/logging actual ESP-NOW loss per kite to drive the above
  numbers rather than guessing.

---

## ESP-NOW architecture — reducing traffic & improving resilience

> **Status: analysis only — nothing here is implemented.** Deferred until hardware
> is set up to test. Captured so the design thinking isn't lost.

### Was the sticking actually caused by "too much traffic"? No — not bandwidth.

The raw volume is tiny. The remote sends a 5-byte control message per assigned kite
at 50 Hz — worst case 3 kites × 50 Hz = **150 packets/sec of ~5-byte payloads**.
ESP-NOW handles thousands of small frames/sec, so the link was nowhere near
saturated. Individual packets were being **lost**, and three things made that likely:

1. **Fire-and-forget delivery.** Unicast ESP-NOW does a MAC-level ACK + a few
   retries, but if it ultimately fails the app never knows — `onDataSent` is empty
   (`src/remote_control.cpp`). Nothing re-sends a lost packet.
2. **Shared radio + channel with WiFi.** Each kite runs WiFi STA (hotspot) *plus*
   the web dashboard, SPIFFS page serving, and Supabase flight-log uploads — all on
   the same radio/channel as ESP-NOW. That contention (not the control stream) is
   the dominant loss source. The `watchdog_timeout = 0.5` comment in
   `sendMotorCommand()` ("survives SPIFFS reads during page refresh") is direct
   evidence this was already biting.
3. **The receive-side drop guard** (`!newRemoteCommand`) — already removed in
   Option C.

Three kites amplified all of this (more concurrent streams *and* three dashboards'
worth of WiFi traffic), which is why it surfaced with the fleet, not with one kite.

**So the real levers are link reliability under contention and retransmit
awareness, not throughput.** Note that Option A (committed) already makes the system
*safe* regardless of loss; everything below only reduces how often the glitch
happens and hardens the link. **None of these approaches cost command rate or
resolution** — that constraint was explicit.

### Key insight the design should exploit

Control messages are **absolute and idempotent**: `motor_speed` is the full current
stick position, not a delta. Therefore **you never need every packet to arrive, only
for a recent one to arrive.** The entire bug was that the *last* packet before going
idle could be lost with nothing after it. Two design changes lean on this property.

### Approach 1 — Never go fully silent (heartbeat current state)

Same mechanism as Approach B above, but the framing is the point: because commands
are idempotent, a steady low-rate keepalive of the *current* stick state (including
neutral) means arbitrary loss self-heals within one heartbeat interval. Keep full
50 Hz while the stick moves (zero latency cost); never stop emitting the current
value while assigned (~10 Hz idle). **Top resilience win. No speed/resolution cost.**

### Approach 2 — One broadcast "fleet frame" instead of N unicast streams

Instead of the remote sending N separate unicast packets per cycle, send **one
broadcast frame carrying an array of per-kite commands**; each kite reads its own
slot. This is the change that actually *reduces traffic*:

- **~N× fewer transmissions** (1 frame/cycle instead of N) and it removes the
  per-unicast MAC ACK/retry airtime → less airtime → less contention → less loss
  for everyone.
- **Idle kites get their heartbeat for free** — as long as any joystick is active,
  the frame already carries every kite's current (neutral) value.
- **Same rate, same int16 resolution per kite** — nothing sacrificed.
- **Trade-off:** broadcast has no MAC-level ACK/retry, so per-packet reliability is
  lower — which the idempotent-stream + heartbeat design makes irrelevant.

Sketch frame layout (well under the 250-byte ESP-NOW limit):

```cpp
typedef struct __attribute__((packed)) {
  uint8_t  msg_type;      // e.g. MSG_FLEET_CONTROL
  uint8_t  seq;           // rolling counter (see Approach 7)
  uint8_t  count;         // number of valid slots
  struct {
    int16_t motor_speed;  // -1000..1000, absolute (full resolution preserved)
    uint8_t command;      // 0=speed, 2=stop
    uint8_t button;       // 0/1
  } slot[MAX_KITES];      // slot[i] → kite_id i+1
} FleetControlFrame;      // broadcast, ~3 + 4*MAX_KITES bytes
```

Remote sends it to the broadcast MAC each cycle; each kite indexes `slot[myKiteId-1]`.
Reception needs no per-sender peer entry — the recv callback fires for broadcast
frames on the shared channel. If you'd rather keep unicast, use Approach 3 instead.

### Approach 3 — Send-status callback + targeted fast-retransmit (unicast alternative)

If staying unicast, wire up `onDataSent`: when a send reports failure, immediately
re-send that kite's packet once or twice. Adds reliability *only when a packet
actually drops*, with no steady-traffic cost. The reactive complement to Approach 1.

### Approach 4 — Throttle the kite's WiFi work during active manual control

The dashboard telemetry, SPIFFS serving, and Supabase uploads compete with the
control link on the same radio. Reducing their push rate (or deferring uploads)
while `isRemoteActive()` is true directly cuts the contention that causes the drops.
**Biggest environmental win; attacks the actual loss source.** No control-path cost.

### Approach 5 — Lock the ESP-NOW channel deterministically

Today the channel is whatever the iPhone hotspot assigns; if WiFi reconnects on a
different channel, ESP-NOW silently breaks. Pinning it (`esp_wifi_set_channel`) and
verifying all nodes match removes a class of "suddenly unresponsive" failures.
Constraint: with WiFi STA active, ESP-NOW must use the STA's current channel — so
this pairs with a known/fixed AP channel, or a decision about WiFi-vs-ESP-NOW
priority during flight.

### Approach 6 — Bump TX power

`esp_wifi_set_max_tx_power(...)` — a cheap link-margin improvement if range/signal
is ever marginal. Low effort, no downside for a short-range remote.

### Approach 7 — Add a sequence counter for loss measurement

A 1-byte rolling `seq` per control frame doesn't change traffic, but lets each kite
*measure* actual loss (gaps in seq). Turns heartbeat-rate / channel decisions into
data-driven tuning instead of guesswork. Complements the "log ESP-NOW loss" note in
the tuning section.

### Recommended combination (when ready to test)

- Highest impact / least change: **Approach 1 (heartbeat) + Approach 2 (broadcast
  fleet frame)** — fewer packets, self-healing loss, zero latency/resolution cost,
  and #2 folds the heartbeat in almost for free.
- Pair with **Approach 4 (throttle kite WiFi during active control)** to attack the
  root contention.
- **Approach 3** is the fallback if the unicast model is preferred over broadcast.
- Add **Approach 7** first if you want loss numbers to guide the rest.


