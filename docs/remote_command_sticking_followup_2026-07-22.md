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
   kite latched stayed non-zero, and issue (1) held it forever. The kite's ISW
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

### Other suggestions
_(to be appended)_

- …
