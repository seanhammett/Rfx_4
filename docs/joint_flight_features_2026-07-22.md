# Joint-flight features — design notes

**Date:** 2026-07-22
**Branch:** `fix/remote-command-sticking`
**Status:** Design / architecture analysis, **plus one foundational refactor now implemented** —
input parsing has been moved from the kite to the remote (see
[Move input parsing to the remote](#move-input-parsing-to-the-remote-implemented)). The two
joint-flight features themselves (equalize, group-move) remain design-only and unflashed/untested
on hardware. The "Changes made" table at the bottom records what has actually changed.

---

## Goal

Two "joint-flight" (whole-fleet) features, both flagged on the built-in LED:

1. **Equalize height** — all kites move to the *average* line length. Candidate trigger: press
   joystick buttons **1 and 3 together**.
2. **Move all kites together** — a manual group-fly mode. Candidate trigger: **push-and-hold**,
   then steer with a joystick; all kites follow the same command.

---

## How the system is wired today (grounding)

Understanding this is what makes the two features easy or hard, so it's worth stating precisely.

### Topology
- The **remote** ([src/remote_control.cpp](../src/remote_control.cpp)) has `NUM_JOYSTICKS` physical
  joysticks (currently **3**). Each joystick `i` has its **own** X, Y and push-**switch**, and is
  bound 1:1 to `kite_id i+1`. It **unicasts** control packets to that kite's MAC at ~50 Hz
  ([remote_control.cpp:256-289](../src/remote_control.cpp#L256)).
- The **kites** self-organise into a fleet. One kite **self-elects as HOST**
  ([main.cpp:490-533](../src/main.cpp#L490)); the host owns the roster and answers the remote's
  discovery broadcast. Kites join via `MSG_FLEET_REGISTER` / `MSG_FLEET_ANNOUNCE`.
- **Key limitation:** the roster only carries `mac`, `kite_id`, `ip_addr`
  ([fleet_protocol.h:30-34](../src/fleet_protocol.h#L30)). **No kite currently knows any other
  kite's line length**, and the **remote receives no per-kite telemetry** at all — only the roster
  of MACs. This is the one thing that has to change for "equalize height."

### Control message
`FleetControlMsg` is now **6 bytes** ([fleet_protocol.h](../src/fleet_protocol.h)):
`msg_type`, `int16 motor_speed` (−1000..+1000), `command` (**0**=speed, 1=unused, **2**=stop),
and — after the parsing refactor — `event` + `event_seq` (which replaced the old raw `button`
bit; see below). The `FleetMsgType` enum currently runs 0–5
([fleet_protocol.h:20-27](../src/fleet_protocol.h#L20)) — **room to add new types** (6, 7, …),
and `command == 1` is a spare slot.

### Buttons
"Button 1" and "button 3" are the **switches on joystick 1 and joystick 3** — physically separate
inputs, each bound to its **own** kite. The two click actions are:
- **1 click** → toggle target-seeking on that kite.
- **3 clicks** → toggle respool mode on that kite.

> **Update (implemented 2026-07-22):** click parsing now happens **on the remote**, which sends the
> resolved *semantic event* to the kite instead of a raw button bit. See
> **[Move input parsing to the remote](#move-input-parsing-to-the-remote-implemented)** below — this
> is the foundation the two features build on.

**Button chords are still not a concept yet** — the two buttons go to two different kites, and each
press already means something on its own kite. That's the crux of the reliability question (below),
and the remote-side parsing refactor is what makes solving it clean.

### LEDs
- **Remote built-in LED** = the on-board NeoPixel on **GPIO 2**, driven by `neopixelWrite(2, …)`.
  It already runs a small state machine ([remote_control.cpp:291-329](../src/remote_control.cpp#L291)):
  white = extending, red = retracting, and a dim-blue idle "heartbeat" that blips once per
  connected kite. **This is the pilot-facing LED** and the natural place to flag a joint-flight
  mode.
- **Kite built-in LED** = `LED_BUILTIN`, aliased to the **red channel** of the Nano ESP32 RGB LED
  ([main.cpp:27-28](../src/main.cpp#L27)). Only used for the 5-blink boot indicator
  ([main.cpp:367-373](../src/main.cpp#L367)) — otherwise free. Single colour, on/off only.

When the user says "the built-in LED," the **remote NeoPixel is almost certainly what's meant**
(it's what the pilot holds and watches). The kite red LED is available as an optional per-kite
confirmation.

---

## Reliability of "press buttons 1 and 3 together"

**Short answer: yes, it can be made reliable — but only if we detect the chord on the *remote* and
suppress the per-kite single/triple-click actions while the chord is active.** Detecting two
switches at once is trivial; the trap is the meaning those two presses *already* have.

### Why the naive version misfires
Both switches are readable in the same loop pass on the remote
(`slots[0].controller->getSwitch() && slots[2].controller->getSwitch()`), so detecting "both down"
is easy. The trap is that, post-refactor, each joystick's click resolver already turns those presses
into a **1-click → `EVENT_TOGGLE_TARGET_SEEK`** for its own kite. So a chord meant to "equalize"
would *also* flip target-seeking on kites 1 and 3. Timing skew makes it worse: you never press two
buttons on the same millisecond, so each joystick resolves its own click independently.

### What makes it reliable
1. **Detect the chord on the remote, not the kite.** Add chord detection in the remote loop.
2. **Debounce the chord:** require both switches held for a short overlap (e.g. **≥ 60–80 ms**
   both-down) before it counts — rejects a stray one-frame overlap.
3. **Suppress the individual click events during a chord.** While a chord is forming/active, don't
   emit the per-joystick single/triple-click events (hold them at `EVENT_NONE`) so the kites never
   act on the presses. The chord fires a *dedicated* fleet command instead. (The parsing refactor
   above already centralises click→event mapping on the remote, so this suppression is a local check.)
4. **Require both released to re-arm** — prevents repeat-fire while held.
5. **Pick a distinctive pair.** With 3 joysticks, **1 and 3 are the outer two** — ergonomically
   separated, unlikely to be hit together by accident. Good choice. (If `NUM_JOYSTICKS` ever
   changes, define the chord as "outermost pair" rather than hard-coding 1 & 3.)

### Alternatives worth considering (both more robust than a two-button chord)
- **Long-press a single button** (hold one joystick's switch ≥ ~1 s). One input, no skew, no chord
  suppression gymnastics — but it collides with the existing click grammar on that kite, so it'd
  need the same "suppress local click while long-pressing" handling. Cleaner overall than a chord.
- **A dedicated function button** on the remote (if a spare GPIO/button exists) — zero collision,
  most reliable, but needs hardware.

**Recommendation:** the 1+3 chord is fine *if* we do chord-detection + click-suppression on the
remote. If you'd prefer the simplest-to-get-right gesture, a **long-press** is marginally more
robust. Either way the detection and the new fleet command live on the remote.

---

## Move input parsing to the remote (implemented)

**Implemented 2026-07-22.** Both firmwares build clean. Not yet flashed / bench-tested.

The kite used to receive a **raw button level** and reconstruct clicks itself. Parsing now lives on
the **remote**, which sends the kite a **resolved semantic event**. This is the right home for all
the joint-flight gestures (chord, long-press, hold) and it's more robust — the remote parses the
physically-wired switch, so no ESP-NOW subsampling or dropped edge can corrupt a click count.

### What changed
- **Wire format** ([fleet_protocol.h](../src/fleet_protocol.h)): the `button` byte in
  `FleetControlMsg` was replaced by **`event`** (`FleetControlEvent`: `EVENT_NONE`,
  `EVENT_TOGGLE_TARGET_SEEK`, `EVENT_TOGGLE_RESPOOL`) + **`event_seq`** (rolling de-dup id).
  Message grew 5 → 6 bytes.
- **Remote** ([remote_control.cpp](../src/remote_control.cpp)): reads the click count from the
  existing detector in `joystick_handler` (`getClickCount()`, already present but previously unused)
  and maps **1-click → toggle-target-seek**, **3-click → toggle-respool**. It latches the event per
  joystick slot, bumps `event_seq`, and **repeats the event for `REMOTE_EVENT_REPEAT_MS` (120 ms)**
  so a dropped packet doesn't lose the action — forcing packets out during that window even if the
  stick is idle (clicks resolve ~400 ms after the last edge, i.e. outside the normal send windows).
- **Kite** ([main.cpp](../src/main.cpp)): `processRemoteCommands()` no longer does click detection.
  It applies an event **once per new `event_seq`** (de-dup), calling `toggleTargetSeeking()` or the
  new `toggleRespoolMode()` helper. Behaviour is functionally identical to before.

### The reliability pattern (the reason it's structured this way)
Raw button **level** was *self-healing* over a lossy link — every 50 Hz packet re-asserted it, so a
drop didn't matter. A discrete **event** is edge-triggered: one dropped packet = a missed action.
So one-shot events use **repeat + sequence-id de-dup** (send ~6× over 120 ms; kite acts on the first
new seq, ignores the rest). Held/level-triggered states (like a future group-move "active" bit)
should instead stay **level-triggered** — repeating an idempotent state is drop-safe and needs no
de-dup. Pick per action: *one-shot → event+seq; sustained/held → level flag.*

### Kite stays the authority on its own modes
The remote sends *requests/events*; the kite still **owns** its latched booleans (`respool_mode`,
target-seeking). This keeps a future on-kite local button consistent (the `toggleTargetSeeking()`
"shared by local and remote" comment anticipates one; note it is currently vestigial — only the
remote toggles today) and avoids two sources of truth.

### Compatibility note
The protocol changed size, so **flash both firmwares together**. A new remote + old kite (or vice
versa) will mis-handle control: the old kite reads `event` as the old `button` byte, and the new
kite ignores the old 5-byte packet entirely. Legacy 4-byte remotes still get velocity/stop, but
**click-driven modes now require the current remote firmware** (they carry no events).

---

## Feature 1 — Equalize height (average line length)

### What it does
On trigger, every kite drives its line length toward the fleet **average**, using the target-seeking
servo that already exists ([main.cpp:896-925](../src/main.cpp#L896), velocity/accel-limited by
`line_length_target_velocity` / `line_length_target_acceleration`). Kites above average reel in,
kites below average pay out — each self-limited by the unspool tension gate (see
[unspool_tension_light_wind_2026-07-22.md](unspool_tension_light_wind_2026-07-22.md)).

### The one architectural gap: nobody knows the fleet's lengths
Today no kite knows another kite's `line_length`, and the remote knows none of them. We need the
lengths shared. Two ways:

**(A) Distributed (recommended).** Each kite periodically broadcasts its own length in a small new
message, e.g.:
```c
// new type: MSG_KITE_STATE = 6
typedef struct __attribute__((packed)) {
  uint8_t  msg_type;    // MSG_KITE_STATE
  uint8_t  kite_id;
  float    line_length; // metres
} FleetKiteStateMsg;    // 6 bytes
```
Every kite keeps a small table of peers' latest lengths. The trigger is a broadcast `MSG_EQUALIZE`
(from the remote's chord/long-press). On receipt, **each kite independently** sets
`line_length_target = average(all known lengths incl. self)` and enables target-seeking. No single
point of failure; reuses the existing servo. Cost: a low-rate extra broadcast (2–5 Hz, 6 bytes) —
tie this into the ESP-NOW traffic budget discussed in
[remote_command_sticking_followup_2026-07-22.md](remote_command_sticking_followup_2026-07-22.md).

**(B) Host-centric (lighter traffic).** Kites report length only to the host; the host computes
the average and broadcasts a single target. Fewer packets, but the host is a single point of
failure and it adds a code path that only the host runs. Prefer (A) unless traffic is tight.

Either way, the **remote just sends one trigger** — it does not need to know any lengths.

### Trigger
Chord (buttons 1+3) or long-press, per the reliability section → remote broadcasts `MSG_EQUALIZE`.
A momentary press is right here (fire-and-forget); the kites then seek autonomously.

### Safety / caveats
- **Bounded by the existing servo:** motion is already velocity- and accel-limited, and the unspool
  tension gate still applies, so a kite paying out to reach a longer average can't outrun its own
  tension. Good — equalize inherits all current protections for free.
- **Line length ≠ altitude.** `line_length` is measured from each kite's own zero
  (`motor_position_offset`, set at boot / on respool exit,
  [main.cpp:879-886](../src/main.cpp#L879)). Equalizing line length only equalizes *height* if all
  kites were zeroed from the same ground reference. Flag this to the pilot; it's a real-world
  gotcha, not a code bug.
- **Missing/stale telemetry:** compute the average only over kites heard-from within a freshness
  window (e.g. last 1 s); ignore stale entries so a dropped-off kite doesn't skew the target.
- **Respool interaction:** a kite in `respool_mode` should probably ignore equalize (its length
  reference is being redefined). Skip equalize while `respool_mode` is true.
- **Exit:** target-seeking latches on; the pilot resumes manual control the moment they move a
  stick (the motion loop already prefers an active remote command over autopilot,
  [main.cpp:1131-1156](../src/main.cpp#L1131)).

### LED
Remote NeoPixel: **pulsing cyan** for a short confirmation window (~2 s) after trigger, or until
manual input resumes. Optional per-kite: the kite red LED **blinks while it is still seeking** the
target and goes steady/off when within the deadband.

---

## Feature 2 — Move all kites together (manual group fly)

### What it does
While a "group" gesture is held, one **master joystick's** Y-axis command is applied to **every**
kite simultaneously; releasing exits. Each kite still runs its own unspool tension gate and
over-retraction block, so the group command is self-limited per kite (a slack kite won't pay out
just because the fleet is).

### Design
Two clean ways to fan the command out from the remote:
- **Broadcast one `MSG_CONTROL`** to the broadcast MAC (the remote already registers the broadcast
  peer, [remote_control.cpp:231-237](../src/remote_control.cpp#L231)). Every kite's receive callback
  acts on it — exactly the desired "all move together," and only **1 packet** per tick instead of 3.
- **Loop-unicast** the same command to each assigned kite MAC (guaranteed-delivery-style, matches
  today's per-kite path; 3× packets).

**Recommendation: broadcast** for group-move (cheapest on the air, and semantically it *is* a
group command). **Suppress the normal per-joystick unicast while group mode is active** so the two
paths don't fight.

### Trigger / ergonomics
"Push-and-hold, then steer." Cleanest: **hold a designated button** (e.g. the middle joystick's
switch, or the same 1+3 chord *held*), which enters group mode; while held, that joystick's Y drives
the broadcast; **release exits**. Because it's a *hold* (not a click), suppress that joystick's
click actions on its kite for the duration (same suppression trick as the chord).

Decide which stick is "master":
- If entry is a **chord**, the master is ambiguous — pick a fixed one (e.g. joystick 2, the middle)
  and document it.
- If entry is a **hold on one button**, that same joystick is the natural master.

### Safety / caveats
- **Per-kite gates still apply** — this is the big safety win; group-move can't override a kite's own
  tension/over-retraction protection.
- **Failsafe:** the kite already zeros velocity when no fresh remote command arrives
  ([main.cpp:1147-1156](../src/main.cpp#L1147)); a dropped group packet just coasts to neutral, same
  as normal control. Keep sending at the normal 50 Hz while held.
- **Clean exit:** on release, send an explicit zero (broadcast) so all kites stop together rather
  than relying on the failsafe timeout.

### LED
Remote NeoPixel: **solid magenta/purple while group mode is held** (distinct from the white/red
single-kite command colours). Optional per-kite: kite red LED **solid on** while receiving group
commands.

---

## LED indication — fitting the existing state machine

The remote LED logic ([remote_control.cpp:291-329](../src/remote_control.cpp#L291)) currently
chooses colour from the *dominant* joystick command, with an idle heartbeat. Add a **mode check
that takes priority** at the top of that block:

| State | Colour | Notes |
|---|---|---|
| Group-move held | solid magenta | overrides command colours |
| Equalize triggered | pulsing cyan (~2 s) | brief confirmation, then back to normal |
| (existing) extending | white, brightness = speed | unchanged |
| (existing) retracting | red, brightness = speed | unchanged |
| (existing) idle | dim-blue heartbeat, N blips | unchanged |

Keep the joint-flight colours clearly distinct from white/red so mode vs. direction is never
ambiguous. Kite-side red LED is optional and, being single-colour, is best used as blink-while-busy
/ solid-while-grouped.

---

## Suggested phasing

1. **Feature 2 (move-all-together) first** — it needs *no* new telemetry, just a broadcast control
   path + a hold gesture + LED. Smallest, and it exercises the gesture/suppression plumbing that
   Feature 1 also needs.
2. **Add the shared-length telemetry** (`MSG_KITE_STATE`, distributed) — testable on its own by
   printing each kite's view of the fleet's lengths.
3. **Feature 1 (equalize)** on top of that telemetry, reusing target-seeking.
4. **LED states** alongside each.

## Open questions for Sean
- Gesture preference: **1+3 chord** vs **long-press** vs **dedicated button**?
- For group-move, which stick is master (middle stick? the one you hold?).
- Equalize telemetry: distributed (robust) vs host-centric (lighter traffic)?
- Any concern about the extra broadcast traffic given the earlier ESP-NOW load discussion?

---

## Changes made

| Date | File / area | Change | Notes / conditions tested |
|---|---|---|---|
| 2026-07-22 | `fleet_protocol.h` | `FleetControlMsg.button` → `event` + `event_seq` (5→6 bytes); added `FleetControlEvent` enum and `REMOTE_EVENT_REPEAT_MS`. | Builds clean (both envs). **Not flashed / not bench-tested.** |
| 2026-07-22 | `remote_control.cpp` | Parse clicks → semantic events on the remote; repeat 120 ms with rolling seq; force-send during the resend window. | Builds `remote-control` env OK. |
| 2026-07-22 | `main.cpp` | Removed kite-side click detection; apply events once per new seq; extracted `toggleRespoolMode()`. | Builds `rfx-4-main` env OK. Behaviour intended to be identical to prior click handling. |

**Still to verify on hardware:** single-click still toggles target-seek; triple-click still toggles
respool; no double-fire; no missed action under packet loss; legacy-remote velocity/stop unaffected.

