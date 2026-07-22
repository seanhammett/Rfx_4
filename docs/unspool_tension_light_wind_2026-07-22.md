# Unspool (line pay-out) tension protection — light-wind review

**Date:** 2026-07-22
**Branch:** `fix/remote-command-sticking`
**Status:** Analysis + proposed tuning. **No parameter changes have been made yet** — this
documents current behaviour and the options. The "Changes made" section at the bottom is a
placeholder to fill in when/if we tune on hardware.

---

## Observation

Flying in light-ish wind (~2–4 m/s), the kites did not put enough tension on the line to let
the line extend (unspool) quickly. Commanding pay-out either crawled or did nothing.

This is the tension-based unspool speed limiter behaving as designed — but tuned for a stronger
wind envelope than we were flying in.

---

## Where the logic lives

`applySafetyLimits()` — [src/main.cpp:935](../src/main.cpp#L935).

It runs **every motion tick, after the target velocity is chosen** (joystick *or* autopilot) and
**before** `sendMotorCommand()` streams it to the Moteus. Call order in the loop:

```
processJoystickInput() / autopilot.update()   → sets commanded_velocity
applySafetyLimits()                           → clamps commanded_velocity   (main.cpp:1188)
sendMotorCommand()                            → streams to motor            (main.cpp:1189)
```

Because it sits downstream of the velocity source, the gate applies **uniformly** to manual
remote control and to autopilot target-seeking. It is the single choke point for pay-out.

### Sign convention
- `commanded_velocity < 0`  → **unspool / pay line out** (extend)
- `commanded_velocity > 0`  → **retract / reel in**
- `line_length` is positive when extended; `calculateLineLength()` negates motor position
  ([src/main.cpp:268](../src/main.cpp#L268)).

---

## How the gate works (the three zones)

Only the unspool direction (`commanded_velocity < 0`) is gated. It reads the motor's measured
torque, `torque = abs(result.torque)`, and clamps the allowed pay-out speed
([src/main.cpp:944-969](../src/main.cpp#L944)):

| Measured torque | Line tension\* | Pay-out behaviour |
|---|---|---|
| `< MIN_TENSION_TORQUE` (0.015 Nm) | `< ~45 gf` | **Blocked** — `commanded_velocity = 0` |
| `MIN … FULL` (0.015 – 0.08 Nm) | `~45 – 240 gf` | **Ramped** — max speed scales linearly `MIN_UNSPOOL_SPEED → MAX_VELOCITY_RPS` |
| `≥ FULL_TENSION_TORQUE` (0.08 Nm) | `≥ ~240 gf` | Full commanded speed allowed |

\* tension (N) = torque (Nm) ÷ spool radius (0.034 m); grams-force = N ÷ 9.81 × 1000.

Ramp formula (for the middle zone):
```
t         = (torque - MIN_TENSION_TORQUE) / (FULL_TENSION_TORQUE - MIN_TENSION_TORQUE)
max_speed = MIN_UNSPOOL_SPEED + t * (MAX_VELOCITY_RPS - MIN_UNSPOOL_SPEED)   // rev/s
```

**Design intent:** a self-limiting servo. It only lets you release line as fast as the kite is
actually pulling it out, so you can't over-run the kite and create slack (which birdnests the
spool and tangles the line).

---

## Current parameters

Declared at [src/main.cpp:96-100](../src/main.cpp#L96):

| Constant | Value | In physical units | Meaning |
|---|---|---|---|
| `MAX_VELOCITY_RPS` | 8.0 rev/s | **1.71 m/s** line speed | Max pay-out / reel speed |
| `MIN_TENSION_TORQUE` | 0.015 Nm | **~45 gf** line tension | Below this: pay-out blocked |
| `FULL_TENSION_TORQUE` | 0.08 Nm | **~240 gf** line tension | Above this: full-speed pay-out |
| `MIN_UNSPOOL_SPEED` | 0.5 rev/s | **~11 cm/s** line speed | Floor speed at the block threshold |
| `SPOOL_DIAMETER` | 68 mm | radius 0.034 m | Sets torque↔tension and rev↔metre conversion |

Geometry: `meters_per_rev = π × 0.068 = 0.2136 m`.

Worked ramp points (measured torque → allowed pay-out speed):

| Torque (Nm) | ≈ tension | Allowed speed |
|---|---|---|
| 0.015 | 45 gf | 0.5 rev/s (0.11 m/s) |
| 0.030 | 90 gf | 2.2 rev/s (0.48 m/s) |
| 0.050 | 150 gf | 4.5 rev/s (0.97 m/s) |
| 0.080 | 240 gf | 8.0 rev/s (1.71 m/s) — full |

**So to unspool at anywhere near full speed you need ~240 gf of kite pull.** In 2–4 m/s wind a
small/moderate kite — especially at short line length or low flying angle — often pulls well
below that, and near stall below 45 gf, so pay-out is throttled to a crawl or blocked.

### Related (separate) path — autopilot slack reeling
Autopilot mode has its own low-tension logic set up at
[src/main.cpp:811-812](../src/main.cpp#L811): `setSlackTorqueThreshold(0.015, 0.025)` and
`setSlackReelSpeed(0.1, 8.0, 4.0)`. This governs **reel-in on detected slack**, not manual
pay-out, but it shares the same tension scale, so if we recalibrate the tension thresholds we
should keep these consistent with the new numbers.

---

## Why it manifests in light wind (not a bug)

Line tension **is** the kite's pull. The gate throttles pay-out proportional to that pull, so:

- Strong wind → kite pulls hard → tension ≥ 240 gf → full-speed pay-out. Feels great.
- Light wind → kite pulls softly → tension in the 45–240 gf band (or under 45) → pay-out ramped
  down or blocked. Feels stuck.

There is no chicken-and-egg trap: the kite builds tension by *flying*, not by us paying out line,
so refusing to pay out slack line is correct. The issue is purely that the **thresholds assume a
stronger-wind tension envelope** than we were flying in.

### Two secondary factors
1. **Torque sensing floor.** Moteus infers torque from q-axis current × torque constant. At
   0.015–0.03 Nm the estimate is dominated by cogging, bearing/gearbox friction/stiction and
   current-sense noise. The bottom of our range is close to un-measurable, which bounds how far
   `MIN_TENSION_TORQUE` can safely be lowered before noise alone triggers pay-out.
2. **Raw (unfiltered) torque in the gate.** The gate uses `result.torque` directly, whereas the
   autopilot filters torque (`setTorqueFilterAlpha(0.3)`). In gusty light air the raw signal
   chatters, so the ramp speed can jump around.

---

## Proposed tuning options (not yet applied)

Ordered roughly by leverage-for-light-wind vs. risk. None of these change the *structure* of the
protection — they retune where it engages. **Change one lever at a time and test.**

### A. Lower `FULL_TENSION_TORQUE` — biggest lever, low risk
Reaching full pay-out speed at 240 gf is the main constraint. Dropping it lets a lighter pull
reach full speed. e.g. `0.08 → 0.04 Nm` means full speed at **~120 gf** instead of 240 gf, and
doubles the allowed speed everywhere in the ramp.
- **Risk:** low. You still can't pay out with *no* tension (the block threshold is unchanged);
  you just reach full speed at a more realistic light-wind pull.

### B. Raise `MIN_UNSPOOL_SPEED` — faster minimum crawl, low/medium risk
`0.5 → 1.0` rev/s doubles the floor speed (11 → 21 cm/s) so marginal-tension flying isn't a
crawl.
- **Risk:** low/medium. A higher floor means that the *instant* tension crosses the block
  threshold you pay out faster — slightly more prone to momentarily out-running the kite.

### C. Lower `MIN_TENSION_TORQUE` — cautiously
`0.015 → 0.010 Nm` (~30 gf) lets very light pulls unspool at all.
- **Risk:** medium. This is already near the torque noise floor (see above); going lower risks
  noise-triggered pay-out when the line is genuinely slack. If we try this, pair it with option E
  (filter the gate torque) so noise doesn't punch through the threshold.

### D. Non-linear ramp — optional refinement
Replace the linear ramp with one that rises faster just above the block threshold (e.g. sqrt or a
lower knee), so light-wind pulls get usable speed sooner while very-low tension is still gentle.
- **Risk:** low, but more code than tweaking constants; do A/B first and only add this if the
  linear ramp still feels wrong.

### E. Filter the torque used by the gate — robustness, not speed
Feed a low-pass-filtered torque (like the autopilot's alpha 0.3) into `applySafetyLimits()`
instead of raw `result.torque`, to stop the ramp from chattering in gusts. Enables safely
lowering thresholds (C).
- **Risk:** low. Adds a little lag to how fast the gate reacts to a real tension drop — keep the
  filter light.

### F. Manual "force unspool" override — escape hatch
Analogous to the existing `respool_mode` triple-click bypass
([src/main.cpp:125-127](../src/main.cpp#L125), toggled at
[src/main.cpp:878](../src/main.cpp#L878)): a pilot-triggered mode that temporarily relaxes/bypasses
the tension gate for deliberate hand-paced pay-out in light wind (e.g. when walking the kite out).
- **Risk:** medium — it's an intentional safety bypass, so it must be explicit, momentary/obvious,
  and ideally speed-capped rather than fully open.

### Suggested first pass on hardware
Start with **A (`FULL_TENSION_TORQUE` → ~0.04 Nm)** alone and re-fly the same conditions. If the
low end is still a crawl, add **B (`MIN_UNSPOOL_SPEED` → ~1.0 rev/s)**. Only reach for **C + E**
if you need to unspool at genuinely tiny pulls. Keep the autopilot slack thresholds
(main.cpp:811-812) consistent with whatever tension scale we settle on.

---

## Changes made

_None yet — analysis only. Record here what we actually change, with before/after values and the
wind/tension conditions we validated in._

| Date | Parameter | Old | New | Notes / conditions tested |
|---|---|---|---|---|
| | | | | |


