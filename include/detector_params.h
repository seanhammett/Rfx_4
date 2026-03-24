#ifndef DETECTOR_PARAMS_H
#define DETECTOR_PARAMS_H

// ===== Detector Parameters — Single Source of Truth =====
// These values are used by the autopilot firmware AND served to the
// dashboards via the /status JSON endpoint.  Edit them here only.

// Dive detector
#define DIVE_PITCH_RATE_THRESHOLD  -5.0f   // deg/s — pitch rate below this indicates a dive
#define DIVE_TENSION_THRESHOLD      0.8f   // N     — tension above this indicates a loaded dive
#define DIVE_ATTACK_RATE            2.0f   // /s    — confidence rise rate when diving
#define DIVE_DECAY_RATE             1.0f   // /s    — confidence fall rate when not diving
#define DIVE_PITCH_VELOCITY_ALPHA   0.15f  // EMA smoothing factor for pitch velocity

#endif // DETECTOR_PARAMS_H
