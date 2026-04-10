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

// Away-from-wind detector
#define AWW_ANGLE_THRESHOLD        90.0f   // deg   — yaw offset from wind beyond this triggers detection
#define AWW_ATTACK_RATE             2.0f   // /s    — confidence rise rate when away from wind
#define AWW_DECAY_RATE              1.0f   // /s    — confidence fall rate when facing wind
#define AWW_WIND_ALPHA              0.005f // EMA smoothing for prevailing wind estimate (small = slower)

// Active-flight detector
#define AF_PITCH_MIN                5.0f   // deg   — pitch must be above this to be "flying"
#define AF_LINE_LENGTH_MIN          1.0f   // m     — line must be longer than this
#define AF_TENSION_MIN              0.3f   // N     — minimum tension to consider flying
#define AF_VARIATION_THRESHOLD      2.0f   // combined variation score threshold (pitch + yaw + tension)
#define AF_ATTACK_RATE              1.0f   // /s    — confidence rise rate
#define AF_DECAY_RATE               0.1f   // /s    — confidence fall rate (slow decay for stability)
#define AF_VARIATION_ALPHA          0.05f  // EMA smoothing for variation tracking

// State-machine transition thresholds
#define DIVE_RECOVERY_CONFIDENCE_ENTER  0.7f   // dive_confidence above this enters DIVE_RECOVERY
#define DIVE_RECOVERY_CONFIDENCE_EXIT   0.2f   // dive_confidence below this exits DIVE_RECOVERY
#define DIVE_RECOVERY_RETRACT_RPS       3.0f   // retract speed during dive recovery (rev/s)
#define DIVE_RECOVERY_MAX_PITCH        80.0f   // deg — reduce retract when pitch exceeds this (prevent going overhead)

#define AWW_RETURN_CONFIDENCE_ENTER     0.7f   // aww_confidence above this enters AWW_RETURN
#define AWW_RETURN_CONFIDENCE_EXIT      0.3f   // aww_confidence below this exits AWW_RETURN
#define AWW_RETURN_RETRACT_RPS          1.0f   // gentle retract to tighten line and let wind bring kite back

#endif // DETECTOR_PARAMS_H
