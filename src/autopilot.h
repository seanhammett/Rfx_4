#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <Arduino.h>
#include "detector_params.h"

// ===== Autopilot Modes =====
enum class AutopilotMode : uint8_t {
  AP_DISABLED      = 0,
  AP_HOLDING       = 1,
  AP_SLACK         = 2,
  AP_DIVE_RECOVERY = 3,
  AP_AWW_RETURN    = 4
};

const char* autopilotModeName(AutopilotMode mode);

// ===== Detector Results =====
struct DetectorState {
  float dive_confidence = 0.0f;      // 0.0 - 1.0: confidence that kite is diving
  float aww_confidence = 0.0f;       // 0.0 - 1.0: confidence that kite is away from wind
  float wind_direction_deg = 0.0f;   // estimated prevailing wind direction (degrees)
  float aww_angle_offset_deg = 0.0f; // current angular offset from wind (degrees)
  float active_flight_confidence = 0.0f; // 0.0 - 1.0: confidence that kite is actively flying
};

class Autopilot {
public:
  // ===== Configuration (call before enable or anytime) =====
  void setTargetLineLength(float target_m);
  void setTorqueFilterAlpha(float alpha);
  void setSlackTorqueThreshold(float enter_nm, float exit_nm);
  void setSlackReelSpeed(float min_speed_rps, float max_speed_rps, float ramp_time_s);
  void setMaxVelocity(float max_vel_rps);
  void setAcceleration(float accel_rps2, float decel_rps2);
  void setSpoolDiameter(float diameter_mm);
  void setDeadband(float deadband_m);

  // ===== Enable / Disable =====
  void enable();
  void disable();
  bool isEnabled() const { return _enabled; }

  // ===== Main update — call once per control cycle =====
  // current_line_length_m : estimated unspooled line length (positive = extended)
  // current_torque_nm     : absolute motor torque reading
  // dt                    : time step in seconds
  // Returns: commanded velocity in rev/s (positive = retract, negative = unspool)
  float update(float current_line_length_m, float current_torque_nm, float dt);

  // ===== Adjust target from joystick while autopilot is active =====
  // rate_mps: target adjustment rate in m/s (since target is in meters)
  void adjustTarget(float rate_mps, float dt);

  // ===== State queries =====
  bool isSlackDetected() const { return _slack_detected; }
  float getTargetLineLength() const { return _target_line_length; }
  float getCurrentVelocityRPS() const { return _current_velocity_rps; }

  float getFilteredTorque() const { return _filtered_torque; }

  // ===== State machine =====
  AutopilotMode getMode() const { return _mode; }

  // ===== Detectors =====
  // Call updateDetectors() each cycle with IMU data, independent of autopilot enable state
  void updateDetectors(float pitch_deg, float pitch_velocity_dps, float yaw_deg, float tension_n, float line_length_m, float dt);
  const DetectorState& getDetectors() const { return _detectors; }
  
  // Detector configuration
  void setDiveDetectorParams(float pitch_rate_threshold_dps, float tension_threshold_n, 
                              float attack_rate, float decay_rate);
  void setAwwDetectorParams(float angle_threshold_deg, float attack_rate, float decay_rate,
                            float wind_alpha);
  void setActiveFlightDetectorParams(float pitch_min_deg, float line_length_min_m,
                                     float tension_min_n, float variation_threshold,
                                     float attack_rate, float decay_rate);

private:
  bool _enabled = false;

  // Target & deadband
  float _target_line_length = 1.0;   // meters
  float _deadband_m = 0.01;          // 1 cm

  // Velocity & acceleration limits (in rev/s)
  float _max_velocity_rps = 2.0;
  float _acceleration_rps2 = 1.0;
  float _deceleration_rps2 = 5.0;     // Faster decel for quick stops
  float _current_velocity_rps = 0.0;

  // Slack detection
  float _slack_torque_enter = 0.01;   // Nm — torque below this enters slack
  float _slack_torque_exit = 0.02;    // Nm — torque must exceed this to clear slack
  float _slack_reel_min_rps = 0.5;    // Initial reel-in speed (rev/s)
  float _slack_reel_max_rps = 4.0;    // Max reel-in speed after ramp (rev/s)
  float _slack_ramp_time_s = 2.0;     // Seconds to ramp from min to max
  float _slack_duration_s = 0.0;      // How long slack has persisted
  bool _slack_detected = false;
  bool _slack_recovering = false;     // True while decelerating after slack clears

  // Torque filtering (EMA)
  float _torque_filter_alpha = 0.3;   // 0..1, lower = smoother
  float _filtered_torque = 0.0;
  bool _torque_filter_initialized = false;

  // Spool geometry
  float _spool_diameter_mm = 68.0;
  float _meters_per_rev = 0.2136;     // Cached: PI * diameter / 1000

  void _updateMetersPerRev();
  float _distanceToRevolutions(float meters) const;

  // ===== State machine =====
  AutopilotMode _mode = AutopilotMode::AP_DISABLED;
  AutopilotMode _prev_mode = AutopilotMode::AP_DISABLED;
  unsigned long _mode_entered_ms = 0;

  void _evaluateStateTransitions(float current_line_length_m);
  float _updateHolding(float current_line_length_m, float dt);
  float _updateSlack(float dt);
  float _updateDiveRecovery(float dt);
  float _updateAwwReturn(float dt);

  // State transition thresholds
  float _dive_recovery_enter = DIVE_RECOVERY_CONFIDENCE_ENTER;
  float _dive_recovery_exit = DIVE_RECOVERY_CONFIDENCE_EXIT;
  float _dive_recovery_retract_rps = DIVE_RECOVERY_RETRACT_RPS;
  float _dive_recovery_max_pitch = DIVE_RECOVERY_MAX_PITCH;
  float _aww_return_enter = AWW_RETURN_CONFIDENCE_ENTER;
  float _aww_return_exit = AWW_RETURN_CONFIDENCE_EXIT;
  float _aww_return_retract_rps = AWW_RETURN_RETRACT_RPS;

  // ===== Detectors =====
  DetectorState _detectors;
  float _last_pitch_deg = 0.0f;  // stored from updateDetectors() for use in state handlers
  
  // Dive detector parameters (defaults from detector_params.h)
  float _dive_pitch_rate_threshold = DIVE_PITCH_RATE_THRESHOLD;
  float _dive_tension_threshold = DIVE_TENSION_THRESHOLD;
  float _dive_attack_rate = DIVE_ATTACK_RATE;
  float _dive_decay_rate = DIVE_DECAY_RATE;
  float _filtered_pitch_velocity = 0.0f;
  float _pitch_velocity_alpha = DIVE_PITCH_VELOCITY_ALPHA;

  // Away-from-wind detector parameters (defaults from detector_params.h)
  float _aww_angle_threshold = AWW_ANGLE_THRESHOLD;
  float _aww_attack_rate = AWW_ATTACK_RATE;
  float _aww_decay_rate = AWW_DECAY_RATE;
  float _aww_wind_alpha = AWW_WIND_ALPHA;
  float _wind_sin_avg = 0.0f;   // EMA of sin(yaw) for circular mean
  float _wind_cos_avg = 0.0f;   // EMA of cos(yaw) for circular mean
  bool  _wind_initialized = false;

  // Active-flight detector parameters (defaults from detector_params.h)
  float _af_pitch_min = AF_PITCH_MIN;
  float _af_line_length_min = AF_LINE_LENGTH_MIN;
  float _af_tension_min = AF_TENSION_MIN;
  float _af_variation_threshold = AF_VARIATION_THRESHOLD;
  float _af_attack_rate = AF_ATTACK_RATE;
  float _af_decay_rate = AF_DECAY_RATE;
  float _af_variation_alpha = AF_VARIATION_ALPHA;
  // EMA-smoothed values for variation tracking
  float _af_pitch_ema = 0.0f;
  float _af_yaw_ema = 0.0f;
  float _af_tension_ema = 0.0f;
  // EMA-smoothed absolute deviation (variation)
  float _af_pitch_var = 0.0f;
  float _af_yaw_var = 0.0f;
  float _af_tension_var = 0.0f;
  bool  _af_initialized = false;
};

#endif // AUTOPILOT_H
