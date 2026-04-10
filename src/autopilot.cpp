#include "autopilot.h"

// ===== Mode name helper =====
const char* autopilotModeName(AutopilotMode mode) {
  switch (mode) {
    case AutopilotMode::AP_DISABLED:      return "DISABLED";
    case AutopilotMode::AP_HOLDING:       return "HOLDING";
    case AutopilotMode::AP_SLACK:         return "SLACK";
    case AutopilotMode::AP_DIVE_RECOVERY: return "DIVE_RECOVERY";
    case AutopilotMode::AP_AWW_RETURN:    return "AWW_RETURN";
    default:                           return "UNKNOWN";
  }
}

// ===== Internal helpers =====

void Autopilot::_updateMetersPerRev() {
  _meters_per_rev = PI * _spool_diameter_mm / 1000.0f;
}

float Autopilot::_distanceToRevolutions(float meters) const {
  return meters / _meters_per_rev;
}

// ===== Configuration setters =====

void Autopilot::setTargetLineLength(float target_m) {
  _target_line_length = max(0.0f, target_m);
}

void Autopilot::setSlackTorqueThreshold(float enter_nm, float exit_nm) {
  _slack_torque_enter = enter_nm;
  _slack_torque_exit = max(enter_nm, exit_nm);  // exit must be >= enter
}

void Autopilot::setSlackReelSpeed(float min_speed_rps, float max_speed_rps, float ramp_time_s) {
  _slack_reel_min_rps = min_speed_rps;
  _slack_reel_max_rps = max_speed_rps;
  _slack_ramp_time_s = max(0.01f, ramp_time_s);
}

void Autopilot::setMaxVelocity(float max_vel_rps) {
  _max_velocity_rps = max_vel_rps;
}

void Autopilot::setAcceleration(float accel_rps2, float decel_rps2) {
  _acceleration_rps2 = accel_rps2;
  _deceleration_rps2 = decel_rps2;
}

void Autopilot::setSpoolDiameter(float diameter_mm) {
  _spool_diameter_mm = diameter_mm;
  _updateMetersPerRev();
}

void Autopilot::setDeadband(float deadband_m) {
  _deadband_m = deadband_m;
}

void Autopilot::setTorqueFilterAlpha(float alpha) {
  _torque_filter_alpha = constrain(alpha, 0.01f, 1.0f);
}

// ===== Enable / Disable =====

void Autopilot::enable() {
  if (!_enabled) {
    _enabled = true;
    _current_velocity_rps = 0.0;
    _slack_detected = false;
    _slack_duration_s = 0.0;
    _torque_filter_initialized = false;
    _mode = AutopilotMode::AP_HOLDING;
    _prev_mode = AutopilotMode::AP_HOLDING;
    _mode_entered_ms = millis();
    Serial.printf("[Autopilot] ENABLED  target=%.2f m\n", _target_line_length);
  }
}

void Autopilot::disable() {
  if (_enabled) {
    _enabled = false;
    _current_velocity_rps = 0.0;
    _slack_detected = false;
    _slack_duration_s = 0.0;
    _torque_filter_initialized = false;
    _mode = AutopilotMode::AP_DISABLED;
    Serial.println("[Autopilot] DISABLED");
  }
}

// ===== Adjust target from joystick =====

void Autopilot::adjustTarget(float rate_mps, float dt) {
  _target_line_length -= rate_mps * dt;
  if (_target_line_length < 0.0f) {
    _target_line_length = 0.0f;
  }
}

// ===== Main update =====

float Autopilot::update(float current_line_length_m, float current_torque_nm, float dt) {
  if (!_enabled) {
    _current_velocity_rps = 0.0;
    _mode = AutopilotMode::AP_DISABLED;
    return 0.0;
  }

  // --- Filter torque (EMA) ---
  if (!_torque_filter_initialized) {
    _filtered_torque = current_torque_nm;
    _torque_filter_initialized = true;
  } else {
    _filtered_torque += _torque_filter_alpha * (current_torque_nm - _filtered_torque);
  }

  // --- Update slack detection flags (used by state transitions) ---
  if (_slack_detected) {
    if (_filtered_torque >= _slack_torque_exit) {
      _slack_detected = false;
    }
  } else {
    if (_filtered_torque < _slack_torque_enter) {
      _slack_detected = true;
    }
  }

  // --- Evaluate state transitions (priority-ordered) ---
  _evaluateStateTransitions(current_line_length_m);

  // --- Run active state handler ---
  float desired_velocity_rps = 0.0;
  switch (_mode) {
    case AutopilotMode::AP_SLACK:
      desired_velocity_rps = _updateSlack(dt);
      break;
    case AutopilotMode::AP_DIVE_RECOVERY:
      desired_velocity_rps = _updateDiveRecovery(dt);
      break;
    case AutopilotMode::AP_AWW_RETURN:
      desired_velocity_rps = _updateAwwReturn(dt);
      break;
    case AutopilotMode::AP_HOLDING:
    default:
      desired_velocity_rps = _updateHolding(current_line_length_m, dt);
      break;
  }

  // --- Acceleration / deceleration limiting ---
  if (_mode == AutopilotMode::AP_SLACK) {
    _current_velocity_rps = desired_velocity_rps;
  } else {
    if (_prev_mode == AutopilotMode::AP_SLACK && _mode != AutopilotMode::AP_SLACK) {
      _slack_recovering = true;
    }
    if (_slack_recovering && fabs(_current_velocity_rps - desired_velocity_rps) < 0.01f) {
      _slack_recovering = false;
    }
    float rate = _slack_recovering ? _deceleration_rps2 : _acceleration_rps2;
    float max_change = rate * dt;
    float vel_error = desired_velocity_rps - _current_velocity_rps;

    if (fabs(vel_error) > max_change) {
      _current_velocity_rps += (vel_error > 0) ? max_change : -max_change;
    } else {
      _current_velocity_rps = desired_velocity_rps;
    }
  }

  return _current_velocity_rps;
}

// ===== State transition evaluation (priority-ordered) =====

void Autopilot::_evaluateStateTransitions(float current_line_length_m) {
  AutopilotMode new_mode = _mode;

  // Priority 1: SLACK — always wins
  if (_slack_detected) {
    new_mode = AutopilotMode::AP_SLACK;
  }
  // Priority 2: DIVE_RECOVERY
  else if (_detectors.dive_confidence >= _dive_recovery_enter) {
    new_mode = AutopilotMode::AP_DIVE_RECOVERY;
  }
  // Priority 3: AWW_RETURN
  else if (_detectors.aww_confidence >= _aww_return_enter) {
    new_mode = AutopilotMode::AP_AWW_RETURN;
  }
  // Exit conditions for current state → fall back to HOLDING
  else {
    switch (_mode) {
      case AutopilotMode::AP_DIVE_RECOVERY:
        if (_detectors.dive_confidence < _dive_recovery_exit) {
          new_mode = AutopilotMode::AP_HOLDING;
        } else {
          new_mode = _mode; // stay in dive recovery
        }
        break;
      case AutopilotMode::AP_AWW_RETURN:
        if (_detectors.aww_confidence < _aww_return_exit) {
          new_mode = AutopilotMode::AP_HOLDING;
        } else {
          new_mode = _mode; // stay in aww return
        }
        break;
      default:
        new_mode = AutopilotMode::AP_HOLDING;
        break;
    }
  }

  // Log transitions
  if (new_mode != _mode) {
    Serial.printf("[Autopilot] %s -> %s (dive=%.2f aww=%.2f slack=%d)\n",
                  autopilotModeName(_mode), autopilotModeName(new_mode),
                  _detectors.dive_confidence, _detectors.aww_confidence, _slack_detected);
    _prev_mode = _mode;
    _mode = new_mode;
    _mode_entered_ms = millis();

    // Reset slack duration on entry/exit
    if (_mode == AutopilotMode::AP_SLACK) {
      _slack_duration_s = 0.0;
    }
  }
}

// ===== State handlers =====

float Autopilot::_updateHolding(float current_line_length_m, float dt) {
  float line_length_error = _target_line_length - current_line_length_m;

  if (fabs(line_length_error) > _deadband_m) {
    float distance_rev = _distanceToRevolutions(fabs(line_length_error));
    float max_safe_vel = sqrtf(2.0f * _acceleration_rps2 * distance_rev);
    float clamped_vel = min(max_safe_vel, _max_velocity_rps);

    // Positive error → target longer → unspool (negative vel)
    // Negative error → target shorter → retract (positive vel)
    return (line_length_error > 0) ? -clamped_vel : clamped_vel;
  }
  return 0.0f;
}

float Autopilot::_updateSlack(float dt) {
  _slack_duration_s += dt;

  float ramp_t = min(_slack_duration_s / _slack_ramp_time_s, 1.0f);
  float reel_speed = _slack_reel_min_rps + ramp_t * (_slack_reel_max_rps - _slack_reel_min_rps);

  static unsigned long lastSlackMsg = 0;
  if (millis() - lastSlackMsg > 500) {
    Serial.printf("[Autopilot] SLACK (%.1fs) torque=%.4f Nm — reeling at %.2f rev/s (ramp %.0f%%)\n",
                  _slack_duration_s, _filtered_torque, reel_speed, ramp_t * 100.0f);
    lastSlackMsg = millis();
  }

  return reel_speed; // positive = retract
}

float Autopilot::_updateDiveRecovery(float dt) {
  // Retract at controlled rate, but scale down as pitch approaches vertical
  // to prevent kite from going overhead
  float speed = _dive_recovery_retract_rps;

  if (_last_pitch_deg > _dive_recovery_max_pitch) {
    // Linear ramp-down: full speed at max_pitch, zero at 90°
    float remaining = max(0.0f, 90.0f - _last_pitch_deg);
    float range = max(1.0f, 90.0f - _dive_recovery_max_pitch);
    speed *= (remaining / range);
  }

  return speed;
}

float Autopilot::_updateAwwReturn(float dt) {
  // Gentle retract — shortening line increases tension, pulling kite back toward wind window
  return _aww_return_retract_rps;
}

// ===== Detectors =====

void Autopilot::updateDetectors(float pitch_deg, float pitch_velocity_dps, float yaw_deg, float tension_n, float line_length_m, float dt) {
  _last_pitch_deg = pitch_deg;

  // --- Dive detector ---

  // Filter pitch velocity with EMA for smoother detection
  if (_filtered_pitch_velocity == 0.0f) {
    _filtered_pitch_velocity = pitch_velocity_dps;
  } else {
    _filtered_pitch_velocity += _pitch_velocity_alpha * (pitch_velocity_dps - _filtered_pitch_velocity);
  }

  // Dive detection: pitch dropping consistently AND high tension
  bool pitch_dropping = _filtered_pitch_velocity < _dive_pitch_rate_threshold;
  bool high_tension = tension_n > _dive_tension_threshold;

  if (pitch_dropping && high_tension) {
    _detectors.dive_confidence += _dive_attack_rate * dt;
    if (_detectors.dive_confidence > 1.0f) _detectors.dive_confidence = 1.0f;
  } else {
    _detectors.dive_confidence -= _dive_decay_rate * dt;
    if (_detectors.dive_confidence < 0.0f) _detectors.dive_confidence = 0.0f;
  }

  // --- Away-from-wind detector ---

  // Update prevailing wind estimate using circular EMA (sin/cos components)
  float yaw_rad = yaw_deg * (PI / 180.0f);
  float sin_yaw = sinf(yaw_rad);
  float cos_yaw = cosf(yaw_rad);

  if (!_wind_initialized) {
    _wind_sin_avg = sin_yaw;
    _wind_cos_avg = cos_yaw;
    _wind_initialized = true;
  } else {
    _wind_sin_avg += _aww_wind_alpha * (sin_yaw - _wind_sin_avg);
    _wind_cos_avg += _aww_wind_alpha * (cos_yaw - _wind_cos_avg);
  }

  float wind_dir_rad = atan2f(_wind_sin_avg, _wind_cos_avg);
  _detectors.wind_direction_deg = wind_dir_rad * (180.0f / PI);

  // Angular offset from wind (shortest arc, -180 to +180)
  float offset = yaw_deg - _detectors.wind_direction_deg;
  // Normalize to [-180, 180]
  while (offset > 180.0f) offset -= 360.0f;
  while (offset < -180.0f) offset += 360.0f;
  _detectors.aww_angle_offset_deg = offset;

  if (fabsf(offset) > _aww_angle_threshold) {
    _detectors.aww_confidence += _aww_attack_rate * dt;
    if (_detectors.aww_confidence > 1.0f) _detectors.aww_confidence = 1.0f;
  } else {
    _detectors.aww_confidence -= _aww_decay_rate * dt;
    if (_detectors.aww_confidence < 0.0f) _detectors.aww_confidence = 0.0f;
  }

  // --- Active-flight detector ---

  // Track variation using EMA of absolute deviation
  if (!_af_initialized) {
    _af_pitch_ema = pitch_deg;
    _af_yaw_ema = yaw_deg;
    _af_tension_ema = tension_n;
    _af_pitch_var = 0.0f;
    _af_yaw_var = 0.0f;
    _af_tension_var = 0.0f;
    _af_initialized = true;
  } else {
    // Update EMA of each signal
    _af_pitch_ema += _af_variation_alpha * (pitch_deg - _af_pitch_ema);
    _af_yaw_ema += _af_variation_alpha * (yaw_deg - _af_yaw_ema);
    _af_tension_ema += _af_variation_alpha * (tension_n - _af_tension_ema);

    // Track variation as EMA of absolute deviation from the smoothed mean
    _af_pitch_var += _af_variation_alpha * (fabsf(pitch_deg - _af_pitch_ema) - _af_pitch_var);
    _af_yaw_var += _af_variation_alpha * (fabsf(yaw_deg - _af_yaw_ema) - _af_yaw_var);
    _af_tension_var += _af_variation_alpha * (fabsf(tension_n - _af_tension_ema) - _af_tension_var);
  }

  // Combine variation signals (normalize tension variation by min threshold for comparable scale)
  float variation_score = _af_pitch_var + _af_yaw_var + (_af_tension_var / fmaxf(_af_tension_min, 0.01f));

  // All conditions for active flight
  bool has_variation = variation_score > _af_variation_threshold;
  bool pitch_ok = pitch_deg > _af_pitch_min;
  bool line_ok = line_length_m > _af_line_length_min;
  bool tension_ok = tension_n > _af_tension_min;

  if (has_variation && pitch_ok && line_ok && tension_ok) {
    _detectors.active_flight_confidence += _af_attack_rate * dt;
    if (_detectors.active_flight_confidence > 1.0f) _detectors.active_flight_confidence = 1.0f;
  } else {
    _detectors.active_flight_confidence -= _af_decay_rate * dt;
    if (_detectors.active_flight_confidence < 0.0f) _detectors.active_flight_confidence = 0.0f;
  }
}

void Autopilot::setDiveDetectorParams(float pitch_rate_threshold, float tension_threshold,
                                       float attack_rate, float decay_rate) {
  _dive_pitch_rate_threshold = pitch_rate_threshold;
  _dive_tension_threshold = tension_threshold;
  _dive_attack_rate = attack_rate;
  _dive_decay_rate = decay_rate;
}

void Autopilot::setAwwDetectorParams(float angle_threshold_deg, float attack_rate,
                                      float decay_rate, float wind_alpha) {
  _aww_angle_threshold = angle_threshold_deg;
  _aww_attack_rate = attack_rate;
  _aww_decay_rate = decay_rate;
  _aww_wind_alpha = wind_alpha;
}

void Autopilot::setActiveFlightDetectorParams(float pitch_min_deg, float line_length_min_m,
                                              float tension_min_n, float variation_threshold,
                                              float attack_rate, float decay_rate) {
  _af_pitch_min = pitch_min_deg;
  _af_line_length_min = line_length_min_m;
  _af_tension_min = tension_min_n;
  _af_variation_threshold = variation_threshold;
  _af_attack_rate = attack_rate;
  _af_decay_rate = decay_rate;
}
