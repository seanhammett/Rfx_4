#include "autopilot.h"

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
    return 0.0;
  }

  float desired_velocity_rps = 0.0;

  // --- Filter torque (EMA) ---
  if (!_torque_filter_initialized) {
    _filtered_torque = current_torque_nm;
    _torque_filter_initialized = true;
  } else {
    _filtered_torque += _torque_filter_alpha * (current_torque_nm - _filtered_torque);
  }

  // --- Slack detection with hysteresis (using filtered torque) ---
  bool was_slack = _slack_detected;
  if (_slack_detected) {
    if (_filtered_torque >= _slack_torque_exit) {
      _slack_detected = false;
    }
  } else {
    if (_filtered_torque < _slack_torque_enter) {
      _slack_detected = true;
    }
  }

  if (_slack_detected) {
    // Accumulate slack duration for speed ramp
    _slack_duration_s += dt;

    // Ramp retract speed: min → max over _slack_ramp_time_s (rev/s)
    float ramp_t = min(_slack_duration_s / _slack_ramp_time_s, 1.0f);
    float reel_speed = _slack_reel_min_rps + ramp_t * (_slack_reel_max_rps - _slack_reel_min_rps);

    // Reel in (positive velocity = retract)
    desired_velocity_rps = reel_speed;

    static unsigned long lastSlackMsg = 0;
    if (millis() - lastSlackMsg > 500) {
      Serial.printf("[Autopilot] SLACK (%.1fs) torque=%.4f Nm — reeling at %.2f rev/s (ramp %.0f%%)\n",
                    _slack_duration_s, current_torque_nm, reel_speed, ramp_t * 100.0f);
      lastSlackMsg = millis();
    }
  } else {
    if (was_slack) {
      Serial.printf("[Autopilot] Tension restored after %.1fs slack — resuming target seek to %.2f m\n",
                    _slack_duration_s, _target_line_length);
      _slack_duration_s = 0.0;
    }

    // --- Normal target seeking ---
    float line_length_error = _target_line_length - current_line_length_m;

    if (fabs(line_length_error) > _deadband_m) {
      // Convert distance to revolutions for kinematic calc
      float distance_rev = _distanceToRevolutions(fabs(line_length_error));
      // Kinematic limit: v = sqrt(2 * a * d) in rev/s
      float max_safe_vel = sqrtf(2.0f * _acceleration_rps2 * distance_rev);
      float clamped_vel = min(max_safe_vel, _max_velocity_rps);

      // Positive error means target is longer → unspool (negative vel)
      // Negative error means target is shorter → retract (positive vel)
      desired_velocity_rps = (line_length_error > 0) ? -clamped_vel : clamped_vel;
    }
  }

  // --- Acceleration / deceleration limiting ---
  // During slack, skip the accel limiter — the slack ramp already controls speed-up rate.
  // Only apply accel limiting for normal target seeking and slack recovery.
  if (_slack_detected) {
    _current_velocity_rps = desired_velocity_rps;
  } else {
    if (was_slack && !_slack_detected) {
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

// ===== Detectors =====

void Autopilot::updateDetectors(float pitch_deg, float pitch_velocity_dps, float yaw_deg, float tension_n, float dt) {
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
