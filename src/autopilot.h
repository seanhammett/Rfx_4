#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <Arduino.h>

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
};

#endif // AUTOPILOT_H
