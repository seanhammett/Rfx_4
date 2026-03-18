#ifndef JOYSTICK_HANDLER_H
#define JOYSTICK_HANDLER_H

#include <Arduino.h>

class JoystickController {
public:
  // Constructor
  JoystickController(uint8_t x_pin, uint8_t y_pin, uint8_t sw_pin);
  
  // Initialize the joystick
  void begin();
  
  // Read joystick values and update state
  void update();
  
  // Getters
  int getX() const { return x; }
  int getY() const { return y; }
  int getRawX() const { return x_raw; }
  int getRawY() const { return y_raw; }
  bool getSwitch() const { return switch_pressed; }
  bool isInUse() const { return in_use; }
  int getMotorCommand() const { return motor_command; }
  
  // Click detection: returns 1 for single, 2 for double, 3 for triple click
  // Returns the click count once on the update cycle it is resolved, then 0
  uint8_t getClickCount() const { return resolved_clicks; }
  bool isSingleClick() const { return resolved_clicks == 1; }
  bool isDoubleClick() const { return resolved_clicks == 2; }
  bool isTripleClick() const { return resolved_clicks == 3; }
  
  // Setters for configuration
  void setDeadzone(int y_lower, int y_upper, int x_lower, int x_upper);
  void setSpeedLimits(int min_speed, int max_speed);
  void setFilterAlpha(float alpha);
  void setIdleTimeout(unsigned long timeout_ms);
  
private:
  // Pin assignments
  uint8_t pin_x;
  uint8_t pin_y;
  uint8_t pin_sw;
  
  // State variables
  float x_filtered;
  float y_filtered;
  int x_raw;
  int y_raw;
  int x;
  int y;
  int motor_command;
  bool switch_pressed;
  bool in_use;
  unsigned long last_use_time;
  
  // Configuration
  float filter_alpha;
  int y_center_lower;
  int y_center_upper;
  int x_center_lower;
  int x_center_upper;
  int max_speed;
  int min_speed;
  unsigned long idle_timeout;
  
  // Click detection state
  bool prev_switch_pressed = false;
  uint8_t click_count = 0;           // Clicks accumulated so far in this sequence
  uint8_t resolved_clicks = 0;       // Final click count exposed to caller (1/2/3), cleared each cycle
  unsigned long last_click_time = 0;  // Time of last rising edge (press)
  static const unsigned long MULTI_CLICK_WINDOW = 400;  // Max ms between clicks for multi-click
};

#endif // JOYSTICK_HANDLER_H
