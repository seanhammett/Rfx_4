#include "joystick_handler.h"

JoystickController::JoystickController(uint8_t x_pin, uint8_t y_pin, uint8_t sw_pin)
  : pin_x(x_pin), pin_y(y_pin), pin_sw(sw_pin),
    x_filtered(0), y_filtered(0), x_raw(0), y_raw(0),
    x(0), y(0), motor_command(0),
    switch_pressed(false), in_use(false), last_use_time(0),
    filter_alpha(0.7),
    y_center_lower(1900), y_center_upper(2100),
    x_center_lower(1900), x_center_upper(2100),
    max_speed(5000), min_speed(-5000),  // converting these for deprecated unitless to rpm*100
    idle_timeout(250) {
}

void JoystickController::begin() {
  pinMode(pin_x, INPUT);
  pinMode(pin_y, INPUT);
  pinMode(pin_sw, INPUT_PULLUP);
}

void JoystickController::update() {
  // Read raw values
  x_raw = analogRead(pin_x);
  y_raw = analogRead(pin_y);
  
  // Apply low-pass filter for smooth response
  x_filtered = filter_alpha * x_filtered + (1.0 - filter_alpha) * x_raw;
  y_filtered = filter_alpha * y_filtered + (1.0 - filter_alpha) * y_raw;
  
  x = (int)x_filtered;
  y = (int)y_filtered;
  switch_pressed = (digitalRead(pin_sw) == LOW);
  
  // --- Multi-click detection ---
  resolved_clicks = 0;  // Clear each cycle; only non-zero on the cycle a sequence resolves
  
  // Detect rising edge (button just pressed)
  if (switch_pressed && !prev_switch_pressed) {
    click_count++;
    last_click_time = millis();
  }
  prev_switch_pressed = switch_pressed;
  
  // Resolve the click sequence once the window expires (and button is released)
  if (click_count > 0 && !switch_pressed && (millis() - last_click_time > MULTI_CLICK_WINDOW)) {
    resolved_clicks = min((uint8_t)3, click_count);  // Cap at triple-click
    click_count = 0;
  }
  
  // Check if axes are outside their deadzones
  bool x_active = (x < x_center_lower || x > x_center_upper);
  bool y_active = (y < y_center_lower || y > y_center_upper);
  
  // Joystick is in use if either axis is outside deadzone
  if (x_active || y_active) {
    in_use = true;
    last_use_time = millis();
    
    // Calculate speed multiplier from X axis
    float speed_multiplier = constrain((float)x / 4095.0, 0.05, 1.0);
    
    // Map Y axis (inverted for intuitive control)
    int base_command = map(y, 0, 4095, max_speed, min_speed);
    base_command = constrain(base_command, min_speed, max_speed);
    motor_command = (int)(base_command * speed_multiplier);
  } else {
    // Both axes in deadzone
    unsigned long now = millis();
    if (now - last_use_time > idle_timeout) {
      in_use = false;
    }
    motor_command = 0;
  }
}

void JoystickController::setDeadzone(int y_lower, int y_upper, int x_lower, int x_upper) {
  y_center_lower = y_lower;
  y_center_upper = y_upper;
  x_center_lower = x_lower;
  x_center_upper = x_upper;
}

void JoystickController::setSpeedLimits(int min, int max) {
  min_speed = min;
  max_speed = max;
}

void JoystickController::setFilterAlpha(float alpha) {
  filter_alpha = constrain(alpha, 0.0, 1.0);
}

void JoystickController::setIdleTimeout(unsigned long timeout_ms) {
  idle_timeout = timeout_ms;
}
