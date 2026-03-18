#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

class IMUHandler {
public:
  // Orientation angles (degrees)
  float yaw = 0;
  float pitch = 0;
  float roll = 0;

  // Angular velocities (degrees per second, filtered)
  float yaw_velocity = 0;
  float pitch_velocity = 0;

  // Initialize the IMU on the given I2C pins and address. Returns true on success.
  // Retries up to max_retries times. Halts execution on failure.
  bool begin(uint8_t sda_pin, uint8_t scl_pin, uint8_t i2c_address = 0x68, int max_retries = 5);

  // Read sensors and update orientation/velocity. Call every loop iteration.
  void update();

  // Whether the IMU has been successfully initialized
  bool isReady() const { return _initialized; }

private:
  Adafruit_ICM20948 _imu;
  sensors_event_t _accel, _gyro, _mag, _temp;
  bool _initialized = false;

  // Internal filter state
  float _prev_yaw = 0;
  float _prev_pitch = 0;
  float _filtered_mag_yaw = 0;

  // Complementary filter constants
  static constexpr float FILTER_ALPHA = 0.98f;
  static constexpr float YAW_VEL_FILTER_ALPHA = 0.88f;
  static constexpr float PITCH_VEL_FILTER_ALPHA = 0.75f;
  static constexpr float MAG_FILTER = 0.95f;
  static constexpr float REDUCED_ALPHA = 0.96f;
};

#endif // IMU_HANDLER_H
