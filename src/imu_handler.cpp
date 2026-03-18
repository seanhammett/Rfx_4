#include "imu_handler.h"

bool IMUHandler::begin(uint8_t sda_pin, uint8_t scl_pin, uint8_t i2c_address, int max_retries) {
  Wire.begin(sda_pin, scl_pin);
  delay(100);

  Serial.printf("Initializing ICM-20948 IMU at 0x%02X...\n", i2c_address);
  const int RETRY_DELAY = 500;  // ms
  bool connected = false;

  for (int attempt = 1; attempt <= max_retries; attempt++) {
    if (_imu.begin_I2C(i2c_address, &Wire)) {
      Serial.printf("✓ IMU connected on attempt %d\n", attempt);
      connected = true;
      break;
    }
    if (attempt < max_retries) {
      Serial.printf("⚠ IMU connection failed, retrying... (%d/%d)\n", attempt, max_retries);
      delay(RETRY_DELAY);
    }
  }

  if (!connected) {
    Serial.println("✗ IMU connection FAILED after all attempts!");
    Serial.println("Check I2C wiring and IMU power.");
    _initialized = false;
    return false;
  }

  _imu.setGyroRateDivisor(1);
  _imu.setAccelRateDivisor(1);
  _initialized = true;
  Serial.println("✓ IMU initialization complete");
  return true;
}

void IMUHandler::update() {
  if (!_initialized) return;

  if (_imu.getEvent(&_accel, &_gyro, &_temp, &_mag)) {
    float accel_roll = atan2(_accel.acceleration.y, _accel.acceleration.z) * 180.0 / M_PI;
    float accel_pitch = atan2(-_accel.acceleration.x,
                              sqrt(_accel.acceleration.y * _accel.acceleration.y +
                                   _accel.acceleration.z * _accel.acceleration.z)) * 180.0 / M_PI;

    float gx_dps = _gyro.gyro.x * 180.0 / M_PI;
    float gy_dps = _gyro.gyro.y * 180.0 / M_PI;

    static unsigned long lastTime = millis();
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Prevent division by zero and skip first reading
    if (dt < 0.001f) dt = 0.001f;

    roll = FILTER_ALPHA * (roll + gx_dps * dt) + (1.0 - FILTER_ALPHA) * accel_roll;
    pitch = FILTER_ALPHA * (pitch + gy_dps * dt) + (1.0 - FILTER_ALPHA) * accel_pitch;

    float mx = _mag.magnetic.x;
    float my = _mag.magnetic.y;
    float mag_yaw = atan2(my, mx) * 180.0 / M_PI;
    float gz_dps = _gyro.gyro.z * 180.0 / M_PI;

    // Smooth magnetometer readings with low-pass filter (reduce jitter)
    _filtered_mag_yaw = MAG_FILTER * _filtered_mag_yaw + (1.0 - MAG_FILTER) * mag_yaw;

    // Complementary filter - trust gyro more to reduce noise
    yaw = REDUCED_ALPHA * (yaw + gz_dps * dt) + (1.0 - REDUCED_ALPHA) * _filtered_mag_yaw;

    // Calculate angular velocities with rolling average
    static bool firstRead = true;
    if (!firstRead) {
      float raw_yaw_vel = (yaw - _prev_yaw) / dt;
      float raw_pitch_vel = (pitch - _prev_pitch) / dt;

      yaw_velocity = YAW_VEL_FILTER_ALPHA * yaw_velocity + (1.0 - YAW_VEL_FILTER_ALPHA) * raw_yaw_vel;
      pitch_velocity = PITCH_VEL_FILTER_ALPHA * pitch_velocity + (1.0 - PITCH_VEL_FILTER_ALPHA) * raw_pitch_vel;
    } else {
      firstRead = false;
    }

    _prev_yaw = yaw;
    _prev_pitch = pitch;
  }
}
