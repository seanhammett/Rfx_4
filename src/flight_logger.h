#ifndef FLIGHT_LOGGER_H
#define FLIGHT_LOGGER_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include "flight_logger_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

struct FlightSample {
    float commanded_vel_rps;
    float actual_vel_rps;
    float torque_nm;
    float tension_n;
    float line_length_m;
    bool target_enabled;
    float target_length_m;
    int16_t remote_joy;
    bool remote_active;
    float pitch_deg;
    float pitch_vel_dps;
    float yaw_deg;
    float yaw_vel_dps;
    float roll_deg;
    float dive_conf;
    float aww_conf;
    float af_conf;
    // Kite-mounted IMU (CodeCell BNO085, received via ESP-NOW)
    float kite_pitch_deg;
    float kite_roll_deg;
    float kite_yaw_deg;
    float kite_gyro_x;
    float kite_gyro_y;
    float kite_gyro_z;
    uint8_t kite_imu_battery;
};

// Commands sent from main loop to writer task via queue
enum FlightLogCmd : uint8_t { FL_CMD_START, FL_CMD_SAMPLE, FL_CMD_STOP, FL_CMD_UPLOAD };

struct FlightLogMessage {
    FlightLogCmd cmd;
    FlightSample sample;  // only used for FL_CMD_SAMPLE
};

class FlightLogger {
public:
    void begin(uint8_t kite_id);

    // Call every control cycle — lightweight state machine, enqueues to writer task
    void update(float active_flight_conf, const FlightSample& sample);

    bool isRecording() const { return _state == RECORDING || _state == COOLDOWN; }
    bool isUploading() const { return _upload_in_progress; }
    uint32_t getSampleCount() const { return _sample_count; }
    uint32_t getFileBytes() const { return _file_bytes; }

    // Trigger upload of all pending flight log files (non-blocking, spawns task)
    void uploadPending();

private:
    enum State { IDLE, RECORDING, COOLDOWN };
    State _state = IDLE;

    uint8_t _kite_id = 0;
    char _filepath[32];

    // Timing
    unsigned long _last_log_ms = 0;
    unsigned long _last_update_ms = 0;
    float _cooldown_elapsed_s = 0.0f;

    // Stats (updated by writer task, read by main loop — atomic-safe for single-word reads)
    volatile uint32_t _file_bytes = 0;
    volatile uint32_t _sample_count = 0;

    // Writer task queue — main loop enqueues, writer task dequeues & does all SPIFFS I/O
    QueueHandle_t _queue = nullptr;
    static void _writerTaskFunc(void* param);

    // --- Writer task internals (only touched by writer task on Core 0) ---
    File _file;
    char _buffer[FL_BUFFER_SIZE];
    size_t _buffer_pos = 0;
    unsigned long _recording_start_ms = 0;

    void _handleStart();
    void _handleSample(const FlightSample& sample);
    void _handleStop();
    void _flushBuffer();

    // Upload
    volatile bool _upload_in_progress = false;
    void _uploadAllFiles();
    bool _uploadFile(const char* path);
    static void _uploadTaskFunc(void* param);

    uint32_t _nextFileCounter();
};

#endif // FLIGHT_LOGGER_H
