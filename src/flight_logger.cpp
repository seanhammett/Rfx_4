#include "flight_logger.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>

static const char* FL_TAG = "[FlightLog]";
static const char* CSV_HEADER =
    "Date,Time,Timestamp_ms,Kite_ID,Commanded_Vel_rps,Actual_Vel_rps,"
    "Torque_Nm,Tension_N,Line_Length_m,Target_Enabled,Target_Length_m,"
    "Remote_Joy,Remote_Active,Pitch_deg,Pitch_Vel_dps,Yaw_deg,"
    "Yaw_Vel_dps,Roll_deg,Dive_Conf,AWW_Conf,AF_Conf,"
    "Kite_Pitch_deg,Kite_Roll_deg,Kite_Yaw_deg,"
    "Kite_Gyro_X,Kite_Gyro_Y,Kite_Gyro_Z,Kite_IMU_Battery\n";

// Queue depth: enough to buffer ~1s of samples at FL_LOG_RATE_HZ + commands
static const int QUEUE_DEPTH = FL_LOG_RATE_HZ * 2 + 4;

void FlightLogger::begin(uint8_t kite_id) {
    _kite_id = kite_id;

    // Configure NTP for timestamping (non-blocking, syncs in background)
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    // Create the queue and writer task — all SPIFFS I/O happens on Core 0
    _queue = xQueueCreate(QUEUE_DEPTH, sizeof(FlightLogMessage));
    xTaskCreatePinnedToCore(_writerTaskFunc, "flog_wr", 8192, this, 1, NULL, 0);

    Serial.printf("%s Initialized for kite %d (queue depth %d)\n", FL_TAG, kite_id, QUEUE_DEPTH);

    // Upload any files left from previous flights
    if (WiFi.status() == WL_CONNECTED && strlen(SUPABASE_URL) > 0) {
        uploadPending();
    }
}

// ---------- Main-loop side (Core 1, never touches SPIFFS) ----------

void FlightLogger::update(float active_flight_conf, const FlightSample& sample) {
    unsigned long now = millis();
    float dt = (_last_update_ms > 0) ? (now - _last_update_ms) / 1000.0f : 0.0f;
    if (dt > 1.0f) dt = 1.0f;
    _last_update_ms = now;

    switch (_state) {
        case IDLE:
            if (active_flight_conf >= FL_START_CONFIDENCE) {
                // Enqueue START command (non-blocking, drop if queue full)
                FlightLogMessage msg;
                msg.cmd = FL_CMD_START;
                xQueueSend(_queue, &msg, 0);
                _state = RECORDING;
                _last_log_ms = now;
            }
            break;

        case RECORDING:
            if (now - _last_log_ms >= (unsigned long)(1000 / FL_LOG_RATE_HZ)) {
                _last_log_ms = now;
                FlightLogMessage msg;
                msg.cmd = FL_CMD_SAMPLE;
                msg.sample = sample;
                xQueueSend(_queue, &msg, 0);  // drop if queue full — never block
            }
            if (active_flight_conf <= 0.0f) {
                _cooldown_elapsed_s = 0.0f;
                _state = COOLDOWN;
            }
            break;

        case COOLDOWN:
            if (now - _last_log_ms >= (unsigned long)(1000 / FL_LOG_RATE_HZ)) {
                _last_log_ms = now;
                FlightLogMessage msg;
                msg.cmd = FL_CMD_SAMPLE;
                msg.sample = sample;
                xQueueSend(_queue, &msg, 0);
            }
            if (active_flight_conf > 0.0f) {
                _state = RECORDING;
            } else {
                _cooldown_elapsed_s += dt;
                if (_cooldown_elapsed_s >= FL_STOP_DELAY_S) {
                    FlightLogMessage msg;
                    msg.cmd = FL_CMD_STOP;
                    xQueueSend(_queue, &msg, 0);
                    _state = IDLE;
                }
            }
            break;
    }
}

// ---------- Writer task (Core 0, owns all SPIFFS I/O) ----------

void FlightLogger::_writerTaskFunc(void* param) {
    FlightLogger* self = (FlightLogger*)param;
    FlightLogMessage msg;

    for (;;) {
        // Block waiting for next message — no CPU burn
        if (xQueueReceive(self->_queue, &msg, portMAX_DELAY) == pdTRUE) {
            switch (msg.cmd) {
                case FL_CMD_START:   self->_handleStart();             break;
                case FL_CMD_SAMPLE:  self->_handleSample(msg.sample); break;
                case FL_CMD_STOP:    self->_handleStop();              break;
                case FL_CMD_UPLOAD:  self->_uploadAllFiles();          break;
            }
        }
    }
}

void FlightLogger::_handleStart() {
    uint32_t counter = _nextFileCounter();
    snprintf(_filepath, sizeof(_filepath), "/f%04lu.csv", (unsigned long)counter);

    _file = SPIFFS.open(_filepath, FILE_WRITE);
    if (!_file) {
        Serial.printf("%s ERROR: Cannot open %s\n", FL_TAG, _filepath);
        return;
    }

    _buffer_pos = 0;
    _file_bytes = 0;
    _sample_count = 0;
    _recording_start_ms = millis();

    // Write CSV header into buffer
    size_t hlen = strlen(CSV_HEADER);
    memcpy(_buffer, CSV_HEADER, hlen);
    _buffer_pos = hlen;

    Serial.printf("%s Recording started -> %s\n", FL_TAG, _filepath);
}

void FlightLogger::_handleStop() {
    _flushBuffer();
    if (_file) _file.close();
    Serial.printf("%s Recording stopped: %s (%lu samples, %lu bytes)\n",
                  FL_TAG, _filepath, (unsigned long)_sample_count, (unsigned long)_file_bytes);
    uploadPending();
}

void FlightLogger::_handleSample(const FlightSample& sample) {
    if (!_file) return;

    uint32_t ts = millis() - _recording_start_ms;

    char date_str[12] = "";
    char time_str[10] = "";
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {
        strftime(date_str, sizeof(date_str), "%Y-%m-%d", &timeinfo);
        strftime(time_str, sizeof(time_str), "%H:%M:%S", &timeinfo);
    }

    char row[384];
    int len = snprintf(row, sizeof(row),
        "%s,%s,%lu,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u\n",
        date_str, time_str, (unsigned long)ts, _kite_id,
        sample.commanded_vel_rps, sample.actual_vel_rps,
        sample.torque_nm, sample.tension_n, sample.line_length_m,
        sample.target_enabled ? 1 : 0, sample.target_length_m,
        (int)sample.remote_joy, sample.remote_active ? 1 : 0,
        sample.pitch_deg, sample.pitch_vel_dps,
        sample.yaw_deg, sample.yaw_vel_dps, sample.roll_deg,
        sample.dive_conf, sample.aww_conf, sample.af_conf,
        sample.kite_pitch_deg, sample.kite_roll_deg, sample.kite_yaw_deg,
        sample.kite_gyro_x, sample.kite_gyro_y, sample.kite_gyro_z,
        (unsigned)sample.kite_imu_battery);

    if (len <= 0 || len >= (int)sizeof(row)) return;

    if (_buffer_pos + len > FL_BUFFER_SIZE) {
        _flushBuffer();
    }

    memcpy(_buffer + _buffer_pos, row, len);
    _buffer_pos += len;
    _sample_count++;
}

void FlightLogger::_flushBuffer() {
    if (_buffer_pos == 0 || !_file) return;
    size_t written = _file.write((uint8_t*)_buffer, _buffer_pos);
    _file_bytes += written;
    _buffer_pos = 0;
}

// ---------- Upload (also runs on Core 0 via queue or dedicated task) ----------

void FlightLogger::uploadPending() {
    if (_upload_in_progress) return;
    if (strlen(SUPABASE_URL) == 0 || strlen(SUPABASE_ANON_KEY) == 0) {
        Serial.printf("%s Supabase not configured - files kept on SPIFFS\n", FL_TAG);
        return;
    }
    if (WiFi.status() != WL_CONNECTED) {
        Serial.printf("%s No WiFi - upload deferred\n", FL_TAG);
        return;
    }

    // Send upload command through queue so it runs on the writer task (Core 0)
    if (_queue) {
        _upload_in_progress = true;
        FlightLogMessage msg;
        msg.cmd = FL_CMD_UPLOAD;
        xQueueSend(_queue, &msg, 0);
    }
}

void FlightLogger::_uploadTaskFunc(void* param) {
    // Kept for legacy compatibility — not currently used
    FlightLogger* self = (FlightLogger*)param;
    self->_uploadAllFiles();
    self->_upload_in_progress = false;
    vTaskDelete(NULL);
}

void FlightLogger::_uploadAllFiles() {
    File root = SPIFFS.open("/");
    if (!root) { _upload_in_progress = false; return; }

    String files[32];
    int count = 0;
    File f = root.openNextFile();
    while (f && count < 32) {
        String name = f.name();
        f.close();
        if (name.endsWith(".csv") && (name.startsWith("/f") || name.startsWith("f"))) {
            // Ensure leading slash — some ESP32 frameworks omit it
            if (!name.startsWith("/")) name = "/" + name;
            files[count++] = name;
        }
        f = root.openNextFile();
    }

    for (int i = 0; i < count; i++) {
        if (_uploadFile(files[i].c_str())) {
            SPIFFS.remove(files[i]);
            Serial.printf("%s Uploaded & deleted %s\n", FL_TAG, files[i].c_str());
        } else {
            Serial.printf("%s Upload failed: %s (kept on SPIFFS)\n", FL_TAG, files[i].c_str());
        }
    }
    _upload_in_progress = false;
}

bool FlightLogger::_uploadFile(const char* path) {
    File f = SPIFFS.open(path, FILE_READ);
    if (!f) return false;

    size_t fileSize = f.size();

    char remoteName[64];
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {
        char ts[20];
        strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &timeinfo);
        snprintf(remoteName, sizeof(remoteName), "kite_%u_%s.csv", _kite_id, ts);
    } else {
        snprintf(remoteName, sizeof(remoteName), "kite_%u_%lu.csv", _kite_id, millis());
    }

    String url = String(SUPABASE_URL) + "/storage/v1/object/" + SUPABASE_BUCKET + "/" + remoteName;

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;
    if (!http.begin(client, url)) {
        f.close();
        return false;
    }

    http.addHeader("apikey", SUPABASE_ANON_KEY);
    http.addHeader("Authorization", String("Bearer ") + SUPABASE_ANON_KEY);
    http.addHeader("Content-Type", "text/csv");
    http.addHeader("x-upsert", "true");

    int httpCode = http.sendRequest("POST", &f, fileSize);
    f.close();

    if (httpCode >= 200 && httpCode < 300) {
        http.end();
        return true;
    }
    String body = http.getString();
    Serial.printf("%s HTTP %d uploading %s: %s\n", FL_TAG, httpCode, path, body.c_str());
    http.end();
    return false;
}

uint32_t FlightLogger::_nextFileCounter() {
    uint32_t maxNum = 0;
    File root = SPIFFS.open("/");
    if (!root) return 0;
    File f = root.openNextFile();
    while (f) {
        const char* name = f.name();
        f.close();
        const char* p = name;
        if (*p == '/') p++;
        if (*p == 'f' && strstr(name, ".csv")) {
            uint32_t num = atoi(p + 1);
            if (num >= maxNum) maxNum = num + 1;
        }
        f = root.openNextFile();
    }
    return maxNum;
}
