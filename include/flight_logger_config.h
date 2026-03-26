#ifndef FLIGHT_LOGGER_CONFIG_H
#define FLIGHT_LOGGER_CONFIG_H

// ===== Supabase Storage Configuration =====
// Fill in your Supabase project details.
// Leave SUPABASE_URL empty to disable cloud upload (files stay on SPIFFS).
#define SUPABASE_URL       "https://iigtcugucufmrosrkgjv.supabase.co"   // e.g. "https://abcdefg.supabase.co"
#define SUPABASE_ANON_KEY  "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImlpZ3RjdWd1Y3VmbXJvc3JrZ2p2Iiwicm9sZSI6ImFub24iLCJpYXQiOjE3NzQ1MTc1OTgsImV4cCI6MjA5MDA5MzU5OH0.uAce-cVvKKpfZcImL2xKzsVZlm3mAnlshcMAgYaSV18"   // Your project's anon/public key
#define SUPABASE_BUCKET    "flight-logs"

// ===== Flight Recording Parameters =====
#define FL_START_CONFIDENCE  0.5f   // Start recording when AF confidence exceeds this
#define FL_STOP_DELAY_S      10.0f  // Seconds at zero confidence before stopping
#define FL_LOG_RATE_HZ       30     // Samples per second (30 = one row every ~33ms)
#define FL_BUFFER_SIZE       4096   // RAM buffer size — flushes to SPIFFS when full

#endif // FLIGHT_LOGGER_CONFIG_H
