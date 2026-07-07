// Remote Control Joystick Calibration
// Reads all configured joysticks (X and Y) for 10 seconds and reports
// the average "zero" center value for each axis.
// LED: 5 rapid red blinks, then solid white during data gathering.

#include <Arduino.h>

// Set to true if remote_control.cpp uses setInvert(true, true)
// so that the reported deadband values are in the same (inverted) space.
#define INVERT_AXES  true

// Number of joysticks physically present on this controller (1..4).
// Keep in sync with NUM_JOYSTICKS in remote_control.cpp.
#define NUM_JOYSTICKS 3
static_assert(NUM_JOYSTICKS >= 1 && NUM_JOYSTICKS <= 4,
              "NUM_JOYSTICKS must be between 1 and 4");

// =================== JOYSTICK PINS ===================
// Joystick 1 (option 1)
#define JOY1_X_PIN  6
#define JOY1_Y_PIN  7
// Joystick 2 (option 2)
#define JOY2_X_PIN  4
#define JOY2_Y_PIN  5
// Joystick 3 (option 3)
#define JOY3_X_PIN  12
#define JOY3_Y_PIN  10
// Joystick 4 (option 4)
#define JOY4_X_PIN  13
#define JOY4_Y_PIN  11

// Full pin/label tables for all 4 possible joysticks;
// only the first NUM_JOYSTICKS are read.
static const uint8_t JOY_PINS[] = {
  JOY1_X_PIN, JOY1_Y_PIN,
  JOY2_X_PIN, JOY2_Y_PIN,
  JOY3_X_PIN, JOY3_Y_PIN,
  JOY4_X_PIN, JOY4_Y_PIN
};
static const char* JOY_LABELS[] = {
  "Joy1_X", "Joy1_Y",
  "Joy2_X", "Joy2_Y",
  "Joy3_X", "Joy3_Y",
  "Joy4_X", "Joy4_Y"
};
static const int NUM_AXES = NUM_JOYSTICKS * 2;

static const unsigned long GATHER_DURATION_MS = 10000;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== Joystick Calibration ===");
  Serial.println("Leave all joysticks centred / at rest.\n");

  // Configure ADC pins
  for (int i = 0; i < NUM_AXES; i++) {
    pinMode(JOY_PINS[i], INPUT);
    analogSetPinAttenuation(JOY_PINS[i], ADC_11db);
  }

  // --- 5 rapid red blinks ---
  neopixelWrite(2, 0, 0, 0);
  for (int i = 0; i < 5; i++) {
    neopixelWrite(2, 255, 0, 0);
    delay(150);
    neopixelWrite(2, 0, 0, 0);
    delay(150);
  }

  // --- Solid white while gathering ---
  neopixelWrite(2, 255, 255, 255);
  Serial.println("Gathering data for 10 seconds...\n");

  long sums[NUM_AXES] = {0};
  int mins[NUM_AXES];
  int maxs[NUM_AXES];
  unsigned long sampleCount = 0;

  // Initialise min/max with first reading
  for (int i = 0; i < NUM_AXES; i++) {
    int v = analogRead(JOY_PINS[i]);
    if (INVERT_AXES) v = 4095 - v;
    mins[i] = v;
    maxs[i] = v;
    sums[i] = v;
  }
  sampleCount = 1;

  for (int j = 0; j < NUM_JOYSTICKS; j++) {
    Serial.printf("J%d_X  J%d_Y  ", j + 1, j + 1);
  }
  Serial.println(" t(s)");
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print("----  ");
  }
  Serial.println(" ----");

  unsigned long startTime = millis();
  unsigned long lastPrint = startTime;
  while (millis() - startTime < GATHER_DURATION_MS) {
    unsigned long now = millis();
    int current[NUM_AXES];
    for (int i = 0; i < NUM_AXES; i++) {
      current[i] = analogRead(JOY_PINS[i]);
      if (INVERT_AXES) current[i] = 4095 - current[i];
      sums[i] += current[i];
      if (current[i] < mins[i]) mins[i] = current[i];
      if (current[i] > maxs[i]) maxs[i] = current[i];
    }
    sampleCount++;

    // Print a row ~100 times per second
    if (now - lastPrint >= 10) {
      lastPrint = now;
      for (int i = 0; i < NUM_AXES; i++) {
        Serial.printf("%4d  ", current[i]);
      }
      Serial.printf("  %.2f\n", (now - startTime) / 1000.0);
    }

    delay(5);  // ~200 Hz sample rate
  }

  // --- Done – LED off ---
  neopixelWrite(2, 0, 0, 0);

  // --- Print results ---
  Serial.printf("Samples collected: %lu\n\n", sampleCount);
  Serial.println("Axis       Avg     Min     Max");
  Serial.println("---------- ------- ------- -------");
  int avgs[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) {
    avgs[i] = (int)(sums[i] / (long)sampleCount);
    Serial.printf("%-10s %5d   %5d   %5d\n", JOY_LABELS[i], avgs[i], mins[i], maxs[i]);
  }

  // --- Suggested deadband per joystick (paired X/Y axes) ---
  // The deadband is the min/max observed range plus a 15% margin,
  // suitable for use with setDeadzone(y_lower, y_upper, x_lower, x_upper).
  Serial.println("\n--- Suggested Deadband (with 15%% margin) ---");
  Serial.println("Joystick   X_lower  X_upper  Y_lower  Y_upper   setDeadzone() call");
  Serial.println("---------- -------- -------- -------- --------  --------------------");
  for (int j = 0; j < NUM_JOYSTICKS; j++) {
    int xi = j * 2;      // X axis index
    int yi = j * 2 + 1;  // Y axis index

    int x_range = maxs[xi] - mins[xi];
    int y_range = maxs[yi] - mins[yi];
    int x_margin = max(x_range / 8, 32);  // at least 25 counts
    int y_margin = max(y_range / 8, 32);

    int x_lower = mins[xi] - x_margin;
    int x_upper = maxs[xi] + x_margin;
    int y_lower = mins[yi] - y_margin;
    int y_upper = maxs[yi] + y_margin;

    Serial.printf("Joy%d       %5d    %5d    %5d    %5d     setDeadzone(%d, %d, %d, %d)\n",
                  j + 1, x_lower, x_upper, y_lower, y_upper,
                  y_lower, y_upper, x_lower, x_upper);
  }

  Serial.println("\nUse the Avg values as the joystick centre (zero) references.");
  Serial.println("Calibration complete.");
}

void loop() {
  // Nothing to do – calibration runs once in setup().
}
