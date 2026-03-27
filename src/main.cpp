/*
 * ESP32 Orientation System
 * ========================
 * MPU6050 (Accel + Gyro) + QMC5883L (Magnetometer) + SSD1306 OLED
 *
 * - Roll & Pitch from Madgwick AHRS filter (accel + gyro fusion)
 * - Yaw from tilt-compensated magnetometer (hard-iron corrected)
 * - OLED displays all three angles at 10 Hz
 * - Sensor loop runs at ~100 Hz, fully non-blocking
 */

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <QMC5883LCompass.h>
#include <MadgwickAHRS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Inclinometer_inferencing.h>

// ============================================================
// TUNABLE PARAMETERS — adjust these as needed
// ============================================================
#define MADGWICK_BETA           0.1f    // Madgwick filter gain (higher = faster convergence, more noise)
#define MAGNETIC_DECLINATION    0.5f    // Magnetic declination for Karnal, Haryana, India (degrees)
#define MAG_OFFSET_X            0       // Hard-iron calibration offset X
#define MAG_OFFSET_Y            0       // Hard-iron calibration offset Y
#define MAG_OFFSET_Z            0       // Hard-iron calibration offset Z
#define I2C_SDA                 21      // ESP32 I2C SDA pin
#define I2C_SCL                 22      // ESP32 I2C SCL pin
#define OLED_UPDATE_INTERVAL_MS 100     // OLED refresh interval (100ms = 10 Hz)
#define SENSOR_INTERVAL_MS      10      // Sensor read interval (10ms = 100 Hz)
#define BUTTON_PIN              15      // GPIO15 — zero calibration button (active LOW)
#define MODE_BUTTON_PIN         19      // GPIO19 — display mode cycling button (active LOW)
#define DEBOUNCE_MS             50      // Button debounce time (ms)
#define MODE_DEBOUNCE_MS        50      // Mode button debounce time (ms)
#define ZERO_MSG_DURATION_MS    1000    // "ZERO SET!" OLED message display time (ms)
#define CONFIDENCE_THRESHOLD    0.7f    // Minimum confidence to accept a gesture
#define EI_SAMPLE_INTERVAL_MS   16      // ~60 Hz sampling for Edge Impulse (matches model training)
#define TOTAL_MODES             4       // Number of display modes (Normal, Z, X, Y)
#define OUTER_RING_RADIUS       27      // Bubble level outer circle radius (px)
#define BUBBLE_RADIUS           4       // Bubble dot radius (px)
#define BUBBLE_CENTER_X         64      // Circle center X on 128-wide screen
#define BUBBLE_CENTER_Y         32      // Circle center Y (perfect vertical center)
#define MAX_TILT_ANGLE          45.0f   // Angle (°) at which bubble reaches outer ring edge

// ============================================================
// OLED Configuration
// ============================================================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1    // No hardware reset pin
#define OLED_ADDR     0x3C  // I2C address for SSD1306

// ============================================================
// Sensor Scaling Constants
// ============================================================
#define ACCEL_SCALE   16384.0f  // LSB/g for ±2g range
#define GYRO_SCALE    131.0f    // LSB/(°/s) for ±250°/s range
#define DEG_TO_RAD_F  (PI / 180.0f)
#define RAD_TO_DEG_F  (180.0f / PI)

// ============================================================
// Global Objects
// ============================================================
MPU6050 mpu;
QMC5883LCompass compass;
Madgwick madgwickFilter;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============================================================
// Timing Variables
// ============================================================
unsigned long prevSensorTime = 0;   // Last sensor read timestamp (ms)
unsigned long prevDisplayTime = 0;  // Last OLED update timestamp (ms)

// ============================================================
// Orientation Angles (degrees)
// ============================================================
float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

// ============================================================
// Zero Calibration Offsets
// ============================================================
float rollOffset  = 0.0f;
float pitchOffset = 0.0f;
float yawOffset   = 0.0f;

// ============================================================
// Button Debounce State
// ============================================================
int   lastButtonState      = HIGH;  // Previous raw reading (seeded in setup)
int   acceptedButtonState   = HIGH;  // Debounced stable state (seeded in setup)
unsigned long lastDebounceTime = 0;  // Timestamp of last state change

// ============================================================
// OLED Zero-Set Feedback Timing
// ============================================================
bool  showZeroMsg          = false;  // Whether to show "ZERO SET!" on OLED
unsigned long zeroMsgStartTime = 0;  // When the message was triggered

// ============================================================
// Mode Button State
// ============================================================
int   currentMode              = 0;     // 0=Normal, 1=Z-level, 2=X-level, 3=Y-level
int   lastModeButtonState      = HIGH;  // Previous raw reading
int   acceptedModeButtonState  = HIGH;  // Debounced stable state
unsigned long lastModeDebounceTime = 0; // Timestamp of last mode button state change

// ============================================================
// Edge Impulse Gesture Inference State
// ============================================================
float ei_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE]; // Buffer: 120 samples × 6 axes = 720 floats
int   ei_buffer_ix          = 0;       // Current write index into ei_buffer
bool  ei_buffer_full        = false;   // Whether buffer has filled (ready for inference)
unsigned long prevEISampleTime = 0;    // Last EI sample timestamp (ms)
String gestureLabel         = "idle";  // Detected gesture label
float  gestureConfidence    = 0.0f;    // Confidence of detected gesture

// ============================================================
// Edge Impulse: required ei_printf implementation
// ============================================================
void ei_printf(const char *format, ...) {
  static char print_buf[256] = { 0 };
  va_list args;
  va_start(args, format);
  int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);
  if (r > 0) Serial.write(print_buf);
}

// ============================================================
// Edge Impulse: callback to feed buffer data into the classifier
// ============================================================
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, ei_buffer + offset, length * sizeof(float));
  return 0;
}

// ============================================================
// Helper: Format angle string natively +123.4
// ============================================================
void formatAngle(char* buf, float angle) {
  char sign = (angle >= 0.0f) ? '+' : '-';
  float absAngle = fabs(angle);
  if (absAngle > 999.9f) absAngle = 999.9f;  // Clamp overflow
  sprintf(buf, "%c%05.1f", sign, absAngle);
}

// ============================================================
// Helper: Format angle string compactly +12.3
// ============================================================
void formatAngleCompact(char* buf, float angle) {
  char sign = (angle >= 0.0f) ? '+' : '-';
  float absAngle = fabs(angle);
  if (absAngle > 99.9f) absAngle = 99.9f;
  sprintf(buf, "%c%.1f", sign, absAngle);
}

// ============================================================
// Helper: Draw a rounded-corner rectangle (1px border radius)
// ============================================================
void drawRoundBox(int16_t x, int16_t y, int16_t w, int16_t h) {
  // Horizontal lines (inset 1px from corners)
  display.drawFastHLine(x + 1, y, w - 2, SSD1306_WHITE);
  display.drawFastHLine(x + 1, y + h - 1, w - 2, SSD1306_WHITE);
  // Vertical lines (inset 1px from corners)
  display.drawFastVLine(x, y + 1, h - 2, SSD1306_WHITE);
  display.drawFastVLine(x + w - 1, y + 1, h - 2, SSD1306_WHITE);
}

// ============================================================
// Helper: Draw a polished bubble level on the OLED
// mode: 1=Z(roll+pitch), 2=X(pitch only), 3=Y(roll only)
// ============================================================
void drawBubbleLevel(int mode, float displayRoll, float displayPitch) {
  const int cx = BUBBLE_CENTER_X;
  const int cy = BUBBLE_CENTER_Y;
  const int R  = OUTER_RING_RADIUS;
  const int r  = BUBBLE_RADIUS;
  const float maxOffset = (float)(R - r - 2); // max bubble travel from center

  // === Outer ring (double line for thickness) ===
  display.drawCircle(cx, cy, R, SSD1306_WHITE);
  display.drawCircle(cx, cy, R - 1, SSD1306_WHITE);

  // === Tick marks at cardinal points (4px long, inside ring) ===
  display.drawFastVLine(cx, cy - R + 2, 4, SSD1306_WHITE); // Top
  display.drawFastVLine(cx, cy + R - 5, 4, SSD1306_WHITE); // Bottom
  display.drawFastHLine(cx - R + 2, cy, 4, SSD1306_WHITE); // Left
  display.drawFastHLine(cx + R - 5, cy, 4, SSD1306_WHITE); // Right

  // === Center crosshair (7x7 plus sign) ===
  display.drawFastHLine(cx - 3, cy, 7, SSD1306_WHITE);
  display.drawFastVLine(cx, cy - 3, 7, SSD1306_WHITE);

  // === Gauge-style Mode Header (cuts into the top of the ring) ===
  const char* modeLabel;
  if (mode == 1) modeLabel = "Z-LEVEL";
  else if (mode == 2) modeLabel = "X-LEVEL";
  else modeLabel = "Y-LEVEL";
  
  int labelLen = strlen(modeLabel);
  int labelWidth = labelLen * 6;
  int labelX = cx - (labelWidth / 2);
  
  // Cutout background
  display.fillRect(labelX - 4, 0, labelWidth + 8, 10, SSD1306_BLACK);
  display.setCursor(labelX, 1);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print(modeLabel);

  // === Calculate bubble position ===
  float bx_offset = 0.0f;
  float by_offset = 0.0f;

  if (mode == 1) {
    // Z-level: Roll controls X, Pitch controls Y
    bx_offset = (displayRoll / MAX_TILT_ANGLE) * maxOffset;
    by_offset = (displayPitch / MAX_TILT_ANGLE) * maxOffset;
  } else if (mode == 2) {
    // X-level: Pitch controls horizontal only
    bx_offset = (displayPitch / MAX_TILT_ANGLE) * maxOffset;
    by_offset = 0.0f;
  } else {
    // Y-level: Roll controls horizontal only
    bx_offset = (displayRoll / MAX_TILT_ANGLE) * maxOffset;
    by_offset = 0.0f;
  }

  // Clamp: bubble must stay within outer ring travel limit
  float dist = sqrt(bx_offset * bx_offset + by_offset * by_offset);
  if (dist > maxOffset) {
    float scale = maxOffset / dist;
    bx_offset *= scale;
    by_offset *= scale;
  }

  int bx = cx + (int)bx_offset;
  int by = cy + (int)by_offset;

  // === Draw bubble (filled circle with a highlight) ===
  display.fillCircle(bx, by, r, SSD1306_WHITE);
  // Tiny "shine" dot for 3D effect
  display.drawPixel(bx - 1, by - 1, SSD1306_BLACK);

  // === Angle Data Display ===
  char angleBuf[12];
  display.setTextSize(1);

  if (mode == 1) {
    // Z-Level: Place cleanly on the left and right outer sides
    // Left side: Roll
    display.setCursor(0, 22);
    display.print("ROLL");
    display.setCursor(0, 33);
    formatAngleCompact(angleBuf, displayRoll);
    display.print(angleBuf);
    display.print((char)247); // Degree symbol

    // Right side: Pitch
    display.setCursor(98, 22);
    display.print("PITCH");
    formatAngleCompact(angleBuf, displayPitch);
    int textX = 127 - (strlen(angleBuf) + 1) * 6; // Right-align
    display.setCursor(textX, 33);
    display.print(angleBuf);
    display.print((char)247);
  } else {
    // Modes 2/3: 1-axis. Place actively tracked value in a bottom cutout.
    String valStr = (mode == 2 ? "PITCH: " : "ROLL: ");
    formatAngleCompact(angleBuf, mode == 2 ? displayPitch : displayRoll);
    valStr += angleBuf;
    
    int valLen = valStr.length() + 1; // +1 for degree symbol
    int valWidth = valLen * 6;
    int valX = cx - (valWidth / 2);
    
    // Bottom cutout
    display.fillRect(valX - 4, 54, valWidth + 8, 10, SSD1306_BLACK);
    display.setCursor(valX, 55);
    display.print(valStr);
    display.print((char)247);
  }

  // Place Yaw: N/A cleanly out of the way in bottom-left
  display.setCursor(0, 56);
  display.print("Y:N/A");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  // --- Serial ---
  Serial.begin(115200);
  while (!Serial) { ; }  // Wait for Serial (mainly for USB-native boards)
  Serial.println();
  Serial.println("=================================");
  Serial.println("  ESP32 Orientation System Init");
  Serial.println("=================================");

  // --- I2C Bus ---
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400 kHz Fast Mode — required for reliable SSD1306 comms
  delay(50);              // Let I2C bus settle before device init
  Serial.println("[I2C] Bus initialized (SDA=" + String(I2C_SDA) + ", SCL=" + String(I2C_SCL) + ")");

  // --- I2C Scanner (debug) ---
  Serial.println("[I2C] Scanning bus...");
  int devicesFound = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      devicesFound++;
    }
  }
  Serial.println("[I2C] " + String(devicesFound) + " device(s) found.");
  if (devicesFound == 0) {
    Serial.println("[I2C] ERROR: No devices found! Check SDA/SCL wiring.");
    while (1) { ; }
  }

  // Re-init I2C after scanner to clear bus state
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // Re-apply 400 kHz after bus re-init
  delay(100);

  // --- MPU6050 Initialization ---
  Serial.print("[MPU6050] Initializing... ");
  mpu.initialize();
  delay(100);  // Give MPU6050 time to wake up

  if (mpu.testConnection()) {
    Serial.println("OK");
  } else {
    // WHO_AM_I mismatch — common with clone chips, but device is on the bus
    Serial.println("WARNING: testConnection failed (clone chip?), continuing anyway.");
  }

  // --- QMC5883L Initialization ---
  Serial.print("[QMC5883L] Initializing... ");
  compass.init();
  // Set magnetometer mode, data rate, range, and oversampling
  compass.setMode(0x01, 0x0C, 0x10, 0x00);  // Continuous, 200Hz ODR, 8G range, 512 OSR
  delay(100);
  // Quick verification: read and check if we get non-zero data
  compass.read();
  int magTestX = compass.getX();
  int magTestY = compass.getY();
  int magTestZ = compass.getZ();
  Serial.print("Mag raw: X="); Serial.print(magTestX);
  Serial.print(" Y="); Serial.print(magTestY);
  Serial.print(" Z="); Serial.println(magTestZ);
  if (magTestX == 0 && magTestY == 0 && magTestZ == 0) {
    Serial.println("  WARNING — all zeros (device may be at 0x2C instead of 0x0D)");
  } else {
    Serial.println("OK");
  }

  // --- OLED Initialization ---
  Serial.print("[OLED] Initializing SSD1306 at 0x3C... ");
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("FAILED! Check wiring.");
    while (1) { ; }  // Halt — display is required
  }
  Serial.println("OK");

  // Show a brief splash screen
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Outer border
  drawRoundBox(2, 2, 124, 60);

  // Title
  display.setTextSize(2);
  display.setCursor(16, 8);
  display.print("INCLINE");

  // Thin separator
  display.drawFastHLine(10, 27, 108, SSD1306_WHITE);

  // Subtitle
  display.setTextSize(1);
  display.setCursor(16, 33);
  display.print("Orientation System");

  // Version / loading indicator
  display.setCursor(34, 50);
  display.print("Loading...");

  display.display();
  delay(1000);  // Only delay in setup — splash screen

  // --- Zero Calibration Button ---
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(10);  // Let pull-up settle before first read
  lastButtonState    = digitalRead(BUTTON_PIN);  // Seed with actual pin state
  acceptedButtonState = lastButtonState;          // No spurious trigger on first loop
  Serial.println("[BUTTON] GPIO" + String(BUTTON_PIN) + " configured (INPUT_PULLUP, initial=" + String(lastButtonState) + ")");

  // --- Mode Button ---
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  delay(10);
  lastModeButtonState    = digitalRead(MODE_BUTTON_PIN);
  acceptedModeButtonState = lastModeButtonState;
  Serial.println("[MODE]   GPIO" + String(MODE_BUTTON_PIN) + " configured (INPUT_PULLUP, initial=" + String(lastModeButtonState) + ")");

  // --- Madgwick Filter ---
  // Begin with the target sensor sample rate (100 Hz)
  madgwickFilter.begin(100);

  // Record starting times
  prevSensorTime  = millis();
  prevDisplayTime = millis();
  prevEISampleTime = millis();

  Serial.println();
  Serial.println("[EI] Model: " + String(EI_CLASSIFIER_PROJECT_NAME));
  Serial.println("[EI] Labels: " + String(EI_CLASSIFIER_LABEL_COUNT) + ", Window: " + String(EI_CLASSIFIER_RAW_SAMPLE_COUNT) + " samples @ " + String(EI_CLASSIFIER_FREQUENCY) + "Hz");
  Serial.println();
  Serial.println("All sensors initialized. Entering main loop.");
  Serial.println("-------------------------------------------------");
}

// ============================================================
// MAIN LOOP — fully non-blocking
// ============================================================
void loop() {
  unsigned long now = millis();

  // ===========================================================
  // SENSOR UPDATE — runs at ~100 Hz (every 10ms)
  // ===========================================================
  if (now - prevSensorTime >= SENSOR_INTERVAL_MS) {
    // Calculate accurate dt in seconds
    float dt = (now - prevSensorTime) / 1000.0f;
    prevSensorTime = now;

    // ---------------------------------------------------------
    // 1. Read MPU6050 raw accelerometer and gyroscope data
    // ---------------------------------------------------------
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // Convert accelerometer to g's (±2g range: 16384 LSB/g)
    float accelX = ax_raw / ACCEL_SCALE;
    float accelY = ay_raw / ACCEL_SCALE;
    float accelZ = az_raw / ACCEL_SCALE;

    // Convert gyroscope to °/s (±250°/s range: 131 LSB/°/s)
    float gyroX = gx_raw / GYRO_SCALE;
    float gyroY = gy_raw / GYRO_SCALE;
    float gyroZ = gz_raw / GYRO_SCALE;

    // ---------------------------------------------------------
    // 2. Feed accelerometer + gyroscope into Madgwick filter
    //    Using updateIMU (6-axis) — Roll & Pitch come from here
    // ---------------------------------------------------------
    madgwickFilter.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

    // Extract Roll and Pitch from the Madgwick filter output
    roll  = madgwickFilter.getRoll();
    pitch = madgwickFilter.getPitch();

    // ---------------------------------------------------------
    // 3. Read QMC5883L magnetometer
    // ---------------------------------------------------------
    compass.read();
    float magX = (float)compass.getX() - MAG_OFFSET_X;
    float magY = (float)compass.getY() - MAG_OFFSET_Y;
    float magZ = (float)compass.getZ() - MAG_OFFSET_Z;

    // ---------------------------------------------------------
    // 4. Tilt-compensate magnetometer using Roll & Pitch
    //    This is CRITICAL — never compute heading from raw mag values
    // ---------------------------------------------------------
    float rollRad  = roll  * DEG_TO_RAD_F;
    float pitchRad = pitch * DEG_TO_RAD_F;

    float cosRoll  = cos(rollRad);
    float sinRoll  = sin(rollRad);
    float cosPitch = cos(pitchRad);
    float sinPitch = sin(pitchRad);

    // Tilt-compensated magnetic field components in the horizontal plane
    float magXcomp = magX * cosPitch + magY * sinRoll * sinPitch + magZ * cosRoll * sinPitch;
    float magYcomp = magY * cosRoll  - magZ * sinRoll;

    // ---------------------------------------------------------
    // 5. Calculate Yaw (heading) from tilt-compensated values
    // ---------------------------------------------------------
    float heading = atan2(-magYcomp, magXcomp) * RAD_TO_DEG_F;

    // Apply magnetic declination correction (Karnal, Haryana, India)
    heading += MAGNETIC_DECLINATION;

    // Normalize heading to 0–360° range
    if (heading < 0)    heading += 360.0f;
    if (heading >= 360) heading -= 360.0f;

    yaw = heading;

    // ---------------------------------------------------------
    // 6. Edge Impulse: sample into inference buffer at ~60 Hz
    // ---------------------------------------------------------
    if (now - prevEISampleTime >= EI_SAMPLE_INTERVAL_MS) {
      prevEISampleTime = now;

      // Push 6 axis values into the buffer
      ei_buffer[ei_buffer_ix + 0] = accelX;
      ei_buffer[ei_buffer_ix + 1] = accelY;
      ei_buffer[ei_buffer_ix + 2] = accelZ;
      ei_buffer[ei_buffer_ix + 3] = gyroX;
      ei_buffer[ei_buffer_ix + 4] = gyroY;
      ei_buffer[ei_buffer_ix + 5] = gyroZ;
      ei_buffer_ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME; // += 6

      // Buffer is full — run inference
      if (ei_buffer_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ei_buffer_ix = 0;

        signal_t signal;
        signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        signal.get_data = &raw_feature_get_data;

        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

        if (err == EI_IMPULSE_OK) {
          // Find the label with highest confidence
          float maxVal = 0.0f;
          int   maxIdx = -1;
          for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            if (result.classification[i].value > maxVal) {
              maxVal = result.classification[i].value;
              maxIdx = i;
            }
          }
          gestureConfidence = maxVal;
          if (maxVal >= CONFIDENCE_THRESHOLD && maxIdx >= 0) {
            gestureLabel = String(result.classification[maxIdx].label);
          } else {
            gestureLabel = "idle";
          }
        }
      }
    }

    // ---------------------------------------------------------
    // 7. Serial output for debugging (offset-corrected values)
    // ---------------------------------------------------------
    float displayRoll  = roll  - rollOffset;
    float displayPitch = pitch - pitchOffset;
    float displayYaw   = yaw   - yawOffset;

    char rollStr[12], pitchStr[12], yawStr[12];
    formatAngle(rollStr, displayRoll);
    formatAngle(pitchStr, displayPitch);
    formatAngle(yawStr, displayYaw);

    Serial.print("Roll: ");
    Serial.print(rollStr);
    Serial.print("° | Pitch: ");
    Serial.print(pitchStr);
    Serial.print("° | Yaw: ");
    Serial.print(yawStr);
    Serial.print("° | Gesture: ");
    Serial.print(gestureLabel);
    Serial.print(" (");
    Serial.print(gestureConfidence, 2);
    Serial.println(")");
  }

  // ===========================================================
  // BUTTON DEBOUNCE + ZERO CALIBRATION — non-blocking
  // ===========================================================
  int currentReading = digitalRead(BUTTON_PIN);

  if (currentReading != lastButtonState) {
    lastDebounceTime = now;  // Reset debounce timer on any state change
  }

  if ((now - lastDebounceTime) >= DEBOUNCE_MS) {
    // Reading has been stable for DEBOUNCE_MS — accept it
    if (currentReading != acceptedButtonState) {
      acceptedButtonState = currentReading;

      // Trigger on press (HIGH → LOW transition)
      if (acceptedButtonState == LOW) {
        // Capture current raw angles as new zero reference
        rollOffset  = roll;
        pitchOffset = pitch;
        yawOffset   = yaw;

        // Serial feedback
        char rBuf[12], pBuf[12], yBuf[12];
        formatAngle(rBuf, roll  - rollOffset);
        formatAngle(pBuf, pitch - pitchOffset);
        formatAngle(yBuf, yaw   - yawOffset);
        Serial.print("[CALIBRATION] Zero reference set — Roll: ");
        Serial.print(rBuf); Serial.print("° | Pitch: ");
        Serial.print(pBuf); Serial.print("° | Yaw: ");
        Serial.print(yBuf); Serial.println("°");

        // Trigger OLED feedback
        showZeroMsg = true;
        zeroMsgStartTime = now;
      }
    }
  }

  lastButtonState = currentReading;

  // ===========================================================
  // MODE BUTTON DEBOUNCE — non-blocking, cycles display modes
  // ===========================================================
  int modeReading = digitalRead(MODE_BUTTON_PIN);

  if (modeReading != lastModeButtonState) {
    lastModeDebounceTime = now;
  }

  if ((now - lastModeDebounceTime) >= MODE_DEBOUNCE_MS) {
    if (modeReading != acceptedModeButtonState) {
      acceptedModeButtonState = modeReading;

      if (acceptedModeButtonState == LOW) {
        currentMode = (currentMode + 1) % TOTAL_MODES;
        Serial.println("[MODE] Switched to mode " + String(currentMode));
      }
    }
  }

  lastModeButtonState = modeReading;

  // Auto-clear "ZERO SET!" message after duration expires
  if (showZeroMsg && (now - zeroMsgStartTime >= ZERO_MSG_DURATION_MS)) {
    showZeroMsg = false;
  }

  // ===========================================================
  // OLED DISPLAY UPDATE — runs at ~10 Hz (every 100ms)
  // Independent of sensor loop
  // ===========================================================
  if (now - prevDisplayTime >= OLED_UPDATE_INTERVAL_MS) {
    prevDisplayTime = now;

    display.clearDisplay();

    if (showZeroMsg) {
      // ═══════════════════════════════════════════════════════
      // "ZERO SET!" Feedback — checkmark + bordered layout
      // ═══════════════════════════════════════════════════════
      drawRoundBox(4, 2, 120, 60);

      // Large checkmark icon (drawn with lines)
      display.drawLine(40, 30, 52, 42, SSD1306_WHITE);  // short leg
      display.drawLine(52, 42, 76, 18, SSD1306_WHITE);  // long leg
      display.drawLine(40, 31, 52, 43, SSD1306_WHITE);  // thicken
      display.drawLine(52, 43, 76, 19, SSD1306_WHITE);  // thicken

      // "ZERO SET" text below checkmark
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(32, 50);
      display.print("ZERO  SET!");

    } else if (currentMode == 0) {
      // ═══════════════════════════════════════════════════════
      // Mode 0: Normal Orientation Display — clean, readable layout
      // ═══════════════════════════════════════════════════════
      float displayRoll  = roll  - rollOffset;
      float displayPitch = pitch - pitchOffset;
      float displayYaw   = yaw   - yawOffset;

      char angleBuf[12];

      // --- Top bar: inverted "INCLINOMETER" header ---
      display.fillRect(0, 0, 128, 11, SSD1306_WHITE);
      display.setTextSize(1);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(22, 2);
      display.print("INCLINOMETER");
      display.setTextColor(SSD1306_WHITE);  // Reset for below

      // --- Row 1: Roll — label small, value large ---
      // Y = 13
      display.setTextSize(1);
      display.setCursor(0, 15);
      display.print("R");

      display.setTextSize(2);
      display.setCursor(10, 13);
      formatAngle(angleBuf, displayRoll);
      display.print(angleBuf);
      // Degree symbol at text size 1 after the large number
      display.setTextSize(1);
      display.setCursor(82, 13);
      display.print((char)247);

      // --- Row 2: Pitch ---
      // Y = 31
      display.setTextSize(1);
      display.setCursor(0, 33);
      display.print("P");

      display.setTextSize(2);
      display.setCursor(10, 31);
      formatAngle(angleBuf, displayPitch);
      display.print(angleBuf);
      display.setTextSize(1);
      display.setCursor(82, 31);
      display.print((char)247);

      // --- Thin separator ---
      display.drawFastHLine(0, 48, 128, SSD1306_WHITE);

      // --- Row 3: Yaw (compass heading) — bottom section ---
      // Y = 50
      display.setTextSize(1);
      display.setCursor(0, 52);
      display.print("Y");

      display.setCursor(10, 52);
      formatAngle(angleBuf, displayYaw);
      display.print(angleBuf);
      display.print((char)247);

      // --- Gesture label — right side of bottom row ---
      String upperGesture = gestureLabel;
      upperGesture.toUpperCase();

      // Small vertical separator between yaw and gesture
      display.drawFastVLine(68, 50, 14, SSD1306_WHITE);

      // Filled accent box for gesture
      display.fillRect(70, 50, 58, 14, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      // Center gesture text within box
      int gestLen = upperGesture.length();
      int gestX = 70 + (58 - gestLen * 6) / 2;
      if (gestX < 72) gestX = 72;
      display.setCursor(gestX, 53);
      display.print(upperGesture);
      display.setTextColor(SSD1306_WHITE);  // Restore

    } else {
      // ═══════════════════════════════════════════════════════
      // Modes 1–3: Bubble Level Display
      // ═══════════════════════════════════════════════════════
      float displayRoll  = roll  - rollOffset;
      float displayPitch = pitch - pitchOffset;
      drawBubbleLevel(currentMode, displayRoll, displayPitch);
    }

    // Push to display
    display.display();
  }
}