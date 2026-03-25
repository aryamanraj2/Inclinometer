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
// Helper: Format angle string with explicit sign and 2 decimals
// Writes into the provided buffer (must be at least 10 chars)
// ============================================================
void formatAngle(char* buf, float angle) {
  char sign = (angle >= 0.0f) ? '+' : '-';
  float absAngle = fabs(angle);
  // Format: +123.45 or -005.67
  sprintf(buf, "%c%06.2f", sign, absAngle);
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
  display.setTextSize(1);
  display.setCursor(20, 10);
  display.println("ORIENTATION");
  display.setCursor(30, 25);
  display.println("SYSTEM");
  display.setCursor(25, 45);
  display.println("Starting...");
  display.display();
  delay(1000);  // Only delay in setup — splash screen

  // --- Madgwick Filter ---
  // Begin with the target sensor sample rate (100 Hz)
  madgwickFilter.begin(100);

  // Record starting times
  prevSensorTime = millis();
  prevDisplayTime = millis();

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
    // 6. Serial output for debugging
    // ---------------------------------------------------------
    char rollStr[12], pitchStr[12], yawStr[12];
    formatAngle(rollStr, roll);
    formatAngle(pitchStr, pitch);
    formatAngle(yawStr, yaw);

    Serial.print("Roll: ");
    Serial.print(rollStr);
    Serial.print("° | Pitch: ");
    Serial.print(pitchStr);
    Serial.print("° | Yaw: ");
    Serial.print(yawStr);
    Serial.println("°");
  }

  // ===========================================================
  // OLED DISPLAY UPDATE — runs at ~10 Hz (every 100ms)
  // Independent of sensor loop
  // ===========================================================
  if (now - prevDisplayTime >= OLED_UPDATE_INTERVAL_MS) {
    prevDisplayTime = now;

    display.clearDisplay();

    // --- Header ---
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(22, 0);
    display.println("ORIENTATION");

    // Separator line
    display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    // --- Angle Values ---
    char angleBuf[12];

    // Roll
    display.setTextSize(1);
    display.setCursor(0, 16);
    display.print("Roll  : ");
    formatAngle(angleBuf, roll);
    display.setTextSize(1);
    display.setCursor(50, 16);
    display.print(angleBuf);
    display.print((char)247);  // ° symbol

    // Pitch
    display.setCursor(0, 30);
    display.print("Pitch : ");
    formatAngle(angleBuf, pitch);
    display.setCursor(50, 30);
    display.print(angleBuf);
    display.print((char)247);

    // Yaw
    display.setCursor(0, 44);
    display.print("Yaw   : ");
    formatAngle(angleBuf, yaw);
    display.setCursor(50, 44);
    display.print(angleBuf);
    display.print((char)247);

    // Bottom separator
    display.drawLine(0, 56, 127, 56, SSD1306_WHITE);

    // Push to display
    display.display();
  }
}