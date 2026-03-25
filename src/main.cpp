#include <Wire.h>
#include <MPU6050.h>

// Create an MPU6050 object to represent the sensor
MPU6050 mpu;

// Roll, Pitch, Yaw angles (degrees)
float roll = 0, pitch = 0, yaw = 0;

// Timing
unsigned long prevTime = 0;

// Complementary filter weight (0.0–1.0)
// Higher = trust gyro more, Lower = trust accelerometer more
const float alpha = 0.96;

void setup() {
  // Start serial communication at a baud rate of 115200
  Serial.begin(115200);
  
  // Initialize I2C communication (SDA = GPIO 21, SCL = GPIO 22 for ESP32)
  Wire.begin(21, 22);
  
  // Initialize the MPU6050 sensor
  mpu.initialize();
  
  // Check if the connection is successful
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  prevTime = millis();
}

void loop() {
  // Read raw sensor data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate elapsed time in seconds
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  // Convert accelerometer values to g's (±2g range, 16384 LSB/g)
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  // Convert gyroscope values to degrees/sec (±250°/s range, 131 LSB/°/s)
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;

  // Roll & Pitch from accelerometer (degrees)
  float accelRoll  = atan2(accelY, accelZ) * 180.0 / PI;
  float accelPitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  // Complementary filter: blend gyro integration with accelerometer angles
  roll  = alpha * (roll  + gyroX * dt) + (1.0 - alpha) * accelRoll;
  pitch = alpha * (pitch + gyroY * dt) + (1.0 - alpha) * accelPitch;

  // Yaw from gyro integration only (no magnetometer to correct drift)
  yaw += gyroZ * dt;

  // Print Roll, Pitch, Yaw
  Serial.print("Roll: ");  Serial.print(roll, 2);
  Serial.print("° | Pitch: "); Serial.print(pitch, 2);
  Serial.print("° | Yaw: ");   Serial.print(yaw, 2);
  Serial.println("°");

  delay(50);
}