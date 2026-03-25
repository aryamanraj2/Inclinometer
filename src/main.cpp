#include <Wire.h>
#include <MPU6050.h>

// Create an MPU6050 object to represent the sensor
MPU6050 mpu;

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
}

void loop() {
  // Declare six 16-bit integer variables to store sensor data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  
  // Read all six values (accelerometer and gyroscope) in a single call
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Print the accelerometer values on the serial monitor
  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" | Y: "); Serial.print(ay);
  Serial.print(" | Z: "); Serial.println(az);
  
  // Print the gyroscope values on the serial monitor
  Serial.print("Gyro X: "); Serial.print(gx);
  Serial.print(" | Y: "); Serial.print(gy);
  Serial.print(" | Z: "); Serial.println(gz);
  
  // Add a small delay so the output is easier to read
  delay(500);
}