#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

// Pre-determined offsets (Replace these values with your calibrated offsets)
const int accX_offset = 0.02; // Example accelerometer X offset
const int accY_offset = -0.01; // Example accelerometer Y offset
const int accZ_offset = -0.13; // Example accelerometer Z offset

const int gyroX_offset = -1.87;   // Example gyroscope X offset
const int gyroY_offset = 1.23;  // Example gyroscope Y offset
const int gyroZ_offset = -0.44;   // Example gyroscope Z offset

// const int accX_offset = 0; // Example accelerometer X offset
// const int accY_offset = 0; // Example accelerometer Y offset
// const int accZ_offset = 0; // Example accelerometer Z offset

// const int gyroX_offset = 0;   // Example gyroscope X offset
// const int gyroY_offset = 0;  // Example gyroscope Y offset
// const int gyroZ_offset = 0;   // Example gyroscope Z offset

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize the MPU6050
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 initialization failed with status code: ");
    Serial.println(status);
    while (1); // Stop if initialization fails
  }

  Serial.println("MPU6050 initialized successfully!");

  // Apply pre-determined offsets
  mpu.setAccOffsets(accX_offset, accY_offset, accZ_offset);
  mpu.setGyroOffsets(gyroX_offset, gyroY_offset, gyroZ_offset);
  Serial.println("Pre-determined offsets applied.");
}

void loop() {
  // Update MPU6050 readings
  mpu.update();

  // Get angles
  float pitch = mpu.getAngleX(); // Pitch (rotation around X-axis)
  float roll = mpu.getAngleY();  // Roll (rotation around Y-axis)
  float yaw = mpu.getAngleZ();   // Yaw (rotation around Z-axis)

  // Print angles to Serial Monitor
  Serial.print("Pitch: ");
  Serial.print(roll);
  Serial.print("\n");
  // Serial.print("°, Roll: ");
  // Serial.print(roll);
  // Serial.print("°, Yaw: ");
  // Serial.println(yaw);

  // Optional delay
  delay(10);
}

