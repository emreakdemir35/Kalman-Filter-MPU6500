#include <Arduino.h>
#include "Kalman.h"

// Define arrays to hold sensor data
float gyro[STATE_SIZE];        // Gyroscope input: gx, gy, gz (deg/s)
float accel[3];                // Accelerometer input: ax, ay, az (g)
float measurement[MEAS_SIZE];  // Combined measurement array
float filteredAngles[STATE_SIZE]; // Output: filtered pitch, roll, yaw

Kalman kf;  // Create Kalman filter instance

void setup() {
    Serial.begin(115200);

    // Optionally, initialize gyro/accel hardware here
    // e.g., MPU6050.begin(), configure IMU sampling rates, etc.

    // Initial state can be set to zero (assuming rocket is level at t=0)
    // kf._state is internally initialized to zero in constructor
}

void loop() {
    // ------------------- 1. READ IMU -------------------
    // Example: replace with actual IMU read functions
    // gyro[0] = readGyroX();  // deg/s
    // gyro[1] = readGyroY();
    // gyro[2] = readGyroZ();
    // accel[0] = readAccelX(); // g
    // accel[1] = readAccelY();
    // accel[2] = readAccelZ();

    // ------------------- 2. COMBINE MEASUREMENTS -------------------
    measurement[0] = accel[0]; // ax -> θx
    measurement[1] = accel[1]; // ay -> θy
    measurement[2] = accel[2]; // az -> not used in state directly
    measurement[3] = gyro[0];  // gx -> θx
    measurement[4] = gyro[1];  // gy -> θy
    measurement[5] = gyro[2];  // gz -> θz

    // ------------------- 3. PREDICT -------------------
    // Use gyro angular velocities to predict new angles
    kf.predict(gyro);

    // ------------------- 4. UPDATE -------------------
    // Correct prediction using accelerometer + gyro measurements
    kf.update(measurement);

    // ------------------- 5. GET FILTERED STATE -------------------
    kf.getState(filteredAngles);

    // ------------------- 6. USE FILTERED ANGLES -------------------
    // filteredAngles[0] -> pitch (θx)
    // filteredAngles[1] -> roll  (θy)
    // filteredAngles[2] -> yaw   (θz)
    Serial.print("Pitch: "); Serial.print(filteredAngles[0]);
    Serial.print(" | Roll: "); Serial.print(filteredAngles[1]);
    Serial.print(" | Yaw: "); Serial.println(filteredAngles[2]);

    delay(1); // ~1 ms delay for 1 kHz loop, adjust based on actual sampling
}
