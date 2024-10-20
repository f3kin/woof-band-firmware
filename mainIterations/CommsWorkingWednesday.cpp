// #include "communication.h"
#include "mywifi.h"
#include "pedometer.h"
#include "myComms.h"


void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); 
  }
  // SDA -> 21
  // SCL -> 22
  Wire.begin(21, 22);

  mpu.initialize();
  if (!mpu.testConnection()) {
      Serial.println("Failed to connect to MPU6050");
      while (1) {
      delay(10);
      }
  }
  Serial.println("MPU6050 Connected!");


  // Calibrate the MPU6050
  Serial.println("MPU6050 calibrating...");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println("MPU6050 calibration finished!");

  Serial.print("Accel Bias :");
  Serial.print(accel_bias[0]);
  Serial.print(",");
  Serial.print(accel_bias[1]);
  Serial.print(",");
  Serial.print(accel_bias[2]);
  Serial.print(", ");
  Serial.print("Gyro Bias :");
  Serial.print(gyro_bias[0]);
  Serial.print(",");
  Serial.print(gyro_bias[1]);
  Serial.print(",");
  Serial.print(gyro_bias[2]);
  Serial.println("");

  // Setup motion detection
  mpu.setDLPFMode(MPU6050_DLPF_BW_5); // Low-pass filter setting similar to HighPassFilter in Adafruit
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(1);
  mpu.setInterruptLatch(1); // Keep it latched. Will turn off when reinitialized.
  mpu.setInterruptMode(0);
  mpu.setIntMotionEnabled(true);

  wifi_setup();

  delay(100);
}


void loop() {
  wifi_loop();
  step_counter();
  vTaskDelay(10 / portTICK_PERIOD_MS);
  send_data_to_server(step_count);
  Serial.print("Current step count: ");
  Serial.println(step_count);
  Serial.println("###############");
}
