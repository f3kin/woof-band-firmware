#include "mywifi.h"
#include "pedometer.h"
#include "myComms.h"


unsigned long lastSendTime = 0;
const unsigned long sendInterval = 5000;  // Send data every 5 seconds

void mpu_begin() {
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
  mpu6050_calibrate(accel_bias, gyro_bias); 
  Serial.println("MPU6050 calibration finished!");

  // Setup motion detection
  mpu.setDLPFMode(MPU6050_DLPF_BW_5); // Low-pass filter setting similar to HighPassFilter in Adafruit
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(1);
  mpu.setInterruptLatch(1); // Keep it latched. Will turn off when reinitialized.
  mpu.setInterruptMode(0);
  mpu.setIntMotionEnabled(true);
}


void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }
  mpu_begin();
  wifi_setup();
  getCurrentTimeStamp();
  delay(100);
  ready_post();
}

void loop() {
  // Start and stop recording POST
  if (not started){
    record_post(true);
  }
  // Run step counter every loop iteration
  AccelData data = step_counter();
  
  // Run WiFi-related functions (e.g., maintain connection)
  // wifi_loop();

  // Share data with server, change boolean to false if not recording graphical data
  send_data_to_server(step_count, false, data);

  // Add a small delay to allow other tasks to execute
  vTaskDelay(10 / portTICK_PERIOD_MS);  // 10 ms delay
}