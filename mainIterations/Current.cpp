#include <Wire.h>
#include <MPU6050.h>
#include "esp_timer.h"
#include "communication.h"
#include "MyMPU.h"

float accel_bias[3] = {0, 0, 0};
float gyro_bias[3] = {0, 0, 0};

float quart[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float delta_t = 0.0f;
#define AVG_BUFF_SIZE 20

// Variables for sensor readings
int16_t ax, ay, az;
int16_t gx, gy, gz;

uint64_t now_time = 0, prev_time = 0;
float accel_g_x, accel_g_y, accel_g_z;
float accel_int_x, accel_int_y, accel_int_z;
float gyro_ds_x, gyro_ds_y, gyro_ds_z;
int accel_x_avg_buff[AVG_BUFF_SIZE];
int accel_y_avg_buff[AVG_BUFF_SIZE];
int accel_z_avg_buff[AVG_BUFF_SIZE];
int accel_x_avg_buff_count = 0;
int accel_y_avg_buff_count = 0;
int accel_z_avg_buff_count = 0;
int accel_x_avg, accel_y_avg, accel_z_avg;
int min_reg_accel_x = 0, min_reg_accel_y = 0, min_reg_accel_z = 0;
int max_reg_accel_x = 0, max_reg_accel_y = 0, max_reg_accel_z = 0;
int min_curr_accel_x, min_curr_accel_y, min_curr_accel_z;
int max_curr_accel_x, max_curr_accel_y, max_curr_accel_z;
int dy_thres_accel_x = 0, dy_thres_accel_y = 0, dy_thres_accel_z = 0;
int dy_chan_accel_x, dy_chan_accel_y, dy_chan_accel_z;
int sample_new = 0, sample_old = 0;
int step_size = 200;
int active_axis = 0, interval = 500000;
int step_changed = 0;

bool ledReadOn = true;
bool bluetoothConnected = true;

#define RESISTOR_R1 17800
#define RESISTOR_R2 9640

float mapping(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
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

  delay(100);
}

void step_counter() {
  // Retrieve raw data from MPU6050
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw accelerometer data to g’s (assuming default range of ±2g)
  accel_g_x = (float)ax / 16384.0 - accel_bias[0];
  accel_g_y = (float)ay / 16384.0 - accel_bias[1];
  accel_g_z = (float)az / 16384.0 - accel_bias[2];

  // Ensure accelerometer values are positive
  if (accel_g_x < 0) accel_g_x *= -1;
  if (accel_g_y < 0) accel_g_y *= -1;
  if (accel_g_z < 0) accel_g_z *= -1;

  // Convert raw gyroscope data to degrees per second (dps)
  gyro_ds_x = (float)gx / 131.0 - gyro_bias[0]; // Assuming default range of ±250°/s
  gyro_ds_y = (float)gy / 131.0 - gyro_bias[1];
  gyro_ds_z = (float)gz / 131.0 - gyro_bias[2];

  // Update quaternion using the Madgwick filter (assumes you've implemented this function)
  mpu6050_madgwick_quaternion_update(accel_g_x,
                                     accel_g_y,
                                     accel_g_z,
                                     gyro_ds_x * PI / 180.0f,
                                     gyro_ds_y * PI / 180.0f,
                                     gyro_ds_z * PI / 180.0f,
                                     quart,
                                     delta_t);

  // Integrate accelerometer readings
  accel_int_x = 1000 * accel_g_x;
  accel_int_y = 1000 * accel_g_y;
  accel_int_z = 1000 * accel_g_z;

  // Update moving average buffers
  accel_x_avg_buff[accel_x_avg_buff_count] = accel_int_x;
  accel_x_avg_buff_count = (accel_x_avg_buff_count + 1) % AVG_BUFF_SIZE;
  accel_x_avg = 0;
  for (int i = 0; i < AVG_BUFF_SIZE; i++) accel_x_avg += accel_x_avg_buff[i];
  accel_x_avg /= AVG_BUFF_SIZE;

  accel_y_avg_buff[accel_y_avg_buff_count] = accel_int_y;
  accel_y_avg_buff_count = (accel_y_avg_buff_count + 1) % AVG_BUFF_SIZE;
  accel_y_avg = 0;
  for (int i = 0; i < AVG_BUFF_SIZE; i++) accel_y_avg += accel_y_avg_buff[i];
  accel_y_avg /= AVG_BUFF_SIZE;

  accel_z_avg_buff[accel_z_avg_buff_count] = accel_int_z;
  accel_z_avg_buff_count = (accel_z_avg_buff_count + 1) % AVG_BUFF_SIZE;
  accel_z_avg = 0;
  for (int i = 0; i < AVG_BUFF_SIZE; i++) accel_z_avg += accel_z_avg_buff[i];
  accel_z_avg /= AVG_BUFF_SIZE;

  // Timing for step detection
  now_time = esp_timer_get_time();
  if (now_time - prev_time >= interval) {
    prev_time = now_time;

    min_curr_accel_x = min_reg_accel_x;
    max_curr_accel_x = max_reg_accel_x;
    dy_thres_accel_x = (min_curr_accel_x + max_curr_accel_x) / 2;
    dy_chan_accel_x = (max_reg_accel_x - min_curr_accel_x);
    min_reg_accel_x = accel_x_avg;
    max_reg_accel_x = accel_x_avg;

    min_curr_accel_y = min_reg_accel_y;
    max_curr_accel_y = max_reg_accel_y;
    dy_thres_accel_y = (min_curr_accel_y + max_curr_accel_y) / 2;
    dy_chan_accel_y = (max_reg_accel_y - min_curr_accel_y);
    min_reg_accel_y = accel_y_avg;
    max_reg_accel_y = accel_y_avg;

    min_curr_accel_z = min_reg_accel_z;
    max_curr_accel_z = max_reg_accel_z;
    dy_thres_accel_z = (min_curr_accel_z + max_curr_accel_z) / 2;
    dy_chan_accel_z = (max_reg_accel_z - min_curr_accel_z);
    min_reg_accel_z = accel_z_avg;
    max_reg_accel_z = accel_z_avg;

    if (dy_chan_accel_x >= dy_chan_accel_y && dy_chan_accel_x >= dy_chan_accel_z) {
      if (active_axis != 0) {
        sample_old = 0;
        sample_new = accel_x_avg;
      }
      active_axis = 0;
    } else if (dy_chan_accel_y >= dy_chan_accel_x && dy_chan_accel_y >= dy_chan_accel_z) {
      if (active_axis != 1) {
        sample_old = 0;
        sample_new = accel_y_avg;
      }
      active_axis = 1;
    } else {
      if (active_axis != 2) {
        sample_old = 0;
        sample_new = accel_z_avg;
      }
      active_axis = 2;
    }
  } else if (now_time < 500) {
    if (min_reg_accel_x > accel_x_avg) min_reg_accel_x = accel_x_avg;
    if (max_reg_accel_x < accel_x_avg) max_reg_accel_x = accel_x_avg;

    if (min_reg_accel_y > accel_y_avg) min_reg_accel_y = accel_y_avg;
    if (max_reg_accel_y < accel_y_avg) max_reg_accel_y = accel_y_avg;

    if (min_reg_accel_z > accel_z_avg) min_reg_accel_z = accel_z_avg;
    if (max_reg_accel_z < accel_z_avg) max_reg_accel_z = accel_z_avg;
  }

  sample_old = sample_new;
  switch (active_axis) {
    case 0:
      if (accel_x_avg - sample_old > step_size || accel_x_avg - sample_old < -step_size) {
        sample_new = accel_x_avg;
        if (sample_old > dy_thres_accel_x && sample_new < dy_thres_accel_x) {
          step_count++;
          step_changed = 1;
        }
      }
      break;
    case 1:
      if (accel_y_avg - sample_old > step_size || accel_y_avg - sample_old < -step_size) {
        sample_new = accel_y_avg;
        if (sample_old > dy_thres_accel_y && sample_new < dy_thres_accel_y) {
          step_count++;
          step_changed = 1;
        }
      }
      break;
    case 2:
      if (accel_z_avg - sample_old > step_size || accel_z_avg - sample_old < -step_size) {
        sample_new = accel_z_avg;
        if (sample_old > dy_thres_accel_z && sample_new < dy_thres_accel_z) {
          step_count++;
          step_changed = 1;
        }
      }
      break;
  }

  if (step_changed) {
    Serial.print("Step Counter change to:");
    Serial.println(step_count);
    step_changed = 0;
  }
}


void loop() {
  step_counter();
  vTaskDelay(10 / portTICK_PERIOD_MS);
  if (handleLoop()) {
    Serial.println("Data Success");
  }
}
