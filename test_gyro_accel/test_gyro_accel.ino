// Basic demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float roll_estimate = 0.0;
float pitch_estimate = 0.0;

void setup() {

  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");

  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
    /* Get new sensor events with the readings */
    
  mpu.getEvent(&a, &g, &temp);
  /* Print out the values */
  // Serial.print("Acceleration X:");
  // Serial.print(a.acceleration.x);
  // Serial.print(", Y:");
  // Serial.print(a.acceleration.y);
  // Serial.print(", Z:");
  // Serial.print(a.acceleration.z);
  // Serial.print(" m/s^2\n");

  // Serial.print("Rotation X:");
  // Serial.print(g.gyro.x);
  // Serial.print(", Y:");
  // Serial.print(g.gyro.y);
  // Serial.print(", Z:");
  // Serial.print(g.gyro.z);
  // Serial.print(" rad/s\n");

  // Serial.print("Temperature: ");
  // Serial.print(temp.temperature);
  // Serial.println(" degC");

  float sample_time = 0.004;
  float estimate_accel_pitch, estimate_accel_roll, estimate_gyro_pitch_rate, estimate_gyro_roll_rate;
  float alpha = 0.95;

  estimate_accel_roll = asin(a.acceleration.x/9.81);
  estimate_accel_pitch = atan(a.acceleration.y/a.acceleration.z);

  estimate_gyro_pitch_rate = g.gyro.x+g.gyro.y*sin(pitch_estimate)*tan(roll_estimate)+g.gyro.z*cos(pitch_estimate)*tan(roll_estimate);
  estimate_gyro_roll_rate = g.gyro.y*cos(pitch_estimate)-g.gyro.z*sin(pitch_estimate);

  pitch_estimate = estimate_accel_pitch*alpha + (1-alpha)*(pitch_estimate + sample_time * estimate_gyro_pitch_rate);
  roll_estimate = (estimate_accel_roll*alpha + (1-alpha)*(roll_estimate + sample_time * estimate_gyro_roll_rate));

  // Serial.print("Pitch from Accel:");
  // Serial.print(estimate_accel_pitch);
  // Serial.print(", Roll from Accel:");
  // Serial.print(estimate_accel_roll);

  // Serial.print("Pitch from Gyro:");
  // Serial.print(estimate_gyro_pitch_rate);
  // Serial.print(", Roll from Gyro:");
  // Serial.print(estimate_gyro_roll_rate);

  Serial.print("Pitch Estimate:");
  Serial.print(pitch_estimate*180/PI);
  Serial.print(", Roll Estimate:");
  Serial.print(roll_estimate*180/PI);

  Serial.println(" ");
  delay(4);
}