// This code is for the failsafe of the Drone when control signal is lost

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

#define MOTOR1_PIN 32
#define MOTOR2_PIN 26
#define MOTOR3_PIN 33
#define MOTOR4_PIN 25

Adafruit_MPU6050 mpu;

unsigned long loop_timer;

Servo MOTOR1, MOTOR2, MOTOR3, MOTOR4;

float roll_estimate = 0.0;
float pitch_estimate = 0.0;

float prev_pitch_error = 0.0;
float prev_roll_error = 0.0;

void setup() 
{

  Serial.begin(115200);

  // while (!Serial)
  //   delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Motor Setup!");

  MOTOR1.attach(MOTOR1_PIN, 1000, 2000);
  MOTOR2.attach(MOTOR2_PIN, 1000, 2000);
  MOTOR3.attach(MOTOR3_PIN, 1000, 2000);
  MOTOR4.attach(MOTOR4_PIN, 1000, 2000);
  delay(4000);

  Serial.println("Motor Test!");

  MOTOR1.writeMicroseconds(1200);
  MOTOR2.writeMicroseconds(1200);
  MOTOR3.writeMicroseconds(1200);
  MOTOR4.writeMicroseconds(1200);
  delay(2000);

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



  delay(1000);
}

void loop() {
  sensors_event_t a, g, temp;
    /* Get new sensor events with the readings */
    
  mpu.getEvent(&a, &g, &temp);

  float K_Proll = 5;
  float K_Droll = 0.5;
  float K_Iroll = 0.0;
  float K_Ppitch = 5;
  float K_Dpitch = 0.5;
  float K_Ipitch = 0.0;

  int N_1, N_2, N_3, N_4;
  float N_thrust, N_pitch, N_roll;

  float sample_time = 0.004;
  float estimate_accel_pitch, estimate_accel_roll, estimate_gyro_pitch_rate, estimate_gyro_roll_rate;
  float alpha = 0.95;

  estimate_accel_roll = asin(a.acceleration.x/9.81);
  estimate_accel_pitch = atan(a.acceleration.y/a.acceleration.z);

  estimate_gyro_pitch_rate = g.gyro.x+g.gyro.y*sin(pitch_estimate)*tan(roll_estimate)+g.gyro.z*cos(pitch_estimate)*tan(roll_estimate);
  estimate_gyro_roll_rate = g.gyro.y*cos(pitch_estimate)-g.gyro.z*sin(pitch_estimate);

  pitch_estimate = estimate_accel_pitch*alpha + (1-alpha)*(pitch_estimate + sample_time * estimate_gyro_pitch_rate);
  roll_estimate = (estimate_accel_roll*alpha + (1-alpha)*(roll_estimate + sample_time * estimate_gyro_roll_rate)) - 0.05;

  // Eliminating the bias term from the estimates
  // pitch_estimate = pitch_estimate*180/PI;
  // roll_estimate = roll_estimate*180/PI;

  // Serial.print("Pitch Estimate:");
  // Serial.print(pitch_estimate);
  // Serial.print(", Roll Estimate:");
  // Serial.print(roll_estimate);

  // Serial.println(" ");
  N_thrust = 1250;
  N_roll = K_Proll*(roll_estimate) + K_Droll*(roll_estimate-prev_roll_error)/sample_time + K_Iroll*(roll_estimate+prev_roll_error)*sample_time/2;
  N_pitch = K_Ppitch*(pitch_estimate) + K_Dpitch*(pitch_estimate-prev_pitch_error)/sample_time + K_Ipitch*(pitch_estimate+prev_pitch_error)*sample_time/2;

  // N_roll = int(round(N_roll));
  // N_pitch = int(round(N_pitch));
  prev_roll_error = roll_estimate;
  prev_pitch_error = pitch_estimate;
  // prev_roll_error = roll_estimate;

  N_1 = N_thrust - N_pitch - N_roll;
  N_2 = N_thrust + N_pitch - N_roll;
  N_3 = N_thrust + N_pitch + N_roll;
  N_4 = N_thrust - N_pitch + N_roll;

  Serial.print("Motor 1 speed command:");
  Serial.println(N_1);
  Serial.print("Motor 2 speed command:");
  Serial.println(N_2);
  Serial.print("Motor 3 speed command:");
  Serial.println(N_3);
  Serial.print("Motor 4 speed command:");
  Serial.println(N_4);
  Serial.println(" ");

  MOTOR1.writeMicroseconds(N_1);
  MOTOR2.writeMicroseconds(N_2);
  MOTOR3.writeMicroseconds(N_3);
  MOTOR4.writeMicroseconds(N_4);

  // MOTOR1.writeMicroseconds(1200);
  // MOTOR2.writeMicroseconds(0);
  // MOTOR3.writeMicroseconds(0);
  // MOTOR4.writeMicroseconds(0);
  

  delay(sample_time*1000);
}