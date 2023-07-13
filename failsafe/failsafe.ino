#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define MOTOR1_PIN 32
#define MOTOR2_PIN 26
#define MOTOR3_PIN 33
#define MOTOR4_PIN 25
#define RXD2 16
#define TXD2 17

int dist; /*----actual distance measurements of LiDAR---*/
int strength; /*----signal strength of LiDAR----------------*/
float temprature;
unsigned char check;        /*----save check value------------------------*/
int i;
unsigned char uart[9];  /*----save data measured by LiDAR-------------*/
const int HEADER=89; /*----frame header of data package------------*/
int rec_debug_state = 1;//receive state for frame


Adafruit_MPU6050 mpu;

unsigned long loop_timer;

float roll_estimate = 0.0;
float pitch_estimate = 0.0;

float prev_pitch_error = 0.0;
float prev_roll_error = 0.0;

Servo MOTOR1, MOTOR2, MOTOR3, MOTOR4;

void setup() {

  Serial.begin(115200);

  MOTOR1.attach(MOTOR1_PIN, 1000, 2000);
  MOTOR2.attach(MOTOR2_PIN, 1000, 2000);
  MOTOR3.attach(MOTOR3_PIN, 1000, 2000);
  MOTOR4.attach(MOTOR4_PIN, 1000, 2000);

  delay(1000);

  loop_timer = micros();
  while (micros() - loop_timer < 5000000) {
    MOTOR1.writeMicroseconds(1200);
    MOTOR2.writeMicroseconds(1200);
    MOTOR3.writeMicroseconds(1200);
    MOTOR4.writeMicroseconds(1200);
  }
  delay(100);
  loop_timer = micros();
  
  while (micros() - loop_timer < 5000000) {
    MOTOR1.writeMicroseconds(1000);
    MOTOR2.writeMicroseconds(1000);
    MOTOR3.writeMicroseconds(1000);
    MOTOR4.writeMicroseconds(1000);
  }
  delay(100);
  loop_timer = micros();

  if(!mpu.begin()){
    Serial.println("Failed to find MPU chip!");
    while(1){
      delay(10);
    }
  }
  
  Serial.println("MPU found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: +-8G");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: +-500 deg/s ");

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: 5 Hz");

  delay(1000);
  
}


void Get_Lidar_data(){
  
  if (Serial2.available()) //check if serial port has data input
      {
      if(rec_debug_state == 1)
          {  //the first byte
            uart[0]=Serial2.read();
            if(uart[0] == 89)
                {
                  check = uart[0];
                  rec_debug_state = 2;
                }
          }
          
      else if(rec_debug_state == 2)
         {//the second byte
          uart[1]=Serial2.read();
          
          if(uart[1] == 89)
              {
                check += uart[1];
                rec_debug_state = 3;
              }
          else{
                rec_debug_state = 1;
              }
      }
    
    else if(rec_debug_state == 3)
            {
              uart[2]=Serial2.read();
              check += uart[2];
              rec_debug_state = 4;
            }
            
    else if(rec_debug_state == 4)
            {
              uart[3]=Serial2.read();
              check += uart[3];
              rec_debug_state = 5;
            }
    else if(rec_debug_state == 5)
            {
              uart[4]=Serial2.read();
              check += uart[4];
              rec_debug_state = 6;
            }
    else if(rec_debug_state == 6)
            {
              uart[5]=Serial2.read();
              check += uart[5];
              rec_debug_state = 7;
            }
    else if(rec_debug_state == 7)
            {
              uart[6]=Serial2.read();
              check += uart[6];
              rec_debug_state = 8;
            }
    else if(rec_debug_state == 8)
            {
              uart[7]=Serial2.read();
              check += uart[7];
              rec_debug_state = 9;
            }
    
    else if (rec_debug_state == 9)
            {
              uart[8]=Serial2.read();
              if(uart[8] == check)
                
                {
                  
                  dist = uart[2] + uart[3]*256;//the distance
//                  strength = uart[4] + uart[5]*256;//the strength
//                  temprature = uart[6] + uart[7] *256;//calculate chip temprature
//                  temprature = temprature/8 - 256;                              
//                  Serial.print("dist = ");
//                  Serial.print(dist); //output measure distance value of LiDAR
//                  Serial.print('\n');
//                  Serial.print("strength = ");
//                  Serial.print(strength); //output signal strength value
//                  Serial.print('\n');
//                  Serial.print("\t Chip Temprature = ");
//                  Serial.print(temprature);
//                  Serial.println(" celcius degree"); //output chip temperature of Lidar                                                       
                  while(Serial2.available()){Serial2.read();} // This part is added becuase some previous packets are there in the buffer so to clear serial buffer and get fresh data.
                  delay(100);
                 }
                 
            rec_debug_state = 1;
          }
    }
}


void loop(){

  Get_Lidar_data();
  
  sensors_event_t a,g,temp;

  mpu.getEvent(&a, &g, &temp);

  float K_P;
  float K_D;
  float K_I;
  
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

  //N_thrust = 1420;

  N_thrust = K_P*(dist) + K_D*(dist-prev_dist)/sample_time + K_I*(dist + prev_dist)*sample_time
  
  N_roll = K_Proll*(roll_estimate) + K_Droll*(roll_estimate-prev_roll_error)/sample_time + K_Iroll*(roll_estimate+prev_roll_error)*sample_time/2;
  N_pitch = K_Ppitch*(pitch_estimate) + K_Dpitch*(pitch_estimate-prev_pitch_error)/sample_time + K_Ipitch*(pitch_estimate+prev_pitch_error)*sample_time/2;

  prev_roll_error = roll_estimate;
  prev_pitch_error = pitch_estimate;

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

  if (N_1<=1500 && N_2<=1500 && N_3<=1500 && N_4<=1500){
    
    MOTOR1.writeMicroseconds(N_1);
    MOTOR2.writeMicroseconds(N_2);
    MOTOR3.writeMicroseconds(N_3);
    MOTOR4.writeMicroseconds(N_4);
  }

  else{
  
    MOTOR1.writeMicroseconds(1420);
    MOTOR2.writeMicroseconds(1420);
    MOTOR3.writeMicroseconds(1420);
    MOTOR4.writeMicroseconds(1420);
  }

  prev_dist = dist;

  while(micros() - loop_timer < 1000);
  loop_timer = micros();
  
}
