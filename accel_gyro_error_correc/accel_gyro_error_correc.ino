#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>

////MPU6050 accelgyro;
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;


//IMU Variables
//MPU6050 accelgyro;
//Adafruit_MPU6050 mpu;
//sensors_event_t a, g, temp;
// int16_t ax, ay, az;
// int16_t gx, gy, gz;
float ax1, ay1, az1;
float gx1, gy1, gz1;
//float roll, pitch, yaw;
//float gyroAngleX, gyroAngleY, gyroAngleZ;

int c = 0;
int count = 0;
float AccErrorX, AccErrorY, AccErrorZ;
float GyroErrorX, GyroErrorY, GyroErrorZ;

int max_count = 700;int looptime = 50;


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
Serial.print("Connecting ");
Serial.println("Hi");



//  Serial.println(" CP 1");
//  mpu.begin();
if (!mpu.begin()) {
  Serial.println("Failed to find MPU6050 chip");
  while (1) {
    delay(10);
  }
}
 Serial.println("MPU6050 Found!");
//  Serial.println(" CP 3");

Serial.println(MPU6050_RANGE_2_G);
int range = mpu.getAccelerometerRange();
Serial.println(range);
delay(50);
mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
delay(50);
range = mpu.getAccelerometerRange();
Serial.println(range);
delay(50);
mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
delay(50);
range = mpu.getAccelerometerRange();
Serial.println(range);

//  delay(5000);
}

void loop() {
//   delay(50);
// return;
if (c == 0){  
 
  while( count < max_count){
    delay(looptime);
    mpu.getEvent(&a, &g, &temp);
//    currentTimeEnc = millis();
    ax1 = a.acceleration.x;
    ay1 = a.acceleration.y;
    az1 = a.acceleration.z;
    gx1 = g.gyro.x;
    gy1 = g.gyro.y;
    gz1 = g.gyro.z;
 
 

    AccErrorX = AccErrorX + ax1;
    AccErrorY = AccErrorY + ay1;
    AccErrorZ = AccErrorZ + az1;
 
    GyroErrorX = GyroErrorX + gx1;
    GyroErrorY = GyroErrorY + gy1;
    GyroErrorZ = GyroErrorZ + gz1;
    count++;

    Serial.println(" AccAngleX AccErrX AccErrY AccErrZ gx1 gy1 \tgz1 Count");
//    Serial.print((atan((az1)/sqrt(pow(ax1,2)+pow(ay1,2)))));
//    Serial.print("\t");
//    Serial.print((atan(-1*ax1/sqrt(pow(az1,2)+pow(ay1,2)))));
//    Serial.print("\t");
    Serial.print(atan2((-ay1),(az1))*180/PI);
    Serial.print("\t");
    Serial.print(ax1);
    Serial.print("\t");
    Serial.print(ay1);
    Serial.print("\t");
    Serial.print(az1);
    Serial.print("\t");
    Serial.print(gx1);
    Serial.print("\t");
    Serial.print(gy1);
    Serial.print("\t");
    Serial.print(gz1);
    Serial.print("\t");
    Serial.print(count);
    Serial.println("\t");
  }

  GyroErrorX = GyroErrorX/max_count;
  GyroErrorY = GyroErrorY/max_count;
  GyroErrorZ = GyroErrorZ/max_count;
 
  AccErrorX = AccErrorX/max_count;
  AccErrorY = (AccErrorY/max_count);
  AccErrorZ = AccErrorZ/max_count;
  c++;
  Serial.println("Done");
  Serial.println("Done");
  Serial.println("Done");
  Serial.println("Done");
  Serial.println("Done");
  }

  Serial.println(" AccErrX AccErrY AccErrZ GyroErrX GyroErrY GyroErrZ Count");
  Serial.print(AccErrorX,4);
  Serial.print("\t");
  Serial.print(AccErrorY,4);
  Serial.print("\t");
  Serial.print(AccErrorZ,4);
  Serial.print("\t");
  Serial.print(GyroErrorX,4);
  Serial.print("\t");
  Serial.print(GyroErrorY,4);
  Serial.print("\t");
  Serial.print(GyroErrorZ,4);
  Serial.print("\t");
  Serial.print(count);
  Serial.println("\t");


  while(1){
    delay(10000);
  }
}