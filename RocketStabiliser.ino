

#include<Wire.h>
#include <SPI.h>
//#include <SD.h>
#include <Servo.h>
#include <MsTimer2.h>

float dt = 0.1; // Sample Period
float GYROSCOPE_SENSITIVITY = 65.536;
float ACCELEROMETER_SENSITIVITY = 8192.0;
bool INITTING = true;

Servo servo1, servo2;  // create servo object to control a servo
//File myFile;
int servoSpeedFactor = 5;
float angleX = 0, angleY = 0, angleZ = 0;
const int BUFFERSIZE = 10;
float bX[BUFFERSIZE], bY[BUFFERSIZE], bZ[BUFFERSIZE];
int biX = 0, biY = 0, biZ = 0;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int16_t offset_AcX,offset_AcY,offset_AcZ,offset_GyX,offset_GyY,offset_GyZ;
int initCount = 0;

void SetRedLed(bool state) {
  if (state) {
    Serial.println("R LED on");
    digitalWrite(8,  HIGH);
  } else {
    Serial.println("R LED off");
    digitalWrite(8,  LOW);
  }
}

void SetGreenLed(bool state) {
  if (state) {
    Serial.println("G LED on");
    digitalWrite(9,  HIGH);
  } else {
    Serial.println("G LED off");
    digitalWrite(9,  LOW);
  }
}

void InitialiseServo(int pin, Servo target, int initialPosition) {
  target.attach(pin);  // attaches the servo on pin to the servo object
  target.write(initialPosition); // centre servo
  delay(500); // pause for coolness
  
  SweepServo(target, 135, servoSpeedFactor);   
  SweepServo(target, 45, servoSpeedFactor);
  SweepServo(target, initialPosition, servoSpeedFactor);   
}

void SweepServo(Servo target, int destination, int waitTime) {
  int inc;
  int pos = target.read();
  if (pos >= destination) {
    inc = -1;
  } else {
    inc = 1;
  }
  for (; pos != destination; pos += inc) {   // goes from start degrees to end degrees in steps of increment degrees
    target.write(pos);                   // tell servo to go to position in variable 'pos'
    delay(waitTime);
  }
}

void InitialiseGyro() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void setup(){
  Serial.begin(9600);
  Serial.println("Init LEDs...");
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  SetRedLed(true);

  Serial.println("Init servo 1...");
  InitialiseServo(3, servo1, 90);
  Serial.println("Init servo 2...");
  InitialiseServo(5, servo2, 90);
  
  Serial.println("Init gyro...");
  InitialiseGyro();
  
  /*Serial.println("Init SD card...");
  if (!SD.begin(4)) {
    Serial.println("Init SD failed! Bailing.");
    return;
  }*/

  /*Serial.println("Init file...");
  myFile = SD.open("Log.txt", FILE_WRITE);
  myFile.println("Starting.");
  myFile.close();*/

  Serial.println("Init timer...");
  MsTimer2::set(100, TakeValues);
  MsTimer2::start();

  Serial.println("Init done.");
}

void TakeValues() {
  sei();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)  
  GyX /= GYROSCOPE_SENSITIVITY;
  GyY /= GYROSCOPE_SENSITIVITY;
  GyZ /= GYROSCOPE_SENSITIVITY;
  AcX /= ACCELEROMETER_SENSITIVITY;
  AcY /= ACCELEROMETER_SENSITIVITY;
  AcZ /= ACCELEROMETER_SENSITIVITY;

  if (INITTING) {
    offset_GyX += (GyX - 90);
    offset_GyY += (GyY - 90);
    offset_GyZ += (GyZ - 90);  
    offset_AcX += (AcX - 90);
    offset_AcY += (AcY - 90);
    offset_AcZ += (AcZ - 90);
    initCount++;
    if (initCount > 100) {
      SetRedLed(false);
      INITTING = false;
    }
  } else {
    if (initCount > 0) {
      initCount = 0;
      offset_GyX /= initCount;
      offset_GyY /= initCount;
      offset_GyZ /= initCount;
      offset_AcX /= initCount;
      offset_AcY /= initCount;    
      offset_AcZ /= initCount;     

      offset_GyX = 0;
      offset_GyY = 0;
      offset_GyZ = 0;
      offset_AcX = 0;
      offset_AcY = 0;    
      offset_AcZ = 0;     
    }
    
    angleX = (0.9 * angleX) + (0.1 * (0.8 * (angleX + ((GyX + offset_GyX) * dt))) + (0.2 * (AcX + offset_AcX)));
    angleY = (0.9 * angleY) + (0.1 * (0.8 * (angleY + ((GyY + offset_GyY) * dt))) + (0.2 * (AcY + offset_AcY)));
    angleZ = (0.9 * angleZ) + (0.1 * (0.8 * (angleZ + ((GyZ + offset_GyZ) * dt))) + (0.2 * (AcZ + offset_AcZ)));
    
    bX[biX] = angleX;
    if (++biX == BUFFERSIZE) {
      biX = 0;
    }
    
    bY[biY] = angleY;
    if (++biY == BUFFERSIZE) {
      biY = 0;
    }
    
    bZ[biZ] = angleZ;
    if (++biZ == BUFFERSIZE) {
      biZ = 0;
    }
  }

  /*if (angleX >= 45 && angleX <= 135) {
    servo1.write((angleX + 90.0) % 180);
  } else {
    servo1.write(90);
  }

  if (angleY >= 45 && angleY <= 135) {
    servo1.write(angleY + 90.0);
  } else {
    servo1.write(90);
  }*/
}


float GetTotal(float values[]) {
  int siz = sizeof(values);
  float result = 0.0;
  for (int i = 0; i < siz; i++) {
    result += values[i];
  }
  result /= siz;
  return result;
}

bool ledState = false;

void loop(){
  ledState = !ledState;
  SetGreenLed(ledState);
  int tx, ty, tz;
  tx = GetTotal(bX);
  ty = GetTotal(bY);
  tz = GetTotal(bZ);
  //myFile = SD.open("Log.txt", FILE_WRITE);
  Serial.print(" | AnX = "); Serial.println(tx);
  //myFile.print("AnX = "); myFile.println(tx);
  Serial.print(" | AnY = "); Serial.println(ty);
  //myFile.print("AnY = "); myFile.println(ty);
  Serial.print(" | AnZ = "); Serial.println(tz);
  //myFile.print("AnZ = "); myFile.println(tz);
  //myFile.close();
  delay(1000);
}
