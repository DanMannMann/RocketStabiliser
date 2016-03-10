

#include<Wire.h>
#include <SPI.h>
#include <SdFat.h>
SdFat SD;
#include <Servo.h>
#include <MsTimer2.h>
#define M_PI_4 0.78539816339744830962

float servo1Adjust = 0;
float servo2Adjust = -4;

bool smoothing = true;
float dt = 0.01; // Sample Period plus 4ms to run the code
float GYROSCOPE_SENSITIVITY = 131;
float ACCELEROMETER_SENSITIVITY = 16384.0;
float pitchAccel = 0, rollAccel = 0, pitchGyro = 0, rollGyro = 0, pitchValue = 0, rollValue = 0;
bool INITTING = true;
float pv = 0, rv = 0;

Servo servo1, servo2;  // create servo object to control a servo
File myFile;
int servoSpeedFactor = 5;
float angleX = 0, angleY = 0, angleZ = 0;
float velocityX = 0, velocityY = 0, velocityZ = 0;
float resultX = 0, resultY = 0, resultZ = 0;
const int BUFFERSIZE = 10;
float bX[BUFFERSIZE], bY[BUFFERSIZE], bZ[BUFFERSIZE];
float gX[BUFFERSIZE], gY[BUFFERSIZE], gZ[BUFFERSIZE];
int giX = 0, giY = 0, giZ = 0;
int aiX = 0, aiY = 0, aiZ = 0;
float aX[BUFFERSIZE], aY[BUFFERSIZE], aZ[BUFFERSIZE];
int biX = 0, biY = 0, biZ = 0;
int acRaw;
int filenumber = 1;
String filename;

float giroVar = 0.1;
float deltaGiroVar = 0.1;
float accelVar = 5;
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float kx, kv;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
float AcXf,AcYf,AcZf,GyXf,GyYf,GyZf;
float offset_AcX,offset_AcY,offset_AcZ,offset_GyX,offset_GyY,offset_GyZ;
int initCount = 0;
bool useSd = true;

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

void SetServo1(int degree) {
  servo1.write(90 + (90 - (degree)) + servo1Adjust);
}

void SetServo2(int degree) {
  servo2.write(90 + (90 - (degree)) + servo2Adjust);
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

  //Set options on sensor
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1A);
  Wire.write(0x4);
  Wire.endTransmission();
  
  /*Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);
  Wire.write(0x8);
  Wire.endTransmission();*/
  
  Serial.println("Init servo 1...");
  InitialiseServo(3, servo1, 90 + servo1Adjust);
  Serial.println("Init servo 2...");
  InitialiseServo(5, servo2, 90 + servo2Adjust);
  
  Serial.println("Init gyro...");
  InitialiseGyro();

  Serial.println("Init SD card...");
  if (!SD.begin(4)) {
    Serial.println("NO SD!");
    useSd = false;
  } else {
    filename = String(filenumber) + ".log";
    while (SD.exists(filename.c_str())) {
      filename = String(++filenumber) + ".log";
    }
    Serial.println("Filename: " + filename);
  }
  
  Serial.println("Init file...");
  myFile = SD.open(filename.c_str(), FILE_WRITE);
  myFile.print("time");
  myFile.print("\t");
  myFile.print("ptch");
  myFile.print("\t");
  myFile.println("roll");
  myFile.flush();
  myFile.close();
  myFile.close();

  Serial.println("Init timer...");

 // attachInterrupt(digitalPinToInterrupt(2), TakeValues, CHANGE);
  MsTimer2::set(7, TakeValues);
  MsTimer2::start();

  Serial.println("Init done.");
}

float MoveAverage(float* average, float value, int &pos) {
  average[pos++] = value;
  if (pos == BUFFERSIZE) {
    pos = 0;
  }
  float result = 0;
  int divisor = 0;
  for (int i = 0; i < BUFFERSIZE; i++) {
    result += average[i];
    if (average[i] != 0) {
      divisor++;
    }
  }

  if (divisor != 0)
    return result / divisor;
  else
    return result;
}

int mils = 0, prevMils = 0;
float zScaleFactor = 0.3;
float resultScaleFactor = 0.03;

void TakeValues() {
  sei();
  if (prevMils != 0)
    mils = millis() - prevMils;
  
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Wire.read();Wire.read();
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)  

  acRaw = AcX;
  
  GyXf = (float)GyX / GYROSCOPE_SENSITIVITY;
  GyYf = (float)GyY / GYROSCOPE_SENSITIVITY;
  GyZf = (float)GyZ / GYROSCOPE_SENSITIVITY;
  AcXf = (float)AcX / ACCELEROMETER_SENSITIVITY;
  AcYf = (float)AcY / ACCELEROMETER_SENSITIVITY;
  AcZf = (float)AcZ / ACCELEROMETER_SENSITIVITY;

  //GyXf = GyXf + (GyZf * zScaleFactor);
  //GyYf = GyYf + (-GyZf * zScaleFactor);

  if (smoothing) {
    AcXf = MoveAverage(aX, AcXf, aiX);
    AcYf = MoveAverage(aY, AcYf, aiY);
    AcZf = MoveAverage(aZ, AcZf, aiZ);
    GyXf = MoveAverage(gX, GyXf, giX);
    GyYf = MoveAverage(gY, GyYf, giY);
    GyZf = MoveAverage(gZ, GyZf, giZ);
  }
  
  float adjustment = 0;
  if (INITTING) {
    offset_GyX += (GyXf - adjustment);
    offset_GyY += (GyYf - adjustment);
    offset_GyZ += (GyZf - adjustment);  
    offset_AcX += (AcXf - adjustment);
    offset_AcY += (AcYf - adjustment);
    offset_AcZ += (AcZf - adjustment - 1); //ignore gravity - 1.0003
    initCount++;
    if (initCount > 100) {
      if (useSd) {
        SetRedLed(false);
      }
      INITTING = false;
      offset_GyX /= (float)initCount;
      offset_GyY /= (float)initCount;
      offset_GyZ /= (float)initCount;
      offset_AcX /= (float)initCount;
      offset_AcY /= (float)initCount;    
      offset_AcZ /= (float)initCount;   
    }
  } else {
    float accelAdjust = 0.06;
    float gyroAdjust = 1 - accelAdjust;

    pitchAccel = atan2((AcYf - offset_AcY), (AcZf - offset_AcZ)) * 360.0 / (2*PI);
    pitchGyro = pitchGyro + ((GyXf - offset_GyX)) * dt;
    pitchValue = pitchValue + pitchGyro;
    
    rollAccel = atan2((AcXf - offset_AcX), (AcZf - offset_AcZ)) * 360.0 / (2*PI);
    rollGyro = rollGyro + ((GyYf - offset_GyY)) * dt; 
    rollValue = rollValue + rollGyro;
    
    Pxx += dt * (2 * Pxv + dt * Pvv);
    Pxv += dt * Pvv;
    Pxx += dt * giroVar;
    Pvv += dt * deltaGiroVar;
    kx = Pxx * (1 / (Pxx + accelVar));
    kv = Pxv * (1 / (Pxx + accelVar));
    
    pitchValue += (pitchAccel - pitchValue) * kx;
    rollValue += (rollAccel - rollValue) * kx;
    
    Pxx *= (1 - kx);
    Pxv *= (1 - kx);
    Pvv -= kv * Pxv;

    pv = (-(pitchValue * 0.03));
    rv = ((rollValue * 0.03));
    pv *= 1.1;
    rv *= 1.1;
    pv += 90.0;
    rv += 90.0;

    if (pv >= 50 + servo1Adjust && pv <= 130 + servo1Adjust) {
      SetServo1(pv);
    } else if (pv <= 130 + servo1Adjust) {
      if (pv <= 30 + servo1Adjust) {
        SetServo1(90);
        SetServo2(90);
        MsTimer2::stop();
        prevMils = millis();
        return;
      } else {
        SetServo1(50);
      }
    } else {
      if (pv >= 150 + servo1Adjust) {
        SetServo1(90);
        SetServo2(90);
        MsTimer2::stop();
        prevMils = millis();
        return;
      } else {
        SetServo1(130);
      }
    }
  
    if (rv >= 50 + servo2Adjust && rv <= 130 + servo2Adjust) {
      SetServo2(rv);
    } else if (rv <= 130 + servo2Adjust) {
      if (rv <= 30 + servo2Adjust) {
        SetServo1(90);
        SetServo2(90);
        MsTimer2::stop();
        prevMils = millis();
        return;
      } else {
        SetServo2(50);
      }
    } else {
      if (rv >= 150 + servo2Adjust) {
        SetServo1(90);
        SetServo2(90);
        MsTimer2::stop();
        prevMils = millis();
        return;
      } else {
        SetServo2(130);
      }
    }

    prevMils = millis();    
    /*Calculate(angleX, GyXf, offset_GyX, velocityX, AcXf, offset_AcX, AcZf, offset_AcZ, resultX, gyroAdjust, accelAdjust);
    Calculate(angleY, GyYf, offset_GyY, velocityY, AcYf, offset_AcY, AcZf, offset_AcZ, resultY, gyroAdjust, accelAdjust);*/
  }
}

void Calculate(float &angle, float Gy, float offset_Gy, float &velocity, float AcAxis, float offset_AcAxis, float AcOffAxis, float offset_AcOffAxis, float &result, float gyroAdjust, float accelAdjust) {
  float angleDelta = angle = (Gy - offset_Gy) * dt;
  float accelDelta = angle = atan2(((AcAxis - offset_AcAxis)), ((AcOffAxis - offset_AcOffAxis))) * 360.0 / (2*PI); //(Ac - offset_Ac) * dt;
  result = (gyroAdjust * (result + angleDelta)) + (accelAdjust * accelDelta);
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
int count = 8;

void loop(){
  if (INITTING) {
    Serial.print(AcZ);
  } else {    
    ledState = !ledState;
    SetGreenLed(ledState);
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(pv);
    Serial.print("\t");
    Serial.println(rv);
   
    
    if (useSd) {
      myFile = SD.open(filename.c_str(), FILE_WRITE);      
      myFile.print(millis());
      myFile.print("\t");
      myFile.print(pv);
      myFile.print("\t");
      myFile.println(rv);
      myFile.flush();
      myFile.close();
    }
    /*Serial.print(millis());
    Serial.print("\t");
    Serial.print(AcXf + offset_AcX);
    Serial.print("\t");
    Serial.print(AcYf + offset_AcY);
    Serial.print("\t");
    Serial.print(AcZf + offset_AcZ);
    Serial.print("\t");

    Serial.print(GyXf + offset_GyX);
    Serial.print("\t");
    Serial.print(GyYf + offset_GyY);
    Serial.print("\t");
    Serial.print(GyZf + offset_GyZ);
    Serial.print("\t");

    Serial.print(pv);
    Serial.print("\t");
    Serial.print(rv);
    Serial.print("\t");

    Serial.print("\n");*/
  }
  delay(100);
}
