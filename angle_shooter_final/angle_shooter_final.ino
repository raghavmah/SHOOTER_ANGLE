#include <EncoderMotor.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <PID_v1.h>

//#define Dir1 23
//#define Dir2 14
//#define pwm_pin 12
motor mY(0, 0, 14, 23, 12, 0);
double Setpoint = 0, Input, Output;
double Kp = 9, Ki = 3,  Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
MPU6050 mpu6050(Wire);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.setTimeout(5);
  myPID.SetMode(AUTOMATIC);

  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(1);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Setpoint = getAngle();
  Serial.println(Setpoint);
  delay(1000);
}
double getAngle(){
  mpu6050.update(true);
  return mpu6050.getGyroAngleY();
}

void updateSetpoint(){
  if(Serial.available()){
    Setpoint = Serial.parseInt();
  }
}
void loop() {
  updateSetpoint();
  Input = getAngle();
  myPID.Compute();
  mY.setPWM(Output);
  Serial.println(String(Input)+","+String(Setpoint)+","+String(Output));
}
