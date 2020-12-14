
#include "vectorTypes.h"
#include "receiver.h"
#include "debug.h"

//#include "NAxisMotion.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//NAxisMotion mySensor;         //Object that for the sensor
Adafruit_BNO055 bno = Adafruit_BNO055();

unsigned long LastOutTime = 0;     // Output - Pwm to Motors, 
const int OutPeriod = 5;          // 200Hz = 5ms

unsigned long LastCmdTime = 0;     // Commands - (Reciver)
const int CmdPeriod = 20;          // 50Hz = 20ms


unsigned long LastIMUTime = 0;     // IMU
const int IMUPeriod = 10;          // 100Hz = 10ms


int potpin = 0;
float pot;
// raw clipped receiver input
float Roll_cmd;
float Pitch_cmd;
float yaw_rx;
float thrust_rx;
float aux0_rx;
float aux1_rx;

float tau_yaw, Yaw_old, Yaw_cmd, tau_yawdt;
float tau_roll, tau_rolldt;
float tau_pitch, tau_pitchdt;

float cal_yaw, cal_roll, cal_pitch;
int cal_int;
int thrust;
int state = 0;
int motor1Pin = 6, motor2Pin = 9 ,motor3Pin = 7 ,motor4Pin = 8; 
int Motor1PWM,Motor2PWM,Motor3PWM,Motor4PWM;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PID:
float Pu,Ru,Yu,T;
//   PITCH:
float  kpp = 0.7, kip = 0.00, kdp = 0.175;
//   ROLL:
int  kpr = 1, kir = 0.001, kdr = 0.1;
//   YAW:
int  kpy = 1, kiy = 0.001, kdy = 0.1;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
Serial.begin(115200);
  //Receiver.h has a namespace can we run the init?
  receiverModule::init(); ////Not entirely sure why KyrellGod uses a namespace instead of a class
  delay(250); ////but head over to receiver.cpp to see the namespace functions
  receiverModule::resetLimitStats();

pinMode(motor1Pin,OUTPUT);
pinMode(motor2Pin,OUTPUT);
pinMode(motor3Pin,OUTPUT);
pinMode(motor4Pin,OUTPUT);
analogWriteResolution(12);
delay(5);
analogWrite(motor1Pin,1000);
analogWrite(motor2Pin,1000);
analogWrite(motor3Pin,1000);
analogWrite(motor4Pin,1000);
delay(500);
  LastCmdTime = millis();
  LastOutTime = millis();
  
//  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
//  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
//  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
//  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
                                  //Setting to MANUAL requires fewer reads to the sensor  
/*
    if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  Yaw_old = 0;
  */
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{
    if ((millis() - LastCmdTime) >= CmdPeriod)
    {
         LastCmdTime = millis();  
         readReceiver();  
    }

    if ((millis() - LastIMUTime) >= IMUPeriod)
    {
         LastIMUTime = millis();
         readIMU();
    }
    
    if ((millis() - LastOutTime) >= OutPeriod)
    {
         LastOutTime = millis(); 
         callStablizer();
        // callMotors();  //This should test the motors!
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void readReceiver(){
  Pitch_cmd = map(receiverModule::getClippedChannelValue_f(THROTTLE),1131,1906,20,-20);//,1100,1900,-20,20);
  Roll_cmd = map(receiverModule::getClippedChannelValue_f(ELEVATOR),1135,1912,20,-20);
  yaw_rx = map(receiverModule::getClippedChannelValue_f(RUDDER),1135,1900,20,-20);
  thrust_rx = map(receiverModule::getClippedChannelValue_f(AILERON),1130,1910,800,1700); //PWM CORRECTION
  aux0_rx = receiverModule::getClippedChannelValue_f(AUX0);
  aux1_rx = receiverModule::getClippedChannelValue_f(AUX1);
  
//  Yaw_cmd = Yaw_old + (yaw_rx * 0.1);
//  if (Yaw_cmd > 360){
//      Yaw_old = Yaw_cmd - 360;
//  }
//  else if(Yaw_cmd < 0){
//      Yaw_old = Yaw_cmd + 360;
//  }
//  else{
//      Yaw_old = Yaw_cmd;
//  }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*         //DEBUG Reciver:
           Serial.print("Time: ");
           Serial.print(LastOutTime);
           Serial.print("us ");

           Serial.print("\t T: ");
           Serial.print(thrust_rx);
           Serial.print("us ");

           Serial.print("\t Y: ");
           Serial.print(yaw_rx); //Yaw data
           Serial.print("deg ");
    
           Serial.print("\t R: ");
           Serial.print(Roll_cmd); //Roll data
           Serial.print("deg ");
    
           Serial.print("\t P: ");
           Serial.print(Pitch_cmd); //Pitch data
           Serial.print("deg");
           Serial.println();
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void readIMU(){
  
  imu::Quaternion q = bno.getQuat();
  q.normalize();
  float temp = q.x();  q.x() = -q.y();  q.y() = temp;
  q.z() = -q.z();
  
  imu::Vector<3> euler = q.toEuler();
  tau_roll = 180/M_PI * euler.z();
  tau_pitch = -180/M_PI * euler.y();
  tau_yaw = 180/M_PI * euler.x();
  Serial.print("Roll: ");
  Serial.print(180/M_PI * euler.z());  // pitch, nose-down is positive, x-axis points right
  Serial.print("\t Pitch: ");
  Serial.print(-180/M_PI * euler.y());
  Serial.print("\t Yaw: ");
  Serial.print(180/M_PI * euler.x());
  

  imu::Vector<3> rate = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  tau_rolldt = -rate.y();
  tau_pitchdt = -rate.x();
  tau_yawdt = -rate.z();
  Serial.print("\t Roll Rate: ");
  Serial.print(-rate.y());
  Serial.print("\t Pitch Rate: ");
  Serial.print(-rate.x());
  Serial.print("\t Yaw Rate: ");
  Serial.println(-rate.z());
  
//  mySensor.updateEuler();
//  mySensor.updateGyro();
//  mySensor.updateCalibStatus();   
//
//  tau_yaw = mySensor.readEulerHeading();
//  tau_roll = mySensor.readEulerRoll();
//  tau_pitch = mySensor.readEulerPitch();
//  tau_yawdt = mySensor.readGyroY();
//  tau_rolldt = mySensor.readGyroX();
//  tau_pitchdt = mySensor.readGyroY();

//  if (tau = 360){
//      Yaw_old = Yaw_cmd - 360;
//  }
//  else if(Yaw_cmd < 0){
//      Yaw_old = Yaw_cmd + 360;
//  }
//  else{
//      Yaw_old = Yaw_cmd;
//  }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*         //DEBUG IMU:
           Serial.print("Time: ");
           Serial.print(LastOutTime);
           Serial.print("us: ");

           Serial.print("\t Y: ");
           Serial.print(tau_yaw); //Yaw data
           Serial.print("deg ");
    
           Serial.print("\t R: ");
           Serial.print(tau_roll); //Roll data
           Serial.print("deg");
    
           Serial.print("\t tau_P: ");
           Serial.print(tau_pitch); //Pitch data
           Serial.print("deg ");
           
           Serial.print("\t Y_cmd= ");
           Serial.print(Yaw_cmd);
           Serial.println();
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void callStablizer(){
  
    Pu = kpp*(Pitch_cmd - tau_pitch) + kdp*(tau_pitchdt);
    Ru = 0;//kpr*(Roll_cmd - tau_roll) + kdr*(tau_rolldt);
    Yu = 0;//kpy*(Yaw_cmd - tau_yaw) + kdy*(tau_yawdt);
//    Serial.print("IMU_Yaw: ");
//    Serial.print(tau_yaw);
//  Serial.print("\t Pitch_Cmd: ");
//  Serial.print(receiverModule::getClippedChannelValue_f(THROTTLE));
  //Serial.print("\t ");////////////////////////////////////////////////////////////////////////////////
  //Serial.print(Pitch_cmd);
  //Serial.print("\t ");
  //Serial.println(tau_pitch);//////////////////////////////////////////////////////////////////////////
//  Serial.print("\t Pitch Rate: ");
//  Serial.print(tau_pitchdt);
//  Serial.print("\t Pu: ");
//  Serial.println(Pu);
//    Serial.print("\t Ru: ");
//    Serial.print(Ru);
//    Serial.print("\t Yu: ");
//    Serial.print(Yu);
//    Serial.println();

    Motor1PWM = thrust_rx+Pu;//+Ru+Yu;
    Motor2PWM = thrust_rx-Pu;//+Ru-Yu;
    Motor3PWM = thrust_rx-Pu;//-Ru+Yu;
    Motor4PWM = thrust_rx+Pu;//-Ru-Yu;

//    Serial.print("\t thrust_rx: ");
//    Serial.print(thrust_rx);
//    Serial.print("\t M2: ");
//    Serial.print(Motor2PWM);
//    Serial.print("\t M3: ");
//    Serial.print(Motor3PWM);
//    Serial.print("\t M4: ");
//    Serial.print(Motor4PWM);
//    Serial.println();
    
    
    analogWrite(motor1Pin,Motor1PWM);
    analogWrite(motor2Pin,Motor2PWM);
    analogWrite(motor3Pin,Motor3PWM);
    analogWrite(motor4Pin,Motor4PWM);
//    analogWrite(motor1Pin,thrust_rx);
//    analogWrite(motor2Pin,thrust_rx);
//    analogWrite(motor3Pin,thrust_rx);
//    analogWrite(motor4Pin,thrust_rx);
}

void callMotors(){
//     Serial.print("Motor1: ");
//     Serial.print(Motor1PWM);
//     Serial.print("\t Motor2: ");
//     Serial.print(Motor2PWM);
//     Serial.print("\t Motor3: ");
//     Serial.print(Motor3PWM);
//     Serial.print("\t Motor4: ");
//     Serial.print(Motor4PWM);
//     Serial.println();
//     
//     analogWrite(motor1Pin,Motor1PWM);
//     analogWrite(motor2Pin,Motor2PWM);
//     analogWrite(motor3Pin,Motor3PWM);
//     analogWrite(motor4Pin,Motor4PWM);
}
