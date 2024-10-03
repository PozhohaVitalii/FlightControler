#include <Wire.h>
#include <Arduino.h>
#include <DShotRMT.h>
#include <math.h>
#include <stdio.h>
#include <VL53L1X.h>

// STATE
bool ARMED = 1;

//I2C values
//pins
#define SCL 36
#define SDA 37
//addresses
#define PFER_ADDRESS1 0x68
#define PFER_ADDRESS2 0x77
#define PFER_ADDRESS3 0x29

//DShot setings 
//pins
#define LED_PIN GPIO_NUM_7
#define RTRCTL0 GPIO_NUM_11
#define RTRCTL1 GPIO_NUM_12
#define RTRCTL2 GPIO_NUM_13
#define RTRCTL3 GPIO_NUM_14
//chanels
#define RTRCTL_RMT_CHNL0 RMT_CHANNEL_0
#define RTRCTL_RMT_CHNL1 RMT_CHANNEL_1
#define RTRCTL_RMT_CHNL2 RMT_CHANNEL_2
#define RTRCTL_RMT_CHNL3 RMT_CHANNEL_3


//DShot variables
DShotRMT dshot0(RTRCTL0, RTRCTL_RMT_CHNL0);
DShotRMT dshot1(RTRCTL1, RTRCTL_RMT_CHNL1);
DShotRMT dshot2(RTRCTL2, RTRCTL_RMT_CHNL2);
DShotRMT dshot3(RTRCTL3, RTRCTL_RMT_CHNL3);

//Throttle 
uint16_t throttle[4] ;

//Time when program start
unsigned long start_ms = 0;

//Orientation values 
//GYRO
//raw
float rateRoll, ratePitch, rateYaw;
//calibration values
float rollCalibration, pitchCalibration, yawCalibration;
int rateCalibrationNumber;
//Accelerometer
float AccX, AccY, AccZ;
float angleRoll, anglePitch;

//Kalman filter
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2*2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2*2;
float Kalman1DOutput[] = {0,0};

//BARO

//Presure
long Presure = 0;
long Temperature = 0;
//BARO calibration values 
short AC1, AC2, AC3;
unsigned short AC4, AC5, AC6;
short _B1, _B2;
short MB, MC, MD;
long rawPresure = 0;
long rawTemperature = 0;
bool itsRightSequation[4]{
  false,
  false,
  false,
  false
};
// Atitude variables
long AtitudeMm[]{0,0,0};
VL53L1X TOF400f;


// PID by Rate //
float DesiredRatePitch;
float DesiredRateRoll;
float DesiredRateYaw;
float InputRoll, InputPitch, InputYaw, InputThrottle = 0;
float PIDReturn[] = {0,0,0};
///  rate   ///////  rate  /////
///  rate   P  I  D  rate  /////
///  rate   ///////  rate  /////
//P - param configuration
float PRateRoll = 0.6;
float PRatePitch = 0.6;
float PRateYaw = 2;
// I - param configuration
float IRateRoll = 3.5;
float IRatePitch = 3.5;
float IRateYaw = 12;
// D - param configuration 
float DRateRoll = 0.03;
float DRatePitch = 0.03;
float DRateYaw = 0;
///  end    ///////  end   /////
///   end   ///////   end  /////
float MotorInput1 = 0, MotorInput2 = 0, MotorInput3 = 0, MotorInput4 = 0;
// PID by Angle
float DesiredAtitude;
float DesiredPitch;
float DesiredRoll;
float DesiredYaw;
float ErrorRoll, ErrorPitch, ErrorYaw, ErrorAtitude;
//float InputRoll, InputPitch, InputYaw;
float PrevErrorRoll, PrevErrorPitch, PrevErrorYaw, PrevErrorAtitude;
float PrevItermRoll, PrevItermPitch, PrevItermYaw, PrevItermAtitude;
///  angle   ///////  angle  /////
///  angle   P  I  D  angle  /////
///  angle   ///////  angle  /////
//P - param configuration
float PRoll = 0.6;
float PPitch = 0.6;
float PYaw = 2;
float PAtitude = 0.02;
// I - param configuration
float IRoll = 3.5;
float IPitch = 3.5;
float IYaw = 12;
float IAtitude = 3.5;
// D - param configuration 
float DRoll = 0.03;
float DPitch = 0.03;
float DYaw = 0;
float DAtitude = 0.02;
///  end    ///////  end   /////
///   end   ///////   end  /////


//Orientation functions
 void upDate_GYRO(){
  //DigitalLowPass Filter 10Hz
  Wire.beginTransmission(PFER_ADDRESS1);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  //Sensivity scale factor 65.5LSB/*/second
  Wire.beginTransmission(PFER_ADDRESS1);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  //Register to start reading from
  Wire.beginTransmission(PFER_ADDRESS1);
  Wire.write(0x43);
  Wire.endTransmission();
  // READING from Module
  //request 6 times by 8 bits from Gyro
  Wire.requestFrom(PFER_ADDRESS1,6);
  int16_t GyroX = Wire.read()<<8 | Wire.read();
   int16_t GyroY = Wire.read()<<8 | Wire.read();
    int16_t GyroZ = Wire.read()<<8 | Wire.read();
  //convert LSB to digrees per second
  rateRoll = (float)GyroX/65.5;
   ratePitch = (float)GyroY/65.5;
    rateYaw = (float)GyroZ/65.5;

  // ACCELEROMETER 8g range
  Wire.beginTransmission(PFER_ADDRESS1);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  //Register to start reading from
  Wire.beginTransmission(PFER_ADDRESS1);
  Wire.write(0x3B);
  Wire.endTransmission();
  // READING from Module
  //request 6 times(bytes) by 8 bits from ACCEL
  Wire.requestFrom(PFER_ADDRESS1,6);
  int16_t AccXlsb = Wire.read()<<8 | Wire.read();
   int16_t AccYlsb = Wire.read()<<8 | Wire.read();
    int16_t AccZlsb = Wire.read()<<8 | Wire.read();
  //lsb to g(9.8 m/s)
  AccX = (float) AccXlsb/4096;
   AccY = (float) AccYlsb/4096;
    AccZ = (float) AccZlsb/4096;
   //Calibration ACCEL
    AccX -= 0.02;
  //  AccY += 0.03;
    AccZ -= 0.31;
  //Calculation of angles 
  angleRoll = atan(AccY / sqrt(AccX*AccX + AccZ*AccZ))*1/(3.142/180);
  anglePitch = -atan(AccX / sqrt(AccY*AccY + AccZ*AccZ))*1/(3.142/180);
 }
 void upDate_BARO(bool isRequestFlag, bool tempORpres) { 
  //precision mode
  int mode = 0;
  if (isRequestFlag && !tempORpres){
    Wire.beginTransmission(PFER_ADDRESS2);
    Wire.write(0xF4);
    Wire.write(0x2E);
    Wire.endTransmission();
    // right sequence implementation
    itsRightSequation[0]=true;}
  else if (isRequestFlag && tempORpres){
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xF4);
  //if (mode==0)Wire.write(0x34);
  Wire.write(0x34);
  //else Serial.println("\t  Chenge precision mode on Baro ");
  Wire.endTransmission();
  // right sequence implementation
  if (itsRightSequation[1]) itsRightSequation[2]=true;
  else Serial.println("\t  Error sequence  ReqPres is caled");
  }
  else if(!isRequestFlag && !tempORpres){
  //here we request transmition of raw presure data from 3 registers (8bits every)
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2, 2);
  
  rawTemperature = Wire.read()<<8|Wire.read();

  // right sequence implementation
  if (itsRightSequation[0])itsRightSequation[1]=true;
  else Serial.println("\t  Error sequence  GetTemp is caled");}
  else if(!isRequestFlag && tempORpres){
  //here we request transmition of raw presure data from 3 registers (8bits every)
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xF6);
  Wire.endTransmission(false);
  Wire.requestFrom(PFER_ADDRESS2, 3);
  
  rawPresure = (Wire.read()<<16|Wire.read()<<8|Wire.read())>>(8-mode);
  // right sequence implementation
  if (itsRightSequation[2])itsRightSequation[3]=true;
  else Serial.println("\t  Error sequence  GetPres is caled");}
 }

 void calculateRTemp() {
  long X1;
  long X2;
  long B5;
  X1 = (rawTemperature * AC6)*AC5 / pow(2,15);
  X2 = MC * pow(2,11)/(X1+MD);
  B5 = X1 + X2;
  Temperature = (B5 + 8) / pow(2,4);
 }
 void calculateRPress() {
  short mode = 0;
  // local calculation variables
  long X1;
  long X2;
  long X3;
  long B3;
  unsigned long B4;
  long B5;
  long B6;
  long B7;
    // calculation for final presure 
  X1 = (rawTemperature * AC6)*AC5/pow(2,15);
  X2 = MC * pow(2,11)/(X1+MD);
  B5 = X1 + X2;
  B6 = B5 - 4000;
  X1 = (_B2* (B6 * B6 / pow(2,12) )) / pow(2,11);
  X2 = AC2 * B6 / pow(2,11);
  X3 = X1 + X2;
  B3 = ((AC1 * 4 + X3)<<mode + 2) / 4;
  X1 = AC3 * B6 / pow(2,13);
  X2 = (_B1 * (B6 * B6 / pow(2,12))) / pow(2,16); 
  X3 = ((X1 + X2) + 2 ) / pow(2,2);
  B4 = AC4 * (unsigned long)(X3 + 32768) / pow(2,15);
  B7 = ((unsigned long)rawPresure - B3) * (50000 >> mode);
  if (B7 < 0x80000000){
    Presure  = (B7 * 2) / B4;
  }else{
    Presure = (B7 / B4) * 2;
  }
  X1 = (Presure / pow(2,8))*(Presure / pow(2,8));
  X1 = (X1 * 3038) / pow(2,16);
  X2 = (-7357 * Presure) / pow(2,16);
  Presure = Presure + (X1 + X2 + 3791) / pow(2,4);
 }





//Kalman function
 void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.01*KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.01 * 0.01 * 4 * 4;
  float KalmanGain = KalmanUncertainty * 1 / ( 1 * KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty; 

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
 }

// PID functions
  void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {


  float Pterm = P * Error;
  float Iterm = PrevIterm + I * (Error + PrevError) * 0.004/2;
  if (Iterm > 400) Iterm = 400;
  else if ( Iterm < -400) Iterm = -400;
  float Dterm = D * (Error - PrevError)/0.004;
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput>400)PIDOutput  = 400;
  else if (PIDOutput < -400) PIDOutput = -400;
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
  }
  void rset_pid(void) {
 PrevErrorRoll = 0; PrevErrorPitch = 0; PrevErrorYaw = 0;
 PrevItermRoll = 0; PrevItermPitch = 0; PrevItermYaw = 0;
  
  }

void setup() {
//Serial start  
  Serial.begin(115200);
 
  dshot0.begin(DSHOT300,false);
  dshot1.begin(DSHOT300,false);
  dshot2.begin(DSHOT300,false);
  dshot3.begin(DSHOT300,false);
  dshot0.sendThrottleValue(0);
  dshot1.sendThrottleValue(0);
  dshot2.sendThrottleValue(0);
  dshot3.sendThrottleValue(0); 
//Wire settup
  Wire.setClock(400000);
  Wire.begin(SDA,SCL);
  delay(250);
//enabling MPU6050
  Wire.beginTransmission(PFER_ADDRESS1);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
//reading Baro calibration data
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xE0);
  Wire.write(0xB6);
  Wire.endTransmission();
  delay(300);
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xAA);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2,2);
  AC1 = Wire.read()<<8|Wire.read(); 
  delay(60);
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xAC);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2,2);
  AC2 = Wire.read()<<8|Wire.read(); 
  delay(60);
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xAE);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2, 2);
  AC3 = Wire.read()<<8|Wire.read();
  delay(60);
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xB0);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2, 2);
  AC4 = Wire.read()<<8|Wire.read();
  delay(60);
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xB2);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2, 2);
  AC5 = Wire.read()<<8|Wire.read(); 
   delay(60); 
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xB4);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2, 2);
  AC6 = Wire.read()<<8|Wire.read();
   delay(60);
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xB6);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2, 2);
  _B1 = Wire.read()<<8|Wire.read();
   delay(60);
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xB8);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2, 2);
  _B2 = Wire.read()<<8|Wire.read();
   delay(60);
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xBA);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2, 2);
  MB = Wire.read()<<8|Wire.read();
   delay(60);
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xBC);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2, 2);
  MC = Wire.read()<<8|Wire.read();
   delay(60);
  Wire.beginTransmission(PFER_ADDRESS2);
  Wire.write(0xBE);
  Wire.endTransmission();
  Wire.requestFrom(PFER_ADDRESS2, 2);
  MD = Wire.read()<<8|Wire.read();
   delay(60);
//calibrating Gyro   
  for (rateCalibrationNumber = 0; rateCalibrationNumber < 2000; rateCalibrationNumber++){
    upDate_GYRO();
    
    rollCalibration += rateRoll;
     pitchCalibration += ratePitch;
      yawCalibration += rateYaw;



    delay(1);
  }
  rollCalibration /= 2000;
    pitchCalibration /= 2000;
      yawCalibration /= 2000;


//enabling TOF400f
  TOF400f.setAddress(PFER_ADDRESS3);
  TOF400f.setTimeout(500);
  if (!TOF400f.init()) {
    Serial.println("Failed to detect and initialize sensor!");
  }
  TOF400f.setDistanceMode(VL53L1X::Long);
  TOF400f.setMeasurementTimingBudget(50000);
  TOF400f.startContinuous(50);
//end
  dshot0.sendThrottleValue(0);
  dshot1.sendThrottleValue(0);
  dshot2.sendThrottleValue(0);
  dshot3.sendThrottleValue(0);
  


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);


  start_ms = millis();
}

void loop() {
  const auto now = millis();
  
  // Baro calling
    if((now % 100 - now % 10)/10 >= 0 && (now % 100 - now % 10)/10 < 2 && !itsRightSequation[0]){
        upDate_BARO(true,false);
    } 
    else  if((now % 100 - now % 10)/10 >= 2 && (now % 100 - now % 10)/10 < 4 && !itsRightSequation[1]){
        upDate_BARO(false,false);
    }
    else  if((now % 100 - now % 10)/10 >= 4 && (now % 100 - now % 10)/10 < 6 && !itsRightSequation[2]){
        upDate_BARO(true,true);
    }
    else  if((now % 100 - now % 10)/10 >= 6 && (now % 100 - now % 10)/10 < 8 && !itsRightSequation[3]){
        upDate_BARO(false,true);
       
    }   
    else  if((now % 100 - now % 10)/10 >= 8 && (now % 100 - now % 10)/10 <=9  && itsRightSequation[3]){
        itsRightSequation[0] = false;
        itsRightSequation[1] = false;        
        itsRightSequation[2] = false;
        itsRightSequation[3] = false;
    }
    
  
  

 

  //Unarmed mode
  if (now < 10000 || !ARMED ){
    dshot0.sendThrottleValue(0);
    dshot1.sendThrottleValue(0);
    dshot2.sendThrottleValue(0);
    dshot3.sendThrottleValue(0);
  }else{

    throttle[0] = (uint16_t) MotorInput1;
    throttle[1] = (uint16_t) MotorInput2;
    throttle[2] = (uint16_t) MotorInput3;
    throttle[3] = (uint16_t) MotorInput4;

  dshot0.sendThrottleValue(throttle[3]);// motor 1 //checkd
  dshot2.sendThrottleValue(throttle[0]);// motor 2 //checkd
  dshot3.sendThrottleValue(throttle[1]);// motor 3 //checkd
  dshot1.sendThrottleValue(throttle[2]);// motor 4 //checkd
  ///MOTTORS///
  //Dch///mot//
  ///0/////1///
  ///2/////2///
  ///3/////3///
  ///1/////4///

  //Atitude mesurement calling 
  if (now % 50 < 10){

    AtitudeMm[0] = AtitudeMm[1];
    AtitudeMm[1] = AtitudeMm[2];
    AtitudeMm[2] = (long)TOF400f.read(false);
  DesiredAtitude = 750 ;
  ErrorAtitude = DesiredAtitude - ((AtitudeMm[0] + AtitudeMm[1] + AtitudeMm[2])/3);
  pid_equation(ErrorAtitude, PAtitude, IAtitude, DAtitude, PrevErrorAtitude, PrevItermAtitude);
       InputThrottle=PIDReturn[0];
       PrevErrorAtitude=PIDReturn[1]; 
       PrevItermAtitude=PIDReturn[2];
  
  }
  //Gyro 
  if (now % 4 == 0 ){
    upDate_GYRO();
    rateRoll -= rollCalibration;
    ratePitch -= pitchCalibration;
    rateYaw -= yawCalibration;
  }

  // Kalman calling for Roll
  kalman_1d(KalmanAngleRoll,KalmanUncertaintyAngleRoll,rateRoll,angleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  // Kalman calling for Pitch
 kalman_1d(KalmanAnglePitch,KalmanUncertaintyAnglePitch,ratePitch,anglePitch);
 KalmanAnglePitch = Kalman1DOutput[0];
 KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

if (fabs(KalmanAngleRoll) > 40 || fabs(KalmanAnglePitch) > 40){
 ARMED = false;
}



//PID settings
 DesiredPitch = 0.55 ;
 DesiredRoll = 0.2 ;
 DesiredYaw = 0 ;
 ErrorRoll = DesiredRoll - KalmanAngleRoll;
 ErrorPitch = DesiredPitch - KalmanAnglePitch;
 ErrorYaw = 0 ;

  pid_equation(ErrorRoll, PRoll, IRoll, DRoll, 0, PrevItermRoll);
       InputRoll=PIDReturn[0];
       PrevErrorRoll=PIDReturn[1]; 
       PrevItermRoll=PIDReturn[2];
  pid_equation(ErrorPitch, PPitch, IPitch, DPitch, 0, PrevItermPitch);
       InputPitch=PIDReturn[0]; 
       PrevErrorPitch=PIDReturn[1]; 
       PrevItermPitch=PIDReturn[2];
  pid_equation(ErrorYaw, PYaw, IYaw, DYaw, 0, PrevItermYaw);
       InputYaw=PIDReturn[0]; 
       PrevErrorYaw=PIDReturn[1]; 
       PrevItermYaw=PIDReturn[2];
  
  MotorInput1 = 1.024*(InputThrottle-InputRoll-InputPitch-InputYaw);
  MotorInput2 = 1.024*(InputThrottle-InputRoll+InputPitch+InputYaw);
  MotorInput3 = 1.024*(InputThrottle+InputRoll+InputPitch-InputYaw);
  MotorInput4 = 1.024*(InputThrottle+InputRoll-InputPitch+InputYaw);


Serial.print("\t Att = ");
 Serial.print(  AtitudeMm[2]);
 Serial.print("\t KalmanAngleRoll = ");
 Serial.print(KalmanAngleRoll);
 Serial.print("\t KalmanAnglePitch = ");
 Serial.print(KalmanAnglePitch);
 Serial.print("\t M1 = ");
 Serial.print(  MotorInput1);
 Serial.print("\t M2 = ");
 Serial.print(  MotorInput2);
 Serial.print("\t M3 = ");
 Serial.print(  MotorInput3);
 Serial.print("\t M4 = ");
 Serial.print(  MotorInput4);
 Serial.print("\t Trottle = ");
 Serial.print(  InputThrottle);
 
 Serial.print("\t now = ");
 Serial.println(  now );
 //calculateRPress();
// Serial.print("\t Press = ");
 //Serial.print(  Presure );
 //calculateRTemp();
 //Serial.print("\t Temp = ");
 //Serial.println(  rawTemperature );
if (MotorInput1>1000)MotorInput1 = 1000;
if (MotorInput2>1000)MotorInput2 = 1000;
if (MotorInput3>1000)MotorInput3 = 1000;
if (MotorInput4>1000)MotorInput4 = 1000;
if (MotorInput1<49)MotorInput1 = 49;
if (MotorInput2<49)MotorInput2 = 49;
if (MotorInput3<49)MotorInput3 = 49;
if (MotorInput4<49)MotorInput4 = 49;

 


  }
  //dshot 300us between pokets
 // delayMicroseconds(250);
}
