/******************************************************************************************************************************************************

  /\\\\\\\\\\\\                                                                 /\\\\\\\\\\\\\                         /\\\        
  \/\\\////////\\\                                                              \/\\\/////////\\\                      \/\\\       
   \/\\\      \//\\\                                                             \/\\\       \/\\\                      \/\\\     
    \/\\\       \/\\\  /\\/\\\\\\\     /\\\\\     /\\/\\\\\\       /\\\\\\\\      \/\\\\\\\\\\\\\\   /\\\    /\\\        \/\\\     
     \/\\\       \/\\\ \/\\\/////\\\  /\\\////\\\  \/\\\////\\\    /\\\/////\\\    \/\\\/////////\\\ \/\\\   \/\\\   /\\\\\\\\\    
      \/\\\       \/\\\ \/\\\   \///  /\\\   \//\\\ \/\\\  \//\\\  /\\\\\\\\\\\     \/\\\       \/\\\ \/\\\   \/\\\  /\\\////\\\   
       \/\\\       \/\\\  \/\\\        \//\\\  /\\\  \/\\\   \/\\\ \//\\///////      \/\\\       \/\\\ \/\\\   \/\\\ \/\\\  \/\\\ 
        \/\\\\\\\\\\\\/   \/\\\         \///\\\\\/   \/\\\   \/\\\  \//\\\\\\\\\\     \/\\\\\\\\\\\\\/  \//\\\\\\\\\  \//\\\\\\\/\\
         \////////////     \///            \/////     \///    \///    \//////////      \/////////////     \/////////    \///////\//  v2.1

******************************************************************************************************************************************************

Welcome to The Drone Bud flight control template. This is a fully functioning drone flight control
program, but only if you use these components:
  
  Arduino101
  BMP280

However, this program is well documented and easy to understand. It's a breeze to put in your own sensors
and even completely different sensors. If the sensor you have in mind has an arduino library, your job
will be even easier. 


Thanks to http://patorjk.com/software/taag/ for that sweet font
******************************************************************************************************************************************************/

#include <Servo.h>
#include "MadgwickAHRS.h" //The Madgwick filter is also found on the arduino library explorer. 
#include "CurieIMU.h" //This library is added when the arduino 101 board is downloaded from the arduino board explorer. 
#include <stdint.h>
#include "SparkFunBME280.h"   //This library is used for the barometer

  Madgwick filter; //filter object is declared


  /******************
  Barometer Variables
  *******************/
  
  BME280 baro;
  float samples[] = {0,0,0,0,0,0,0,0,0,0};
  float sampleage[] = {0,0,0,0,0,0,0,0,0,0};
  float currentsample;


  /***********************
  Flight Control Variables
  ************************/
  
  //PID constants
  static float KPx = .5; static float KIx = 0.01; static float KDx =1.2;
  static float KPy = .5; static float KIy = 0.01; static float KDy =1.2;
  static float KPz = 20; static float KIz = 0.02; static float KDz = 0;
  static float KPup = 2; static float KIup = 0; static float KDup = 3; //PID calibration constants in all axis. 

  //Controller sensitivity constants. Decrease for increased sensitivity. 
  static float Sx = 2.5; static float Sy = 2.5; static float Sz = 30;
  
  float Ix,Iy,Iz,Iup; //The Integration component of the PID controller, which will be modified every loop. 
  float IMAXx = 5000; float IMAXy = 5000; float IMAXz = 5000; //Max integral value for PID, prevents I from going to infinity. 
  float IMAXup = 10000;
  static int throttleMAX = 2000; //2000 The max throttle makes sure that esc outputs don't exceed the max signal length.  
  static int throttleMIN = 1100; //1000 
  static float midpointup = 1500; //midpoints can be trimmed right on the controller, read them then type them in here. 
  static float midpointroll = 1500;
  static float midpointpitch = 1500;
  static float midpointyaw = 1500; 
  
  float olderrorup;
  float oldbaro;
  float olderrorz;
  float oldyaw;
  
  unsigned long looptime = 0;
  unsigned long looptimeholder = 0;
  
  long frontrightchange = 1000;
  long frontleftchange = 1000;
  long backrightchange = 1000;
  long backleftchange = 1000;
  
  Servo frontright;
  Servo frontleft;
  Servo backright;
  Servo backleft; //motor pins, where the escs are connected to. 

  static int rollpin = 8;
  volatile long startroll = 0; volatile long inputroll = 0; volatile byte stateroll = 0; //These record the variables for the receiver inputs. They record the start time of the signal,
  static int pitchpin = 10;
  volatile long startpitch = 0; volatile long inputpitch = 0; volatile byte statepitch = 0; //the length of the signal, and whether it is on. The "state_" variable records the time, and  
  static int yawpin = 11;
  volatile long startyaw = 0; volatile long inputyaw = 0; volatile byte stateyaw = 0; //is used below in the PID calculation. 
  static int throttlepin = 12;
  volatile long startthrottle = 0; volatile long inputthrottle = 0; volatile byte statethrottle = 0;
  static int onoffpin = 13;
  volatile long startonoff = 0; volatile long inputonoff = 0; volatile byte stateonoff = 0;


  bool armed = false;

  //Ultrasonic u(0, 1);
  
//Interrupt functions for the receiver. They are needed because the receiver sends its signals independent of the arduino's timer. They could come at any time.
//The microcontroller uses interrupts to get the data when it enters. 
void interrupt(){             // Declare the interrupt function
  long timenow = micros();    //create a variable that stores the current time in microseconds
  if((digitalRead(rollpin) == HIGH) && stateroll == 0){  //This checks whether a signal is coming in, and if it is, set the state to high. 
   startroll = timenow;
   stateroll = 1;
  } else if ((digitalRead(rollpin) == LOW) && stateroll == 1){ //When the signal stops, the length of the signal is recorded, which indicates the position of the stick. 
    inputroll = timenow - startroll;
    stateroll = 0;
  } 
  if((digitalRead(pitchpin) == HIGH) && statepitch == 0){ //The if statement above is repeated for every input channel. There are 6. 
   startpitch = timenow;
   statepitch = 1;
  } else if ((digitalRead(pitchpin) == LOW ) && statepitch == 1){
    inputpitch = timenow - startpitch;
    statepitch = 0;
  } 
     timenow = micros();
  if((digitalRead(yawpin) == HIGH) && stateyaw == 0){
   startyaw = timenow;
   stateyaw = 1;
  } else if ((digitalRead(yawpin) == LOW ) && stateyaw == 1){
    inputyaw = timenow - startyaw;
    stateyaw = 0;
  } 
  if((digitalRead(throttlepin) == HIGH) && statethrottle == 0){
   startthrottle = timenow;
   statethrottle = 1;
  } else if ((digitalRead(throttlepin) == LOW ) && statethrottle == 1){
    inputthrottle = timenow - startthrottle;
    statethrottle = 0;
  }
      timenow = micros();
  if((digitalRead(onoffpin) == HIGH) && stateonoff == 0){
   startonoff = timenow;
   stateonoff = 1;
  } else if ((digitalRead(onoffpin) == LOW ) && stateonoff == 1){
    inputonoff = timenow - startonoff;
    stateonoff = 0;
  } 
}



void setup() { 
  Serial.begin(9600);
  CurieIMU.begin();
  CurieIMU.setGyroRate(400); //The gyro runs at 800 Hz. 
  CurieIMU.setAccelerometerRate(400); //So does the acceleromter.
  
  frontright.attach(5);
  frontleft.attach(3);
  backright.attach(9);
  backleft.attach(6); //attach our servo pins
  
  filter.begin(150); //The madgwick filter documentation can be found at https://www.arduino.cc/en/Tutorial/Genuino101CurieIMUOrientationVisualiser. 
  //Our loop runs at 153 hertz, so we use a filter running at 150 Hz. 
  
  CurieIMU.setAccelerometerRange(2); //The accelerometer will record values between -2g and 2g. 
  CurieIMU.setGyroRange(360); //This is the start up sequence for the onboard IMU. The documentation is found at https://www.arduino.cc/en/Reference/CurieIMU. 

  baro.settings.commInterface = I2C_MODE;
  baro.settings.I2CAddress = 0x76; //BMP280 has an I2C address of 0x76, the BME280 has an address of 0x77. 
  baro.settings.runMode = 3; //fastest sampling rate
  baro.settings.filter = 0;
  baro.settings.tempOverSample = 1;
  baro.settings.pressOverSample = 1;
  baro.settings.humidOverSample = 1;
  baro.begin(); //Baro settings. 

  pinMode(LED_BUILTIN, OUTPUT); //used to indicate armed status
  pinMode(rollpin,INPUT_PULLUP);
  pinMode(yawpin,INPUT_PULLUP);
  pinMode(pitchpin,INPUT_PULLUP);
  pinMode(throttlepin,INPUT_PULLUP);
  attachInterrupt(rollpin,interrupt,CHANGE);
  attachInterrupt(yawpin,interrupt,CHANGE);
  attachInterrupt(pitchpin,interrupt,CHANGE); //Attach the interrupt function to the receiver pins. 
  attachInterrupt(throttlepin,interrupt,CHANGE);
  attachInterrupt(onoffpin,interrupt,CHANGE);
  frontleft.writeMicroseconds(975);
  frontright.writeMicroseconds(975);
  backleft.writeMicroseconds(975);
  backright.writeMicroseconds(975); //write low
  delay(3000); //wait for the loop. 

}

// The following two functions convert the raw acceleration and gyro data into gs and degrees per second. They are taken from
// the Curie 101 documentation. 

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  float g = (gRaw * 360.0) / 32768.0;
  return g;
}

int largestindex(float a[]){
  int largest = 0;
  for(int x = 0; x < 9 ;x++){
      if(a[x] > largest)largest = a[x]; 
    }
    return(largest);
  }
  
void filterBaro(){ //very simple filter to get rid of jumpy values. Uses a running average and the standard deviation. 
   float total = 0;
  samples[largestindex(sampleage)] = currentsample;
  for(int x = 0; x < 10; x++){
     total += samples[x];
  }total /= 10;
  float stddev = 0;
  for(int x = 0; x < 10; x++){
    stddev += (samples[x] - total)*(samples[x] - total);
  }
  stddev = sqrt(stddev) / 10; //calculate the standard deviation
  if(currentsample > total + 1.5*stddev || currentsample < total - 1.5*stddev){ //if the sample for this loop is outside of 1.5σ, use the average. 
    currentsample = total;
  } total = 0;
  samples[largestindex(sampleage)] = currentsample;
  for(int x = 0; x < 10; x++){
     total += samples[x];
  }total /= 10;
  stddev = 0;
  for(int x = 0; x < 10; x++){
    stddev += (samples[x] - total)*(samples[x] - total);
  }
  stddev = sqrt(stddev) / 10; //calculate the standard deviation
  if(currentsample > total + 1.5*stddev || currentsample < total - 1.5*stddev){ //if the sample for this loop is outside of 1.5σ, use the average. 
    currentsample = total;
  } 
  }

//********************
// These functions control the PID control algorithms. There is one function per axis. 
//********************

void PIDx(float roll, float gx){ //This controls the side to side motion of the PID. 
  if(inputroll < 1000)inputroll = 1000;
  if(inputroll > 2000)inputroll = 2000;
  float ux = 0;
  int targetanglex;
  if(inputroll < (midpointroll - 16) || inputroll > (midpointroll + 16)) targetanglex = (inputroll - 1500) / Sx; //dead band to reduce noise
  else targetanglex = 0;
  float errorx = targetanglex - (-roll); //The error is calculated
  if(Ix < IMAXx) Ix += errorx * looptime; //I component
  if(roll < 2.35|| roll > 2.27){Ix = 0;}
  ux = (errorx * KPx + Ix * KIx + gx * KDx); //P + I + D
  backrightchange -= ux; backleftchange += ux; frontrightchange -= ux; frontleftchange += ux;
  }

  
void PIDy(float pitch,float gy){ //This controls the front to back motion of the drone. 
  if(inputpitch < 1000)inputpitch = 1000;
  if(inputpitch > 2000)inputpitch = 2000;
  //float ob = u.distanceRead(); //read the distance from the sensor.
  float uy = 0;
  int targetangley;
  if(inputpitch < (midpointpitch - 16) || inputpitch > (midpointpitch + 16)) targetangley = (inputpitch - 1500) / Sy; //dead band to reduce noise
  else targetangley = 0;
  //targetangley /= 5.5;
  float errory = targetangley - (-pitch); //The error is calculated
  if(Iy < IMAXy) Iy += errory * looptime; //I component
  if(pitch < 2.35|| pitch > 2.27){Ix = 0;}
  uy = (errory * KPy + Iy * KIy + gy * KDy);//P + I + D
  backleftchange += uy; frontleftchange -= uy; frontrightchange -= uy; backrightchange += uy; //increase on the left (pins 3 and 6)
  }

void PIDz(float yaw){ //This controls the rotations of the drone.
  // z-axis PID: (yaw)
  yaw *= -1;
  float uz = 0;
  float targetspeedz; 
  float currentspeed = (yaw - oldyaw) / looptime * 100;
  if(inputyaw < (midpointyaw - 16) || inputyaw > (midpointyaw + 16)) {
    targetspeedz = ( inputyaw - 1500 ) / Sz; //dead band to reduce noise
  } else targetspeedz = 0; 
  float errorz = targetspeedz - (currentspeed) / looptime; //gz returns an angle/second, and targetanglez is the angle of the stick. 
  if(Iz < IMAXz) Iz = IMAXz;
  float accel = ((errorz - olderrorz) / looptime);
  uz = (errorz * KPz + yaw * KIz + accel * KDz); // P + I + D
  backrightchange -= uz; backleftchange += uz; frontrightchange += uz; frontleftchange -= uz;  //increase the clockwise motors. (pins 3 and 9)
  olderrorz = errorz;
  oldyaw = yaw; 
  }

void PIDup(float baro){ //This controls the up and down motion of the drone. 
  float uup = 0;
  float targetspeedup; 
  float currentspeed = (baro - oldbaro) / looptime;
  if(inputthrottle < (midpointup - 16) || inputthrottle > (midpointup + 16)) {
    targetspeedup = (inputthrottle - 1500); //dead band to reduce noise
  } else targetspeedup = 0; 
  float errorup = 0;
  errorup = targetspeedup - currentspeed; //gz returns an angle/second, and targetanglez is the angle of the stick. 
  Iup = baro; //I component
  if(Iup < IMAXup) Iup = IMAXup;
  float accel = ((errorup - olderrorup) / looptime);
  uup = (errorup * KPup + Iup * KIup + accel * KDup); // P + I + D
  backrightchange += uup; backleftchange += uup; frontrightchange += uup; frontleftchange += uup;  //increase the clockwise motors. (pins 3 and 9)
  olderrorup = errorup;
  oldbaro = baro; 
  }
  
void loop() {
  //Serial.print("roll:");Serial.print(inputroll);Serial.print("\t throttle:");Serial.print(inputthrottle);Serial.print("\t yaw:");Serial.print(inputyaw);Serial.print("\t pitch:");Serial.print(inputpitch);Serial.print("\t onoff:");Serial.println(inputonoff);
  
  //If it is armed, light the onboard LED
  if(armed == true){
      digitalWrite(LED_BUILTIN,HIGH);
    }
  else{
      digitalWrite(LED_BUILTIN,LOW);
    } 

  float aix = 0; float aiy = 0; float aiz = 0; float gix = 0; float giy = 0; float giz = 0; //these variables record gyro and accelerometer values. 
  int ax,ay,az,gx,gy,gz;
  CurieIMU.readAccelerometer(ax, ay, az); 
  CurieIMU.readGyro(gx, gy, gz); //get gyro + accelerometer values

  aix = convertRawAcceleration(ax);
  aiy = convertRawAcceleration(ay);
  aiz = convertRawAcceleration(az);
  gix = convertRawGyro(gx);
  giy = convertRawGyro(gy);
  giz = convertRawGyro(gz);

    
  filter.updateIMU(gix, giy, giz, aix, aiy, aiz);
  float roll,pitch,yaw;
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw(); //Get the values out of the filter.
    
  //Get the last loop time.
  looptime = millis() - looptimeholder;
  looptimeholder = millis();
  
  if(inputonoff > 1900){
    armed = true;
  }else{
    armed = false;
  }
  if(armed == true){
  
  frontrightchange = 1500;
  frontleftchange = 1500;
  backrightchange = 1500;
  backleftchange = 1500; 
  
  PIDx(roll,gix);
  PIDy(pitch,giy);
  PIDz(yaw); 
  PIDup(currentsample);//If the drone is on, call the PID functions. 
  }

  if(frontrightchange < throttleMIN)frontrightchange = throttleMIN;
  if(frontleftchange < throttleMIN)frontleftchange = throttleMIN;
  if(backrightchange < throttleMIN)backrightchange = throttleMIN;
  if(backleftchange < throttleMIN)backleftchange = throttleMIN;
  
  if(frontrightchange > throttleMAX)frontrightchange = throttleMAX;
  if(frontleftchange > throttleMAX)frontleftchange = throttleMAX;
  if(backrightchange > throttleMAX)backrightchange = throttleMAX;
  if(backleftchange > throttleMAX)backleftchange = throttleMAX; //make sure that motor values do not go above 2000 or below 1000. 

if(armed == false){
   frontrightchange = 1000;
   frontleftchange = 1000;
   backrightchange = 1000;
   backleftchange = 1000;
    }

  baro.readTempC();
  currentsample = baro.readFloatPressure();
  filterBaro();
  Serial.print("frontRight: "); Serial.print(frontrightchange); Serial.print("\tfrontLeft:"); Serial.print(frontleftchange); Serial.print("\tbackRight: "); 
  Serial.print(backrightchange); Serial.print("\tbackLeft: "); Serial.println(backleftchange);
  frontright.writeMicroseconds(frontrightchange);
  frontleft.writeMicroseconds(frontleftchange);
  backright.writeMicroseconds(backrightchange);
  backleft.writeMicroseconds(backleftchange);

}
