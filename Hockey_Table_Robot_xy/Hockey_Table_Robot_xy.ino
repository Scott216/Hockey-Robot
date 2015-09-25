/*
Board: Uno

Source code: https://github.com/Scott216/Hockey-Robot

Program uses Arduino Uno with Adafruit I2C servo shield to control two robotic arms mounted to a hockey table
Users can move arms back and forth and flick the hockey puck.  Flicking the hockey puck uses a spring loaded
mechanism.

Note: Using serial.print() really slows down servo speed a lot

Adafruit 16-channel PWM Servo shield http://www.adafruit.com/product/1411
Tutorial: https://learn.adafruit.com/adafruit-16-channel-pwm-slash-servo-shield
Robotic Arm http://www.sainsmart.com/diy-4-axis-servos-control-palletizing-robot-arm-model-for-arduino-uno-mega2560.html



PWM Servo Info, Arm 1:
Effector:       185 - 580
Forearm:        215 - 457, range  90 deg, low value = arm retracted
Bicep:          230 - 456, range 110 deg, high value =  is retracted
Rotate:         115 - 500, range 200 deg

Servo Characterization
Servo       PWM    Angle     PWM    Angle
Effector
Forearm      210      0      428      90
Bicep        230      5      394      90
Rotate



Kinematics:
http://www.oliverjenkins.com/blog/2012/9/inverse-kinematics-and-robot-arms
http://www.learnaboutrobots.com/inverseKinematics.htm

Formula by fitting curve to points
fwdback = (-0.0038 * UpDown^2) + (3.3112 * UpDown) - 437.85
max range of up/down server: 310 - 492 pwm.  310 pwm arm is extended out


Change Log
09/04/15  v1.00 - initial version
09/06/15 - 1.01 - working on inverse kinematics
09/13/15 - 1.02 - Got angles figured out
09/15/15 - 1.03 - Inverse kinematics are working, but not for the entire range of motion
09/24/15 - 1.04 - Fine tuning IK
09/24/15 - 1.05 - change to X-Y positioning from Angle-Y.  This way using joystick is more intuitive 
*/

#define VERSION "v1.05"
//#define PRINT_DEBUG2

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_SHIELD_ADDR 0x40


enum servoID_t { SERVO_EFFECTOR, SERVO_FOREARM, SERVO_BICEP, SERVO_ROTATE };

const uint8_t JOYSTICK_ON = LOW;  
const uint16_t SPEED_FWD_BACK = 10; // uS delay to slow down servo
const uint16_t SPEED_ROTATE =   1000; // uS delay to slow down servo


// Input pins for Joystick
const uint8_t JOYSTICK_FWD =   12;
const uint8_t JOYSTICK_BACK =  11; 
const uint8_t JOYSTICK_LEFT =  10;
const uint8_t JOYSTICK_RIGHT =  9;
const uint8_t JOYSTICK_BUTTON = 8;
const uint8_t AIR_SOLENOID =    7;
// Save the interrupt pins D2 & D3 in case you want to use an encoder to rotate effector instead of a pot

const uint8_t EFFECTOR_INPUT = A1; // Pot for effector connected to A0
const float Z_POSITION = -70; 
const float X_JOYSTICK_STEP = 0.5;
const float Y_JOYSTICK_STEP = 0.75;
const float MAX_YZ_HYP = 259.0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(SERVO_SHIELD_ADDR);

// function prototypes
int16_t getServoPwm(servoID_t servoID, int16_t servoAngle);
bool MoveFwdBack(float armPositionY, float armPositionZ);


void setup() 
{
 
  Serial.begin(9600);
  Serial.print("Hokey Table Robot ");
  Serial.println(VERSION);
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  pinMode(JOYSTICK_FWD,    INPUT_PULLUP);
  pinMode(JOYSTICK_BACK,   INPUT_PULLUP);
  pinMode(JOYSTICK_LEFT,   INPUT_PULLUP);
  pinMode(JOYSTICK_RIGHT,  INPUT_PULLUP);
  pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);
  pinMode(AIR_SOLENOID,          OUTPUT);

  // Initial position for robot arm
  pwm.setPWM(SERVO_EFFECTOR, 0, getServoPwm(SERVO_EFFECTOR, map(analogRead(EFFECTOR_INPUT), 0, 505, 45, -45)));
  float xStart = 0.0;
  float yStart = 75.0;
  MoveArm(&xStart, &yStart);
  
}  // emd setup()



void loop() 
{
  static float xPosMM =        0.0;  // X position in mm
  static float yPosMM =       75.0;  // Y position in mmm
  static float effectorAngle = 0.0;  // Effector angle +/190
  static float effectorAngleOld = 0.0; // Previous effector angle  

    

  // Move robot arm forward
  bool moveStatus; // whether move was okay or went into restrected regions
  if ( digitalRead(JOYSTICK_FWD) == JOYSTICK_ON )
  {
    yPosMM = yPosMM + Y_JOYSTICK_STEP; 
    moveStatus = MoveArm(&xPosMM, &yPosMM);
    
    // If move went into restricted area, undo move increment
    if ( moveStatus == false )
    {  yPosMM = yPosMM - 2 * Y_JOYSTICK_STEP; }
  }

  // Move robot arm backward
  if ( digitalRead(JOYSTICK_BACK) == JOYSTICK_ON ) 
  {
    yPosMM = yPosMM - Y_JOYSTICK_STEP; 
    moveStatus = MoveArm(&xPosMM, &yPosMM);

    // If move went into restricted area, undo move increment
//    if ( moveStatus == false )
//    {  yPosMM = yPosMM + Y_JOYSTICK_STEP; }
  }
  
  // Move arm to the left
  if ( digitalRead(JOYSTICK_LEFT) == JOYSTICK_ON ) 
  {
    xPosMM = xPosMM - X_JOYSTICK_STEP;
    moveStatus = MoveArm(&xPosMM, &yPosMM);
    
    // If move went into restricted area, undo move increment
//    if ( moveStatus == false )
//    {  xPosMM = xPosMM + X_JOYSTICK_STEP; }
    
    delayMicroseconds(SPEED_ROTATE);
  }

  // Move arm to the right
  if ( digitalRead(JOYSTICK_RIGHT) == JOYSTICK_ON ) 
  {
    xPosMM = xPosMM + X_JOYSTICK_STEP;
    moveStatus = MoveArm(&xPosMM, &yPosMM);
    
     // If move went into restricted area, undo move increment
//    if ( moveStatus == false )
//    {  xPosMM = xPosMM - X_JOYSTICK_STEP; }
   
    delayMicroseconds(SPEED_ROTATE);
  }

  // Swing effector - contoled by a potientiometer connected to analog input
  effectorAngle = map(analogRead(EFFECTOR_INPUT), 0, 505, 45, -45); 
  if ( effectorAngle != effectorAngleOld)
  { 
    pwm.setPWM(SERVO_EFFECTOR, 0, getServoPwm(SERVO_EFFECTOR, effectorAngle)); 
    effectorAngleOld = effectorAngle;
  }
  
  // Turn on air
  static uint32_t airOnTime = millis();  // Timer to keep air solenoid open when trigger is pushed
  static uint32_t airDelayTimer = millis(); // time user has to wait between trigger pulls
  if ( digitalRead(JOYSTICK_BUTTON) == LOW && millis() > airDelayTimer )
  { 
    digitalWrite(AIR_SOLENOID, HIGH);  // Open air solenoid
    airOnTime = millis() + 400;        // Keep solenoid open for 200mS
    airDelayTimer = millis() + 500;    // Don't let user fire again for 300mS
  }
  
  // Turn off air
  if (millis() > airOnTime)
  { digitalWrite(AIR_SOLENOID, LOW); }

 // Print position info
  #ifdef PRINT_DEBUG
    Serial.print("X:");
    Serial.print(xPosMM);
    Serial.print("  Y:");
    Serial.print(yPosMM);
    Serial.println();
  #endif
    
}  // end loop()

bool MoveArm(float *xPos, float *yPos1)
{
  
  // Inverse kinematic to position the arm
  const float BICEP_LEN_MM =   140.0;  // bicep length in mm
  const float FOREARM_LEN_MM = 153.0;  // forarm length in mm


  // Limit arm travel to avoid trig errors and physical boundaries 
  if (*xPos < -183.0)
  { *xPos = -183.0; } 

  if (*xPos > 186.0)
  { *xPos = 186.0; } 

  if (*yPos1 < 2)
  { *yPos1 = 2; } 
//  if (*yPos1 < 45.75)
//  { *yPos1 = 45.75; } 

  if (*yPos1 > 1000.0)
  { *yPos1 = 1000.0; } 


  float hypXY = sqrt( *xPos * *xPos + *yPos1 * *yPos1 ); 

  // Moving from the X-Y plane to Y-Z plane, the X-Y hypotenuse is the y-position in the Y-Z plane
  float yPos2 = hypXY; 

  float zPos = Z_POSITION;

  float hypYZ = sqrt( yPos2 * yPos2 + zPos * zPos );  // calculate hypotenuse for YZ plane, distance from origin to end of forarm

  // Since the arm doesn't move perfectly flat, we need to adjust Z position to accomodiate
  zPos = zHeightCorrection(hypYZ);
  
  // Calculate angle for rotating arm servo
  float angleRotatedegree = acos(*xPos/hypXY) * RAD_TO_DEG - 90.0;

 
  // Calculate angles for forearm and bicep servos (Y-Z plane)
  // See: https://github.com/Scott216/Hockey-Robot/blob/master/Arm%20Angles.jpg
  float angle1degree = ( atan( zPos / yPos2 ) + acos( (BICEP_LEN_MM*BICEP_LEN_MM - FOREARM_LEN_MM*FOREARM_LEN_MM + yPos2*yPos2 + zPos*zPos) / (2 * BICEP_LEN_MM * hypYZ) ) ) * RAD_TO_DEG;
  float angle2degree = 180.0 - ( acos( (hypYZ*hypYZ - BICEP_LEN_MM*BICEP_LEN_MM - FOREARM_LEN_MM*FOREARM_LEN_MM) / (2 * BICEP_LEN_MM * FOREARM_LEN_MM) ) ) * RAD_TO_DEG;
  float angle3degree = 180.0 - angle1degree - angle2degree;

  #ifdef PRINT_DEBUG2
    Serial.print(*xPos);
    Serial.print("  ");
    Serial.print(*yPos1);
    Serial.print("  ");
    Serial.print(zPos);
    Serial.print("  ");
    Serial.print(hypYZ);
    
//    Serial.print("  ");
//    Serial.print(angle1degree);
//    Serial.print("  ");
//    Serial.print(angle2degree);
//    Serial.print("  ");
//    Serial.print(angle3degree);
//    Serial.print("  ");
//    Serial.print(angleRotatedegree);
    Serial.println();
  #endif


  // Check for nan calculation errors
  if (angle1degree != angle1degree )
  { return false; }
  if (angle2degree != angle2degree )
  { return false; }

  // Get servo PWM values
  uint16_t pwmRotate =  getServoPwm(SERVO_ROTATE,  angleRotatedegree);
  uint16_t pwmBicep =   getServoPwm(SERVO_BICEP,   angle1degree);
  uint16_t pwmForearm = getServoPwm(SERVO_FOREARM, angle3degree);

  // Send PWM value to servos
  pwm.setPWM(SERVO_ROTATE,  0, pwmRotate);
  pwm.setPWM(SERVO_BICEP,   0, pwmBicep);
  pwm.setPWM(SERVO_FOREARM, 0, pwmForearm);
  
  // if the Y-Z hypentose is too big, only rotate
  if (hypYZ > MAX_YZ_HYP )
  { return false; }
  else
  {
    return true; 
  }
  
}  // end MoveArm()


// Trying to move arm forward and back in y-Z plane while keeping Z constant doesn't work very perfectly.  Not sure if it an issue with the
// IK calculations or something mechanical.  The function tries to compensate and keep Z height relatively constant
float zHeightCorrection(float hypYZ)
{
  float zOffset = Z_POSITION;
  float hypSquared = hypYZ * hypYZ;
  
  if ( hypYZ < 71.0 )
  { zOffset = -0.111 * hypSquared + 15.807 * hypYZ - 562.379; }
  else if (hypYZ < 186 )
  { zOffset = -0.001816 * hypSquared + 0.74 * hypYZ - 42.2; }
  else if (hypYZ < 226 )
  { zOffset = -0.001452 * hypSquared + 0.449 * hypYZ - 0.6; }
  else 
  { zOffset = 30; }

  return Z_POSITION - zOffset;
  
}  // end adjustZPos()



// for each servo, returns the PWM value for a give angle
int16_t getServoPwm(servoID_t servoID, int16_t servoAngle)
{

  switch (servoID)
  {
    case SERVO_EFFECTOR:
      return map((long)servoAngle, -90, 90, 573, 172); 
      break;
    case SERVO_FOREARM:
      return map((long)servoAngle, 5, 90, 230, 394);
      break; 
    case SERVO_BICEP:
      return  map((long)servoAngle, 0, 90, 210, 428);
      break;
    case SERVO_ROTATE:
      return  map((long)servoAngle, -90, 90, 164, 500);
      break;
    default:
      return 300;
      break;
  } 
  
}  // end getServoPwm()



