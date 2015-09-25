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
09/25/15 - 1.05 - More fine tuning. Pass pointer to MoveFwdBack()
*/

#define VERSION "v1.05"
#define PRINT_DEBUG

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

const uint8_t EFFECTOR_INPUT = A1; // Pot for effector connected to Analog in
const float Z_POSITION = -70; 
const float JOYSTICK_STEP_FWDBACK = 0.75;
const float JOYSTICK_STEP_LR = 0.5;

// Staring point for arm, angle1 = 90, angle 2 = 90
float g_forearmPwm =  230;
float g_bicepPwm =    388;
// float g_rotatePwm =   240; 
float g_effectorPwm = 300;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(SERVO_SHIELD_ADDR);

// function prototypes
int16_t getServoPwm(servoID_t servoID, int16_t servoAngle);
bool MoveFwdBack(float *&armPositionY, float *armPositionZ);


// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
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
  pwm.setPWM(SERVO_FOREARM,  0, g_forearmPwm);
  pwm.setPWM(SERVO_BICEP,    0, g_bicepPwm);

  pwm.setPWM(SERVO_ROTATE,   0, getServoPwm(SERVO_ROTATE, 0));
  pwm.setPWM(SERVO_EFFECTOR, 0, getServoPwm(SERVO_EFFECTOR, map(analogRead(EFFECTOR_INPUT), 0, 505, 45, -45)));
  float yStart = 0.0;
  float zStart = Z_POSITION;
  MoveFwdBack(&yStart, &zStart);
  
}  // emd setup()



// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
void loop() 
{
  static float yPos =            75.0;  // Y position in mm
  static float zPos =      Z_POSITION;  // Z position in mm, should always be the same
  static float rotateAngle =      0.0;  // Angle of arm +/- 90
  static float effectorAngle =    0.0;  // Effector angle +/190
  static float effectorAngleOld = 0.0;  // Previous effector angle  

  
  // Move robot arm forward, away from goal
  if ( digitalRead(JOYSTICK_FWD) == JOYSTICK_ON )
  {
    yPos = yPos + JOYSTICK_STEP_FWDBACK; 
    MoveFwdBack(&yPos, &zPos);
 //   delayMicroseconds(SPEED_FWD_BACK);
  }

  // Move robot arm back, toward from goal
  if ( digitalRead(JOYSTICK_BACK) == JOYSTICK_ON ) 
  {
    yPos = yPos - JOYSTICK_STEP_FWDBACK; 
    MoveFwdBack(&yPos, &zPos);
 //   delayMicroseconds(SPEED_FWD_BACK);
  }
  
  // Rotate arm to the left
  if ( digitalRead(JOYSTICK_LEFT) == JOYSTICK_ON ) 
  {
    rotateAngle = rotateAngle + JOYSTICK_STEP_LR;
    
    // Don't let arm hit back wall
    float backWallAngle = 0.088 * yPos + 58.828;
    if ( rotateAngle > backWallAngle )
    { rotateAngle = backWallAngle; }
    
    pwm.setPWM(SERVO_ROTATE, 0, getServoPwm(SERVO_ROTATE, rotateAngle));
    
    delayMicroseconds(SPEED_ROTATE);
  }

  // rotate arm to the right
  if ( digitalRead(JOYSTICK_RIGHT) == JOYSTICK_ON ) 
  {
    rotateAngle = rotateAngle - JOYSTICK_STEP_LR;

    // Don't let arm hit back wall
    float backWallAngle = -1.0 * (0.088 * yPos + 58.828) ;
    if ( rotateAngle < backWallAngle )
    { rotateAngle = backWallAngle; }

    pwm.setPWM(SERVO_ROTATE, 0, getServoPwm(SERVO_ROTATE, rotateAngle));

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
    airOnTime = millis() + 200;        // Keep solenoid open for 200mS
    airDelayTimer = millis() + 500;    // Don't let user fire again for 300mS
  }
  
  // Turn off air
  if (millis() > airOnTime)
  { digitalWrite(AIR_SOLENOID, LOW); }

  #ifdef PRINT_DEBUG
    Serial.print("Y:");
    Serial.print(yPos);
    Serial.print("  Z:");
    Serial.print(zPos);
    Serial.print("  A:");
    Serial.print(rotateAngle);
    Serial.print(" ");
    Serial.print(getServoPwm(SERVO_ROTATE, rotateAngle));
    Serial.print("  E:");
    Serial.print(effectorAngle);
    Serial.println();
  #endif
    
}  // end loop()



// ---------------------------------------------------------------------------
// Moves the arm forward or backward, keeping the head on the table
// armPosition is the position in the Y axis (in mm), z will be constant - the top of the table relative to the pivit point (in mm)
// angle1 is for the bicep servo, angle3 for the forearm
// The angles calculated from the kinematics formulas need to be conveted to match the orientation of the robotic arm
// Bicep Arm:
//   Horizantal (almost) (angle of 10), pwm = 230
//   Vertical (angle of 90), pwm = 394
// Forearm - angle changes if bicep changes
// Both arms at 90 degrees: Bicep 388, forearm 230
// ---------------------------------------------------------------------------
bool MoveFwdBack(float *armPositionY, float *armPositionZ)
{
 
  // Inverse kinematic to position the arm
  const float BICEP_LEN_MM =   140.0;  // bicep length in mm
  const float FOREARM_LEN_MM = 153.0;  // forarm length in mm

  // Limit arm travel to avoid trig errors 
  if (*armPositionY <= 0)
  { *armPositionY =  0.1; } 

  
  float hyp = sqrt( *armPositionY * *armPositionY + *armPositionZ * *armPositionZ);  // calculate hypotenuse, distance from origin to end of forarm
  
  // Calculate angles for forearm and bicep servos (Y-Z plane)
  // See: https://github.com/Scott216/Hockey-Robot/blob/master/Arm%20Angles.jpg
  float angle1degree = ( atan( *armPositionZ / *armPositionY ) + acos( (BICEP_LEN_MM * BICEP_LEN_MM - FOREARM_LEN_MM * FOREARM_LEN_MM + *armPositionY * *armPositionY + *armPositionZ * *armPositionZ) / (2 * BICEP_LEN_MM * hyp) ) ) * RAD_TO_DEG;
  float angle2degree = 180.0 - ( acos( (hyp * hyp - BICEP_LEN_MM*BICEP_LEN_MM - FOREARM_LEN_MM * FOREARM_LEN_MM) / (2 * BICEP_LEN_MM * FOREARM_LEN_MM) ) ) * RAD_TO_DEG;
  float angle3degree = 180.0 - angle1degree - angle2degree;
  
  uint16_t pwmBicep =   getServoPwm(SERVO_BICEP,   angle1degree);
  uint16_t pwmForearm = getServoPwm(SERVO_FOREARM, angle3degree);


  // Check for calculation errors
  if (angle1degree != angle1degree )
  { return false; }

  if (angle2degree != angle2degree )
  { return false; }

  
  pwm.setPWM(SERVO_BICEP,   0, pwmBicep);
  pwm.setPWM(SERVO_FOREARM, 0, pwmForearm);
  
  return true;
}  // end MoveFwdBack()


// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
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


// ---------------------------------------------------------------------------
// This equation is derived from manually moving the arm with Z fixed then plotting the points 
// and finding the best fit line
// ---------------------------------------------------------------------------
uint16_t calcBicepPwm (float forearmPwm)
{
  return (-0.0038 * forearmPwm * forearmPwm) + (3.3112 * forearmPwm) - 437.85;
}  // end calcBicepPwm()





