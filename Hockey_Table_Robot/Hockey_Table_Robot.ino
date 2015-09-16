/*
Board: Uno

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




To keep angle 2 constant, you need to add 1.2pwm to forearm for every 1.0 pwm you subtract from bicep.  This only works in bicep 230 - 390 pwm



Kinematics:
http://www.oliverjenkins.com/blog/2012/9/inverse-kinematics-and-robot-arms
http://www.learnaboutrobots.com/inverseKinematics.htm

Formula by fitting curve to points
fwdback = (-0.0038 * UpDown^2) + (3.3112 * UpDown) - 437.85
max range of up/down server: 310 - 492 pwm.  310 pwm arm is extended out


Convert to degrees to pulse length
pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);



Change Log
09/04/15  v1.00 - initial version
09/06/15 - 1.01 - working on inverse kinematics
09/13/15 - 1.02 - Got angles figured out
09/15/15 - 1.03 - Inverse kinematics are working, but not for the entire range of motion
*/

#define VERSION "v1.03"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_SHIELD_ADDR 0x40
// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

// PWM range for forearm servo.  Bicep servo will be a calculation
const uint16_t SERVO_FOREARM_BCK_LMT =  492;  // This is retracted
const uint16_t SERVO_FOREARM_FWD_LMT =  310;  // This is extended
const uint16_t SERVO_ROTATE_LEFT_LMT =  400;
const uint16_t SERVO_ROTATE_RIGHT_LMT = 200;


/*
// servo address IDs on Adafruit servo board
const uint8_t SERVO_EFFECTOR = 0; 
const uint8_t SERVO_FOREARM =  1;
const uint8_t SERVO_BICEP =    2; // fwd-back
const uint8_t SERVO_ROTATE =   3; 
*/
enum servoID_t { SERVO_EFFECTOR, SERVO_FOREARM, SERVO_BICEP, SERVO_ROTATE };

const uint8_t JOYSTICK_ON = LOW;  
const uint16_t SPEED_FWD_BACK = 1000; // uS delay to slow down servo
const uint16_t SPEED_ROTATE =   5000; // uS delay to slow down servo


// Input pins for Joystick
const uint8_t JOYSTICK_FWD =   12;
const uint8_t JOYSTICK_BACK =  11; 
const uint8_t JOYSTICK_LEFT =  10;
const uint8_t JOYSTICK_RIGHT =  9;
const uint8_t JOYSTICK_BUTTON = 8;
// Save the interrupt pins D2 & D3 in case you want to use an encoder to rotate effector instead of a pot

const uint8_t EFFECTOR_INPUT = A0; // Pot for effector connected to A0

// Staring point for arm, angle1 = 90, angle 2 = 90
float g_forearmPwm =  230;
float g_bicepPwm =    388;
float g_rotatePwm =   240; 
float g_effectorPwm = 300;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(SERVO_SHIELD_ADDR);

// function prototypes
int16_t getServoPwm(servoID_t servoID, int16_t servoAngle);
bool MoveFwdBack(float armPositionY, float armPositionZ);


void setup() 
{
 
  Serial.begin(9600);
  Serial.print("Hokey Table Robot");
  Serial.println(VERSION);
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  pinMode(JOYSTICK_FWD,    INPUT_PULLUP);
  pinMode(JOYSTICK_BACK,   INPUT_PULLUP);
  pinMode(JOYSTICK_LEFT,   INPUT_PULLUP);
  pinMode(JOYSTICK_RIGHT,  INPUT_PULLUP);
  pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);

  // Initial position for robot arm
  pwm.setPWM(SERVO_FOREARM,  0, g_forearmPwm);
  pwm.setPWM(SERVO_BICEP,    0, g_bicepPwm);
  pwm.setPWM(SERVO_ROTATE,   0, getServoPwm(SERVO_ROTATE,   0));
  pwm.setPWM(SERVO_EFFECTOR, 0, getServoPwm(SERVO_EFFECTOR, 0));
  delay(1000);

}



void loop() 
{
  static float yPos =         75.0;  // Y position in mmm
  static float zPos =        -86.0;  // Z position in mm, should always be the same
  static float rotateAngle =   0.0;  // Angle of arm +/- 90
  static float effectorAngle = 0.0;  // Effector angle +/190
  static float effectorAngleOld = 0.0; // Previous effector angle  

    
//  MoveFwdBack((float)analogRead(A0)/1.5, (float)analogRead(A1)/1.5 - 100 );
//  delay(25);
  

  // Move robot arm forward, away from goal
  if ( digitalRead(JOYSTICK_FWD) == JOYSTICK_ON )
  {
    yPos = yPos + 0.5; 
    MoveFwdBack(yPos, zPos);
    delayMicroseconds(SPEED_FWD_BACK);
  }

  // Move robot arm back, toward from goal
  if ( digitalRead(JOYSTICK_BACK) == JOYSTICK_ON ) 
  {
    yPos = yPos - 0.5; 
    MoveFwdBack(yPos, zPos);
    delayMicroseconds(SPEED_FWD_BACK);
  }
  
  // rotate arm to the left
  if ( digitalRead(JOYSTICK_LEFT) == JOYSTICK_ON ) 
  {
    rotateAngle = rotateAngle + 0.5;
    pwm.setPWM(SERVO_ROTATE, 0, getServoPwm(SERVO_ROTATE, rotateAngle));
    
    // Move the effector in the opposite direction
    effectorAngle = effectorAngle - 0.5;
    pwm.setPWM(SERVO_EFFECTOR, 0, getServoPwm(SERVO_EFFECTOR, effectorAngle));
    delayMicroseconds(SPEED_ROTATE);
  }

  // rotate arm to the right
  if ( digitalRead(JOYSTICK_RIGHT) == JOYSTICK_ON ) 
  {
    rotateAngle = rotateAngle - 0.5;
    pwm.setPWM(SERVO_ROTATE, 0, getServoPwm(SERVO_ROTATE, rotateAngle));

    // Move the effector in the opposite direction
    effectorAngle = effectorAngle + 0.5;
    pwm.setPWM(SERVO_EFFECTOR, 0, getServoPwm(SERVO_EFFECTOR, effectorAngle));
    delayMicroseconds(SPEED_ROTATE);
  }

  // Swing effector - contoled by a potientiometer connected to analog input
  effectorAngle = map(analogRead(EFFECTOR_INPUT), -45, 45, 0, 512);
  if ( effectorAngle != effectorAngleOld)
  { 
    pwm.setPWM(SERVO_EFFECTOR, 0, getServoPwm(SERVO_EFFECTOR, effectorAngle)); 
    effectorAngleOld = effectorAngle;
  }
  
}  // end loop()





// Moves the arm forward or backward, keeping the head on the table
// armPosition is the position in the Y axis (in mm), z will be constant - the top of the table relative to the pivit point (in mm)
// angle1 is for the bicep servo, angle3 for the forearm
// The angles calculated from the kinematics formulas need to be conveted to match the orientation of the robotic arm
// Bicep Arm:
//   Horizantal (almost) (angle of 10), pwm = 230
//   Vertical (angle of 90), pwm = 394
// Forearm - angle changes if bicep changes
// Both arms at 90 degrees: Bicep 388, forearm 230
bool MoveFwdBack(float armPositionY, float armPositionZ)
{

  Serial.print(armPositionY);
  Serial.print("\t");
  Serial.print(armPositionZ);
  Serial.print("\t");
  
  // Inverse kinematic to position the arm
  const float BICEP_LEN_MM =   140.0;  // bicep length in mm
  const float FOREARM_LEN_MM = 153.0;  // forarm length in mm

  // Limit arm travel to avoid trig errors 
  if (armPositionY <= 0)
  { armPositionY =  0.1; } 

  
  float hyp = sqrt(armPositionY*armPositionY + armPositionZ*armPositionZ);  // calculate hypotenuse, distance from origin to end of forarm
  
  // calculate angles for arms
  float angle1degree = ( atan( armPositionZ / armPositionY ) + acos( (BICEP_LEN_MM*BICEP_LEN_MM - FOREARM_LEN_MM*FOREARM_LEN_MM + armPositionY*armPositionY + armPositionZ*armPositionZ) / (2 * BICEP_LEN_MM * hyp) ) ) * RAD_TO_DEG;
  float angle2degree = 180.0 - ( acos( (hyp*hyp - BICEP_LEN_MM*BICEP_LEN_MM - FOREARM_LEN_MM*FOREARM_LEN_MM) / (2 * BICEP_LEN_MM * FOREARM_LEN_MM) ) ) * RAD_TO_DEG;
  float angle3degree = 180.0 - angle1degree - angle2degree;
  
  uint16_t pwmBicep =   getServoPwm(SERVO_BICEP,   angle1degree);
  uint16_t pwmForearm = getServoPwm(SERVO_FOREARM, angle3degree);

  Serial.print(angle1degree);
  Serial.print("\t");
  Serial.print(angle2degree);
  Serial.print("\t");
  Serial.print(angle3degree);
  Serial.print("\t");
  Serial.print(pwmBicep);
  Serial.print("\t");
  Serial.print(pwmForearm);
  Serial.println();



  // Check for calculation errors
  if (angle1degree != angle1degree )
  { return false; }

  if (angle2degree != angle2degree )
  { return false; }

  
  pwm.setPWM(SERVO_BICEP,   0, pwmBicep);
  pwm.setPWM(SERVO_FOREARM, 0, pwmForearm);
  
  return true;
}


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


// This equation is derived from manually moving the arm with Z fixed then plotting the points 
// and finding the best fit line
uint16_t calcBicepPwm (float forearmPwm)
{
  return (-0.0038 * forearmPwm * forearmPwm) + (3.3112 * forearmPwm) - 437.85;
}  // end calcBicepPwm()






