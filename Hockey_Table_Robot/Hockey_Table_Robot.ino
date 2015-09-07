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
Forearm:        215 - 457, range  90 deg
Bicep:          230 - 456, range 110 deg
Rotate:         115 - 500, range 200 deg



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



*/

#define VERSION "v1.00"

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


// servo address IDs on Adafruit servo board
const uint8_t SERVO_EFFECTOR = 0; 
const uint8_t SERVO_FOREARM =  1;
const uint8_t SERVO_BICEP =    2; //fwd-back
const uint8_t SERVO_ROTATE =   3; 


const uint8_t JOYSTICK_ON = LOW;  
const uint16_t SPEED_FWD_BACK = 1000; // uS delay to slow down servo
const uint16_t SPEED_ROTATE =   5000; // uS delay to slow down servo


const uint8_t JOYSTICK_FWD =   12;
const uint8_t JOYSTICK_BACK =  11; 
const uint8_t JOYSTICK_LEFT =  10;
const uint8_t JOYSTICK_RIGHT =  9;
const uint8_t JOYSTICK_BUTTON = 8;

// Staring point for arm
float g_forearmPwm = 406;
float g_bicepPwm =   281;
float g_rotatePwm =  300; 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(SERVO_SHIELD_ADDR);


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

  pwm.setPWM(SERVO_FOREARM, 0, g_forearmPwm);
  pwm.setPWM(SERVO_BICEP,   0, g_bicepPwm);
  pwm.setPWM(SPEED_ROTATE,  0, g_rotatePwm);

}



void loop() 
{
  
  // Move robot arm forward, away from goal
  if ( digitalRead(JOYSTICK_FWD) == JOYSTICK_ON )
  {
    g_forearmPwm--; // reducing value extends arm
    if (g_forearmPwm < SERVO_FOREARM_FWD_LMT)
    { g_forearmPwm = SERVO_FOREARM_FWD_LMT; }
    g_bicepPwm = calcBicepPwm(g_forearmPwm);
    pwm.setPWM(SERVO_FOREARM, 0, g_forearmPwm);
    pwm.setPWM(SERVO_BICEP,   0, g_bicepPwm);
    delayMicroseconds(SPEED_FWD_BACK);
  }

  // Move robot arm back, toward from goal
  if ( digitalRead(JOYSTICK_BACK) == JOYSTICK_ON ) 
  {
    g_forearmPwm++;   // increase value reetracts arm
    if (g_forearmPwm > SERVO_FOREARM_BCK_LMT)
    { g_forearmPwm = SERVO_FOREARM_BCK_LMT; }
    g_bicepPwm = calcBicepPwm(g_forearmPwm);
    pwm.setPWM(SERVO_FOREARM, 0, g_forearmPwm);
    pwm.setPWM(SERVO_BICEP,   0, g_bicepPwm);
    delayMicroseconds(SPEED_FWD_BACK);
  }
  
  // swing arm to the left
  if ( digitalRead(JOYSTICK_LEFT) == JOYSTICK_ON ) 
  {
    g_rotatePwm++;
    if (g_rotatePwm > SERVO_ROTATE_LEFT_LMT )
    { g_rotatePwm = SERVO_ROTATE_LEFT_LMT; }
    pwm.setPWM(SERVO_ROTATE, 0, g_rotatePwm);
    delayMicroseconds(SPEED_ROTATE);
  }

  // swing arm to the right
  if ( digitalRead(JOYSTICK_RIGHT) == JOYSTICK_ON ) 
  {
    g_rotatePwm--;
    if (g_rotatePwm < SERVO_ROTATE_RIGHT_LMT )
    { g_rotatePwm = SERVO_ROTATE_RIGHT_LMT; }
    pwm.setPWM(SERVO_ROTATE, 0, g_rotatePwm);
    delayMicroseconds(SPEED_ROTATE);
  }

  
}  // end loop()


uint16_t calcBicepPwm (float forearmPwm)
{
  return (-0.0038 * forearmPwm * forearmPwm) + (3.3112 * forearmPwm) - 437.85;
}  // end calcBicepPwm()




/*
void slideBackAndForth()
{

  float upDwn;
  float fwdBack;
  
  Serial.println("----------");
  for ( upDwn = 492; upDwn >= 310; upDwn-- )
  {
    fwdBack =  (-0.0038 * upDwn * upDwn) + (3.3112 * upDwn) - 437.85; 
    pwm.setPWM(SERVO_UP_DOWN,  0, upDwn);
    pwm.setPWM(SERVO_FWD_BACK, 0, fwdBack);
    Serial.print(upDwn);
    Serial.print("\t");
    Serial.println(fwdBack);
  }

  Serial.println("----------");
  delay(3000);
  
  for ( upDwn = 310; upDwn <= 492; upDwn++ )
  {
    fwdBack =  (-0.0038 * upDwn * upDwn) + (3.3112 * upDwn) - 437.85; 
    pwm.setPWM(SERVO_UP_DOWN,  0, upDwn);
    pwm.setPWM(SERVO_FWD_BACK, 0, fwdBack);
    Serial.print(upDwn);
    Serial.print("\t");
    Serial.println(fwdBack);
  }
  delay(3000);
  
}
*/

// Will move arm to new position, but it won't move it to fast
void slowMove(int16_t newFwdBk, int16_t newUpDn)
{
   static int16_t currentUpDn = newUpDn;
   static int16_t currentFwdBk = newFwdBk;
   
   int16_t upDnDirection =  1;
   int16_t fwdBkDirection = 1;
   if ( newUpDn < currentUpDn )
   { upDnDirection = -1; }
   if ( newFwdBk < currentFwdBk )
   { fwdBkDirection = -1; }
   
   bool finishedMoving = false;
   
   while (finishedMoving == false)
   {
     if( currentUpDn != newUpDn )
     {  
       currentUpDn = currentUpDn + upDnDirection; 
       pwm.setPWM(SERVO_FOREARM,  0, currentUpDn);
     }

     if( currentFwdBk != newFwdBk )
     {  
       currentFwdBk = currentFwdBk + fwdBkDirection; 
       pwm.setPWM(SERVO_BICEP,  0, currentFwdBk);
     }
     
     if ( (currentUpDn == newUpDn) && (currentFwdBk == newFwdBk) )
     {  finishedMoving = true; }
   }
  
}  // end slowMove()


// Moves the arm forward or backward, keeping the head on the table
// armPosition is the position in the Y axis (in mm), z will be constant - the top of the table relative to the pivit point (in mm)
// angle1 is for the fwd/back servo, angle2 for the up/dn servo
// The angles calculated from the kinematics formulas need to be conveted to match the orientation of the robotic arm
// Fwd/Back Servo (1st arm):
//   Horizantal (angle of zero), pwm = 230
//   Vertical (angle of 90), pwm = 394
// Up/Down Servo (2nd arm):
//   Horizantal (angle of 90), pwm = 215
//   Vertucal (angle of 0), pwm = 415
void MoveFwdBack(float armPositionY, float armPositionZ)
{

  // Inverse kinematic to position the arm
  const float ARM1 = 140.0;  // arm length in mm, fwd/back servo
  const float ARM2 = 153.0;  // up/down servo

  // Limit arm travel to avoid trig errors 
  if (armPositionY > 279.0)
  { armPositionY = 279.0;} 
  
  float hyp = sqrt(armPositionY*armPositionY + armPositionZ*armPositionZ);  // calculate hypotenuse, distance from origint to head
  
  // calculate angle for fwd/back servo
  float angle1 = ( atan( armPositionZ / armPositionY ) + acos( (ARM1*ARM1 - ARM2*ARM2 + armPositionY*armPositionY + armPositionZ*armPositionZ) / (2 * ARM1 * hyp) ) ) * RAD_TO_DEG;
  float angle2 = ( acos( (hyp*hyp - ARM1*ARM1 - ARM2*ARM2) / (2 * ARM1 * ARM2) ) ) * RAD_TO_DEG;
  
  uint16_t pwmFwdBack = map((long)angle1, 0, 90, 230, 394);
  uint16_t pwmUpDown  = map((long)angle2, 0, 90, 215, 457);
  pwm.setPWM(SERVO_BICEP, 0, pwmFwdBack);
  pwm.setPWM(SERVO_FOREARM,  0, pwmUpDown);
  
  Serial.print(armPositionY);
  Serial.print("\t");
  Serial.print(angle1);
  Serial.print("\t");
  Serial.print(angle2);
  Serial.print("\t");
  Serial.print(pwmFwdBack);
  Serial.print("\t");
  Serial.print(pwmUpDown);
  Serial.println();

}


// User pushed the button to swing the hockey stick
void SwingHockeyStick()
{
  
}
