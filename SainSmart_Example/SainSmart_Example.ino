/*

Code from Sain Smart.  I took out the wireless part.  I think it assumes analog joysticks


*/

#include <Servo.h>

long Joystick_1_X;
long Joystick_1_Y;
long Joystick_2_X;
long Joystick_2_Y;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

float k_1;
float k_2;
float k_3;
float k_4;

float speed_1;
float speed_2;
float speed_3;
float speed_4;

void setup()
{
  Serial.begin(9600);
  
  servo1.attach(5);
  servo2.attach(6);
  servo3.attach(9);
  servo4.attach(10);
  
} // end setup()

void loop()
{

  Joystick_1_X = analogRead(A0);
  Joystick_1_Y = analogRead(A1);
  Joystick_2_X = 0;
  Joystick_2_Y = 0;
  
  // convert joystick range from -100 to +100
  if(Joystick_1_X > 530)
  { k_1 = map(Joystick_1_X, 531, 1023, 0, 100) / 2000.00; }
  else if(Joystick_1_X < 470)
  { k_1 = map(Joystick_1_X, 0, 469, -100, 0) / 2000.00; }
  else
  { k_1 = 0; }
  
  if(Joystick_1_Y > 530)
  { k_2 = map(Joystick_1_Y, 531, 1023, 0, 100) / 2000.00; }
  else if(Joystick_1_Y < 470)
  { k_2 = map(Joystick_1_Y, 0, 469, -100, 0) / 2000.00; }
  else
  { k_2 = 0; }
  
  if(Joystick_2_X > 530)
  { k_3 = map(Joystick_2_X, 531, 1023, 0, 100) / 2000.00; }
  else if(Joystick_2_X < 470)
  {k_3 = map(Joystick_2_X, 0, 469, -100, 0) / 2000.00; }
  else
  { k_3 = 0; }
  
  if(Joystick_2_Y > 530)
  { k_4 = map(Joystick_2_Y, 531, 1023, 0, 100) / 2000.00; }
  else if(Joystick_2_Y < 470)
  { k_4 = map(Joystick_2_Y, 0, 469, -100, 0) / 2000.00; }
  else
  { k_4 = 0; }
  
  // Increase/decrease speed by k value, and limit the range
  speed_1 = min(60, max(-60, speed_1 += k_1));  // limit speed to +/-60
  speed_2 = min(35, max(-35, speed_2 += k_2));  // limit speed to +/-35
  speed_3 = min(35, max(-35, speed_3 += k_3));  // limit speed to +/-35
  speed_4 = min(35, max(-35, speed_4 += k_4));  // limit speed to +/-35
  
  Serial.print(k_1);
  Serial.print("\t"); 
  Serial.print(k_2);
  Serial.print("\t"); 
  Serial.print(k_3);
  Serial.print("\t"); 
  Serial.print(k_4);
  Serial.print("\t"); 
  Serial.print("\t"); 
  Serial.print(speed_1);
  Serial.print("\t"); 
  Serial.print(speed_2);
  Serial.print("\t"); 
  Serial.print(speed_3);
  Serial.print("\t"); 
  Serial.print(speed_4);
  Serial.print("\t"); 
  Serial.println();
  
  servo1.write(speed_1 + 90);
  servo2.write(speed_2 + 90);
  servo3.write(speed_3 + 90);
  servo4.write(speed_4 + 90);
  
}  // end loop()

