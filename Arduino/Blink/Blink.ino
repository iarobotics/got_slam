/*
#  Chefbot_ROS_Interface.ino
#  
#  Copyright 2015 Lentin Joseph <qboticslabs@gmail.com>
#  Website : www.qboticslabs.com , www.lentinjoseph.com
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 2 of the License, or
#  (at your option) any later version.
#  
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA 02110-1301, USA.
#  
#  Some of the portion is adapted from I2C lib example code for MPU 6050
*/


//MPU 6050 Interfacing libraries

//Library to communicate with I2C devices
#include "Wire.h"
//I2C communication library for MPU6050
#include "I2Cdev.h"
//MPU6050 interfacing library
#include "MPU6050_6Axis_MotionApps20.h"
//Processing incoming serial data 
#include <Messenger.h>
//Contain definition of maximum limits of various data type
#include <limits.h>

//Creating MPU6050 Object
MPU6050 accelgyro(0x68);
//Messenger object
Messenger Messenger_Handler = Messenger();

//#define OUTPUT_READABLE_QUATERNION
//Encoder pins definition

// Left encoder

#define Left_Encoder_PinA 2    //Wheel A
#define Left_Encoder_PinB 3

volatile long Left_Encoder_Ticks = 0;
volatile bool LeftEncoderBSet;

//Right Encoder

#define Right_Encoder_PinA 5   //Wheel B
#define Right_Encoder_PinB 6
volatile long Right_Encoder_Ticks = 0;
volatile bool RightEncoderBSet;

/////////////////////////////////////////////////////////////////
//Motor Pin definition
//Left Motor pins

#define A_1 11
#define B_1 12

//PWM 1 pin number
#define PWM_1 10

//Right Motor
#define A_2 26 
#define B_2 28

//PWM 2 pin number
#define PWM_2 30

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Ultrasonic pins definition
//const int echo = 9, Trig = 10;
long duration, cm;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Battery level monitor for future upgrade
#define BATTERY_SENSE_PIN 23

float battery_level = 12;

//Reset pin for resetting Tiva C, if this PIN set high, Tiva C will reset

#define RESET_PIN 24

#define GREEN_LED 12
#define BOARD_LED 13

/////////////////////////////////////////////////////////////////////////////////////////
//Time  update variables

unsigned long LastUpdateMicrosecs = 0;    
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;

///////////////////////////////////////////////////////////////////////////////////////
//Motor speed from PC
//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;
/////////////////////////////////////////////////////////////////


//Setup serial, encoders, ultrasonic, MPU6050 and Reset functions
void setup()
{
  
  //Init Serial port with 115200 baud rate
  Serial.begin(115200);  

  Setup_MPU6050();
  //Setup Reset pins
  //SetupReset();
  //Set up Messenger 
  Messenger_Handler.attach(OnMssageCompleted);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(BOARD_LED, OUTPUT);

}


void Setup_MPU6050()
{


    Wire.begin();
   // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP

void loop()
{
    //Read from Serial port
    Read_From_Serial();
    
    
    //Send time information through serial port
    Update_Time();
    
    //Send encoders values through serial port
    Update_Encoders();
    
    //Send ultrasonic values through serial port
    //Update_Ultra_Sonic();
        

    //Update motor values with corresponding speed and send speed values through serial port
    Update_Motors();


    //Send MPU 6050 values through serial port
    Update_MPU6050();
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial()

{
   while(Serial.available() > 0)
    {
     
       int data = Serial.read();
       Messenger_Handler.process(data);
     
     
    } 
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition

void OnMssageCompleted()
{
   
  char reset[] = "r";
  char set_speed[] = "s";
  
  if(Messenger_Handler.checkString(reset))
  {
    
     Serial.println("Reset Request");
     delay(3000); 
  }
  if(Messenger_Handler.checkString(set_speed))
  {
    Serial.println("Set speed");
    delay(3000);
    
     //This will set the speed
     Set_Speed();
     return; 
    
    
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set speed
void Set_Speed()
{
    
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong();
  
  
  
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update both motors
void Update_Motors()
{
  
  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);

  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\t");
  Serial.print(motor_right_speed);  
  Serial.print("\n");


}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Will update both encoder value through serial port
void Update_Encoders()
{
 
  Serial.print("e");
  Serial.print("\t");
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
  Serial.print(Right_Encoder_Ticks);
  Serial.print("\n");
  
  
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update MPU6050

void Update_MPU6050()
{
  
  
  
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print("a\t");
  Serial.print(ax);
  Serial.print("\t");
  Serial.print(ay);
  Serial.print("\t");
  Serial.println(az);

  Serial.print("g\t");
  Serial.print(gx);
  Serial.print("\t");
  Serial.print(gy);
  Serial.print("\t");
  Serial.println(gz);
 
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update time function
void Update_Time()
{
  
      
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0)
    {
  MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;

    }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
 
  
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Conversion of microsecond to centimeter
long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29 / 2;
}

//Motor running function


void moveRightMotor(float rightServoValue)
{
  // Blink GREEN_LED
  if (rightServoValue>0)
  {

  digitalWrite(GREEN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(GREEN_LED, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second
       
 // digitalWrite(B_1,LOW);
 // analogWrite(PWM_1,rightServoValue);
    
  }
  else if(rightServoValue<0)
  {
 // digitalWrite(A_1,LOW);
 // digitalWrite(B_1,HIGH);
 // analogWrite(PWM_1,abs(rightServoValue));
  digitalWrite(GREEN_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for a second
  digitalWrite(GREEN_LED, LOW);    // turn the LED off by making the voltage LOW
  delay(500);               // wait for a second
 
  }
  
  else if(rightServoValue == 0)
  {
 // digitalWrite(A_1,HIGH);
 // digitalWrite(B_1,HIGH);
    digitalWrite(GREEN_LED, LOW);
    
    
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveLeftMotor(float leftServoValue)
{
 if (leftServoValue > 0)
  {
  digitalWrite(BOARD_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(BOARD_LED, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second
// digitalWrite(A_2,LOW);
// digitalWrite(B_2,HIGH);
// analogWrite(PWM_2,leftServoValue);
  }
  else if(leftServoValue < 0)
  {
  digitalWrite(BOARD_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for a second
  digitalWrite(BOARD_LED, LOW);    // turn the LED off by making the voltage LOW
  delay(500);               // wait for a second
 // digitalWrite(A_2,HIGH);
 // digitalWrite(B_2,LOW);
 // analogWrite(PWM_2,abs(leftServoValue));

  }
  else if(leftServoValue == 0)
  {
    digitalWrite(BOARD_LED, LOW);

   // digitalWrite(A_2,HIGH);
   // digitalWrite(B_2,HIGH);
  
   }  
  
  
}
