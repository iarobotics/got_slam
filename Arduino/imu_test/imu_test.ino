#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

MPU6050 accelgyro;
HMC5883L mag;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float mx_cal, my_cal, mz_cal;
float cross_int=0;
float wd_old=0;

const int MAGXMAX = -110;
const int MAGXMIN = -582;
const int MAGYMAX = 395;
const int MAGYMIN = -92;

#define LED_PIN 13
bool blinkState = false;

const int analogOutPin1 = 6; // Analog output pin that the LED is attached to
int InBPin1 = 10;
int InAPin1 = 9;
boolean InA1 = LOW;
boolean InB1;

const int analogOutPin2 = 4; // Analog output pin that the LED is attached to
int InBPin2 = 3;
int InAPin2 = 5;
boolean InA2 = HIGH;
boolean InB2;
  float wA,wB; //commanded wheel speeds
  int outA,outB;

enum Pos_Parse_State_Type {StartByteRec = 1, Idle = 0};
enum Pos_Parse_State_Type Pos_Parse_State = Idle;
char Pos_Data[50];
int Pos_Cnt=0;


double xPos,yPos,zPos; //Position Data from GoT
float angle, distance;
double ref_Pos[2][2]={{6000.0,0.0},{2000.0,0.0}};
double HeadingX,HeadingY;
int done=0;

void setup() {

   Wire.begin();
   accelgyro.setI2CMasterModeEnabled(false);
   accelgyro.setI2CBypassEnabled(true) ;
   accelgyro.setSleepEnabled(false);

   Serial.begin(115200);

   // initialize device
   Serial.println("Initializing I2C devices...");
   accelgyro.initialize();
   mag.initialize();
   Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

   // verify connection
   Serial.println("Testing device connections...");
   Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop()
{

   //Serial.println(mag.testConnection() ? "HMC5883L connection OK" : "HMC5883L connection FAIL");
   //Serial.println(accelgyro.testConnection() ? "MPU6050 connection OK" : "MPU6050 connection FAIL");

  
mag.getHeading(&mx, &my, &mz);



mx_cal=(float)mx-(float)(MAGXMAX+MAGXMIN)/2;
mx_cal=2*mx_cal/((float)(MAGXMAX-MAGXMIN));

my_cal=(float)my-(float)(MAGYMAX+MAGYMIN)/2;
my_cal=2*my_cal/((float)(MAGYMAX-MAGYMIN));


Serial.print("Mag:");
Serial.print(mx);
Serial.print(',');
Serial.print(my);
Serial.print(',');
Serial.print(mx_cal);
Serial.print(',');
Serial.println(my_cal);
//
////outA=(int)(7*wA);
////outB=(int)(7*wB);
//
//Serial.print("Out:");
//Serial.print(wA);
//Serial.print(',');
//Serial.print(outA);
//Serial.print(':');
//Serial.print(wB);
//Serial.print(',');
//Serial.println(outB);
}
