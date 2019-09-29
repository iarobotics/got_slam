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

const int MAGXMAX = -154;
const int MAGXMIN = -538;
const int MAGYMAX = 362;
const int MAGYMIN = -34;

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

void setup() {

   pinMode(InAPin1, OUTPUT);
   pinMode(InBPin1, OUTPUT);
   pinMode(InAPin2, OUTPUT);
   pinMode(InBPin2, OUTPUT);
  
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

   // configure Arduino LED for
   pinMode(LED_PIN, OUTPUT);
   InB1 = not InA1;
   InB2 = not InA2;
}

void loop() {
  float crosspr,ws,wd;
  float refX=0.9848,refY=-0.1736;
  float wA,wB; //commanded wheel speeds
  int outA,outB;
  float outScale=50;
  
  ws=2; //turn around center axis
   
   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   mag.getHeading(&mx, &my, &mz);



mx_cal=(float)mx-(float)(MAGXMAX+MAGXMIN)/2;
mx_cal=2*mx_cal/((float)(MAGXMAX-MAGXMIN));

my_cal=(float)my-(float)(MAGYMAX+MAGYMIN)/2;
my_cal=2*my_cal/((float)(MAGYMAX-MAGYMIN));

crosspr=refX*my_cal-refY*mx_cal; //compute cross product between desired heading and heading
cross_int+=0.1*crosspr;
wd=1.0*crosspr+0.4*cross_int;
float out_diff=(wd-0.97*wd_old)/(1-0.97);
wd_old=wd;
//out_diff=0.0;
wA=(ws+out_diff)/2;
wB=(ws-out_diff)/2;

if(wA>=0)
{
  InA1 = HIGH;
  outA=(int)(outScale*wA);
}
else
{
  InA1 = LOW;
  outA=(int)(-outScale*wA);
}

if(wB>=0)
{
  InA2 = HIGH;
  outB=(int)(outScale*wB);
}
else
{
  InA2 = LOW;
  outB=(int)(-outScale*wB);
}

InB1 = not InA1;
InB2 = not InA2;




Serial.print("Mag:");
Serial.print(mx_cal); 
Serial.print(','); 
Serial.println(my_cal);

//outA=(int)(7*wA);
//outB=(int)(7*wB);

Serial.print("Out:");
Serial.print(wA); 
Serial.print(','); 
Serial.print(outA); 
Serial.print(':'); 
Serial.print(wB);
Serial.print(','); 
Serial.println(outB);
   

  digitalWrite(InAPin1, InA1);
  digitalWrite(InBPin1, InB1);
  analogWrite(analogOutPin1, outA);

  digitalWrite(InAPin2, InA2);
  digitalWrite(InBPin2, InB2);
  analogWrite(analogOutPin2, outB);
  delay (50);
}
