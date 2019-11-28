#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#include "got_serial.h"

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

const int analogOutPin1 = 10; // Analog output pin that the LED is attached to
int InBPin1 = 11;
int InAPin1 = 12;
boolean InA1 = LOW;
boolean InB1;

const int analogOutPin2 = 30; // Analog output pin that the LED is attached to
int InBPin2 = 26;
int InAPin2 = 28;
boolean InA2 = HIGH;
boolean InB2;

float wA,wB; //commanded wheel speeds
int outA,outB;

//enum Pos_Parse_State_Type {StartByteRec = 1, Idle = 0};
//enum Pos_Parse_State_Type Pos_Parse_State = Idle;
char Pos_Data[50];
int Pos_Cnt=0;


double xPos,yPos,zPos; //Position Data from GoT
float angle, distance;
double ref_Pos[2][2]={{6000.0,0.0},{2000.0,0.0}};
double HeadingX,HeadingY;
int done=0;

void setup() {
  delay(3000);
   pinMode(InAPin1, OUTPUT);
   pinMode(InBPin1, OUTPUT);
   pinMode(InAPin2, OUTPUT);
   pinMode(InBPin2, OUTPUT);

   Wire.begin();
   accelgyro.setI2CMasterModeEnabled(false);
   accelgyro.setI2CBypassEnabled(true) ;
   accelgyro.setSleepEnabled(false);

   Serial.begin(115200);
   Serial1.begin(115200);

   // initialize device
   Serial.println("Initializing I2C devices...");
   accelgyro.initialize();
   mag.initialize();
   Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

   // verify connection
   Serial.println("Testing device connections...");
   Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

   // configure Arduino LED for
   //pinMode(LED_PIN, OUTPUT);
   InB1 = not InA1;
   InB2 = not InA2;

  //Neede for GoT data
  while(1==0)
  {
    for(int i=0;i<21;i++)
     Serial.write(test_bytes[i]);
     delay(100);
  }
}

double inner_Product(double x1, double x2, double y1, double y2)
{
  return(x1*y1+x2*y2);
}

//Pos_Data[Pos_Cnt]
// xPos, yPos, zPos;


void Compute_Heading_Pt()
{
  xPos = x_est;
  yPos = y_est;
  zPos = z_est;
  
  double a;
  a=inner_Product(ref_Pos[0][0]-xPos, ref_Pos[0][1]-yPos, ref_Pos[0][0]-ref_Pos[1][0], ref_Pos[0][1]-ref_Pos[1][1]);
  a=a/inner_Product(ref_Pos[0][0]-ref_Pos[1][0], ref_Pos[0][1]-ref_Pos[1][1], ref_Pos[0][0]-ref_Pos[1][0], ref_Pos[0][1]-ref_Pos[1][1]);
//  Serial.print("a: ");
  a+=+0.1;
  if(a>1)
     a=1;
  if(a<0)
     a=0;
//  Serial.println(a);
  HeadingX=a*ref_Pos[1][0]+(1-a)*ref_Pos[0][0];
  HeadingY=a*ref_Pos[1][1]+(1-a)*ref_Pos[0][1];
//  Serial.print("HX: ");
//  Serial.print(HeadingX);
//  Serial.print("\t HY: ");
//  Serial.println(HeadingY);

//  Serial.print("Mag:");
//  Serial.print(mx_cal);
//  Serial.print(',');
//  Serial.println(my_cal);

//  Serial.print("Out:");
//  Serial.print(wA);
//  Serial.print(',');
//  Serial.print(outA);
//  Serial.print(':');
//  Serial.print(wB);
//  Serial.print(',');
//  Serial.println(outB);
Serial.print("xPos: ");
Serial.print(xPos);
Serial.print("\t yPos: ");
Serial.print(yPos);
Serial.print("\t zPos: ");
Serial.println(zPos);

Serial.print("\t ref_Pos[0][0]: ");
Serial.print(ref_Pos[0][0]);
Serial.print("\t ref_Pos[0][1]: ");
Serial.print(ref_Pos[0][1]);
Serial.print("\t ref_Pos[1][0]: ");
Serial.print(ref_Pos[1][0]);
Serial.print("\t ref_Pos[1][1]: ");
Serial.println(ref_Pos[1][1]);

Serial.print("\t mx_cal: ");
Serial.print(mx_cal);
Serial.print("\t my_cal: ");
Serial.println(my_cal);

}

void loop() {
  float crosspr,ws,wd;
  float refX=0.9848,refY=-0.1736;
  //wA,wB; //commanded wheel speeds
  float outScale=60;

 // ws=5;

   //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   mag.getHeading(&mx, &my, &mz);

   Get_Position();
   Compute_Heading_Pt();

mx_cal=(float)mx-(float)(MAGXMAX+MAGXMIN)/2;
mx_cal=2*mx_cal/((float)(MAGXMAX-MAGXMIN));

my_cal=(float)my-(float)(MAGYMAX+MAGYMIN)/2;
my_cal=2*my_cal/((float)(MAGYMAX-MAGYMIN));

double my_temp=my_cal;
my_cal=-mx_cal;
mx_cal=-my_temp;

refX=HeadingX-xPos;
refY=HeadingY-yPos;
double refLength=sqrt(pow(refX,2)+pow(refY,2));

refX=refX/refLength;
refY=refY/refLength;

Serial.print("\t refX: ");
Serial.print(refX);
Serial.print("\t refY: ");
Serial.print(refY);
Serial.print("\t HeadinX: ");
Serial.print(HeadingX);
Serial.print("\t HeadinY: ");
Serial.println(HeadingY);



crosspr=refX*my_cal-refY*mx_cal; //compute cross product between desired heading and heading
crosspr=-crosspr/(abs(crosspr)+0.1);

cross_int+=0*crosspr;
wd=2.0*crosspr;//+0.01*cross_int;
ws=5;
if(abs(ref_Pos[1][0]-xPos)<1500.0 && abs(ref_Pos[1][1]-xPos)<1500.0)
{
   done=1;
}
if(done==1)
{
  ws=0;
  wd=0;
}

float out_diff=(wd-0.97*wd_old)/(1-0.97);
wd_old=wd;
//out_diff=0.0;
wA=(ws+out_diff)/2;
wB=(ws-out_diff)/2;

if(wA>=0)
{
  InA1 = HIGH;
  outA=(int)(outScale*wA);
  if(outA>255)
     outA=255;
}
else
{
  InA1 = LOW;
  outA=(int)(-outScale*wA);
  if(outA>255)
     outA=255;
}

if(wB>=0)
{
  InA2 = HIGH;
  outB=(int)(outScale*wB);
  if(outB>255)
     outB=255;
}
else
{
  InA2 = LOW;
  outB=(int)(-outScale*wB);
  if(outB>255)
     outB=255;
}

InB1 = not InA1;
InB2 = not InA2;


//digitalWrite(InAPin1, InA1);
//digitalWrite(InBPin1, InB1);
//analogWrite(analogOutPin1, outA);

//digitalWrite(InAPin2, InA2);
//digitalWrite(InBPin2, InB2);
//analogWrite(analogOutPin2, outB);
//delay (50);
}
