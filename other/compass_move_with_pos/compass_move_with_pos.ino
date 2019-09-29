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

enum Lidar_Parse_State_Type {StartByteRecL = 1, IdleL = 0};
enum Lidar_Parse_State_Type Lidar_Parse_State = IdleL;
char Lidar_Data[50];
int Lidar_Cnt=0;

double xPos,yPos,zPos; //Position Data from GoT
float lidar_angle, lidar_distance;
double ref_Pos[2][2]={{6000.0,0.0},{2000.0,0.0}};
double HeadingX,HeadingY;
int done=0;

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
   Serial1.begin(115200);
   Serial2.begin(115200);

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

double inner_Product(double x1, double x2, double y1, double y2)
{
  return(x1*y1+x2*y2);
}

void Parse_Pos_Data(char inByte)
{
  switch(Pos_Parse_State) {
     case Idle:
         if(inByte==':')
         {
            Pos_Cnt=0;
            Pos_Data[Pos_Cnt]=inByte;
            Pos_Parse_State=StartByteRec;
            Pos_Cnt++;
         }
         break;
     case StartByteRec:
            Pos_Data[Pos_Cnt]=inByte;
            if(inByte==';')
            {
              my_sscanf(Pos_Data);
              Pos_Parse_State=Idle;
            }
            Pos_Cnt++;
            Pos_Data[Pos_Cnt]=0; //zero-terminated
  }
}

int my_str_lgth(char *instring)
{
  int cnt=0;
  while(instring[cnt]!=0)
     cnt++;
  return(cnt);
}

void my_sscanf(char *inbuffer)
{

  char *pch;
  int str_ptr=0;
  pch = strtok (&inbuffer[1],",");
  str_ptr+=my_str_lgth(pch)+1;
  xPos=atof(pch);
  pch = strtok (&inbuffer[1+str_ptr],",");
  str_ptr+=my_str_lgth(pch)+1;
  yPos=atof(pch);
  pch = strtok (&inbuffer[1+str_ptr],";");
  zPos=atof(pch);

              Serial.print("xPos: ");
              Serial.println(xPos);
              Serial.print("yPos: ");
              Serial.println(yPos);
              Serial.print("zPos: ");
              Serial.println(zPos);
              Serial.print("StartPos: ");
              Serial.print(ref_Pos[0][0]);
              Serial.print(',');
              Serial.print(ref_Pos[0][1]);
              Compute_Heading_Pt();
}

void Parse_Lidar_Data(char inByte)
{
  switch(Lidar_Parse_State) {
     case IdleL:
         if(inByte==':')
         {
            Lidar_Cnt=0;
            Lidar_Data[Lidar_Cnt]=inByte;
            Lidar_Parse_State=StartByteRecL;
            Lidar_Cnt++;
         }
         break;
     case StartByteRecL:
            Lidar_Data[Lidar_Cnt]=inByte;
            if(inByte==';')
            {
              my_sscanf_lidar(Lidar_Data);
              Lidar_Parse_State=IdleL;
            }
            Lidar_Cnt++;
            Lidar_Data[Lidar_Cnt]=0; //zero-terminated
  }
}

void my_sscanf_lidar(char *inbuffer)
{

  char *pch;
  int str_ptr=0;
  pch = strtok (&inbuffer[1],",");
  str_ptr+=my_str_lgth(pch)+1;
  lidar_angle=atof(pch);
  pch = strtok (&inbuffer[1+str_ptr],";");
  str_ptr+=my_str_lgth(pch)+1;
  lidar_distance=atof(pch);
              Serial.print("Angle: ");
              Serial.print(lidar_angle);
              Serial.print("\t");
              Serial.print("Distance: ");
              Serial.println(lidar_distance);
}

void Compute_Heading_Pt()
{
  double a;
  a=inner_Product(ref_Pos[0][0]-xPos, ref_Pos[0][1]-yPos, ref_Pos[0][0]-ref_Pos[1][0], ref_Pos[0][1]-ref_Pos[1][1]);
  a=a/inner_Product(ref_Pos[0][0]-ref_Pos[1][0], ref_Pos[0][1]-ref_Pos[1][1], ref_Pos[0][0]-ref_Pos[1][0], ref_Pos[0][1]-ref_Pos[1][1]);
  Serial.print("a: ");
  a+=+0.1;
  if(a>1)
     a=1;
  if(a<0)
     a=0;
  Serial.println(a);
  HeadingX=a*ref_Pos[1][0]+(1-a)*ref_Pos[0][0];
  HeadingY=a*ref_Pos[1][1]+(1-a)*ref_Pos[0][1];
  Serial.print("HX: ");
  Serial.println(HeadingX);
  Serial.print("HY: ");
  Serial.println(HeadingY);

  Serial.print("Mag:");
Serial.print(mx_cal);
Serial.print(',');
Serial.println(my_cal);

  Serial.print("Out:");
Serial.print(wA);
Serial.print(',');
Serial.print(outA);
Serial.print(':');
Serial.print(wB);
Serial.print(',');
Serial.println(outB);


}

void loop() {
  float crosspr,ws,wd;
  float refX=0.9848,refY=-0.1736;
//  float wA,wB; //commanded wheel speeds
//  int outA,outB;
  float outScale=60;

 // ws=5;

   //accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
   mag.getHeading(&mx, &my, &mz);
   if (Serial1.available()) {
      char inByte = Serial1.read();
      Parse_Pos_Data(inByte);
   }

   if (Serial2.available()) {
      char inByte2 = Serial2.read();
      Parse_Lidar_Data(inByte2);
   }
   //Serial.println(Serial2.available() ? "GOT SERIAL_2" : "NO");
   //Compute_Heading_Pt();



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

//Insert here command velocity 0 if LIDAR detects obstacle
//Connect got_teensy RX to lidar_teensy TX Serial2?
//Serial: 0 (RX) and 1 (TX); Serial 1: 19 (RX) and 18 (TX); Serial 2: 17 (RX) and 16 (TX); Serial 3: 15 (RX) and 14 (TX). Used to receive (RX) and transmit (TX) TTL serial data. Pins 0 and 1 are also connected to the corresponding pins of the ATmega16U2 USB-to-TTL Serial chip.
//Parse distance and angle

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




//Serial.print("Mag:");
//Serial.print(mx);
//Serial.print(',');
//Serial.print(my);
//Serial.print(',');
//Serial.print(mx_cal);
//Serial.print(',');
//Serial.println(my_cal);
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


  digitalWrite(InAPin1, InA1);
  digitalWrite(InBPin1, InB1);
  analogWrite(analogOutPin1, outA);
  //analogWrite(analogOutPin1, 150.0);

  digitalWrite(InAPin2, InA2);
  digitalWrite(InBPin2, InB2);
  analogWrite(analogOutPin2, outB);
  //analogWrite(analogOutPin2, 0);
  //delay (50);
}
