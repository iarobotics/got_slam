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

///GoT related
byte test_bytes[]{0x02, 13, 1, 0x10, 2, 0x10, 2, 0, 88, 50, 1, 90, 5, 0, 0, 88, 0x10, 2, 74, 216, 0x03};
byte test_bytes_long[]{0x02, 19, 1, 0x10, 2, 0x10, 2, 0, 88, 50, 1, 90, 5, 0, 0, 88, 0x10, 2, 90, 5, 0, 0, 88, 0x10, 2, 74, 216, 0x03};

//<Satellite Id="42497" DistanceTo00="4349" DistanceToX0="4124" DistanceToXY="4341" PositionX="1645" PositionY="-462" PositionZ="4000" />
//      <Satellite Id="42498" DistanceTo00="6010" DistanceToX0="5856" DistanceToXY="5481" PositionX="1591" PositionY="4193" PositionZ="3999" />
//      <Satellite Id="42867" DistanceTo00="8284" DistanceToX0="7739" DistanceToXY="8033" PositionX="6195" PositionY="0" PositionZ="5499" />
//      <Satellite Id="42928" DistanceTo00="12291" DistanceToX0="11679" DistanceToXY="11771" PositionX="10157" PositionY="4204" PositionZ="5499" />
//      <Satellite Id="42929" DistanceTo00="12150" DistanceToX0="11504" DistanceToXY="11974" PositionX="10568" PositionY="-2389" PositionZ="5499" />

//int ID_POS_List[5][4]={{42497,1645,-462,4000},{42498,1591,4193,3999},{42867,6195,0,5499},{42928,10157,4204,5499},{42929,10568,-2389,5499}};
int ID_POS_List[6][4]={{42929,1645,-462,4500},{42531,1591,4193,4500},{42498,1591,4193,3999},{42867,6195,0,5499},{42928,10157,4204,5499},{42497,10568,-2389,5499}};

enum State_Type {EscapeRec = 2, StartByteRec = 1, Idle = 0};
enum State_Type State = Idle;
byte inBytes[25];
int ByteCnt;
int test_cnt=0;
bool cc;

double x_est=0,y_est=0,z_est=0;
double x_target=500,y_target=500,z_target=0;


typedef struct data  {
  byte  rssi;
  byte  TxID_Low;
  byte  TxID_Middle;
  byte  TxID_High;
  byte  TxID_time_Low;
  byte  TxID_time_High;
} data_type;

data_type* data_ptr;

enum Byte_Type {Escape = 0x10, StartByte = 0x02, StopByte = 0x03};
/////////////

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
   //pinMode(LED_PIN, OUTPUT);
   InB1 = not InA1;
   InB2 = not InA2;
}

double inner_Product(double x1, double x2, double y1, double y2)
{
  return(x1*y1+x2*y2);
}

//Pos_Data[Pos_Cnt]

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
  pch = strtok (&inbuffer[1+str_ptr],",");
  zPos=atof(pch);

  pch = strtok (&inbuffer[1+str_ptr],",");
  angle=atof(pch);
  pch = strtok (&inbuffer[1+str_ptr],";");
  angle=atof(pch);

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

              Serial.print(" angle: ");
              Serial.println(angle);
              Serial.print(" distance: ");
              Serial.println(angle);
              Compute_Heading_Pt();
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
