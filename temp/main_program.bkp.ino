#include "Wire.h"
#include "I2Cdev.h"
//#include "MPU6050.h"
#include "HMC5883L.h"

#include "got_serial.h"
#include "MPU6050_6Axis_MotionApps20.h"

////ROS variables
#include <ros.h>
//#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>

//#define OUTPUT_QUATERNION
//#define PRINT_POS
//#define PRINT_REF
//#define ROS_MOVE_CMD
#define ROS_TELEOP
#define SUB_TOPIC "/turtle1/cmd_vel"

ros::NodeHandle nh;

//ros::Publisher chatter("chatter", &str_msg);
geometry_msgs::Pose pose_msg;
ros::Publisher robot_pose("robot_pose", &pose_msg);

#ifdef ROS_MOVE_CMD
  boolean move_robot = false;
  
  void messageCb(const std_msgs::Int8& msg)
  {
    if(msg.data == 1)
      move_robot = true;   //blink the led
  else
    move_robot=false;   //turn off the led
  }
   
  //ros::Subscriber sub("LED", &messageCb);
  ros::Subscriber<std_msgs::Int8> sub("LED", &messageCb);
#endif
////////////////

#ifdef ROS_TELEOP
  boolean move_robot = false;
  
  void teleopCb(const geometry_msgs::Twist& msg)
  {
    Serial.print(msg.linear.x);
    Serial.print(', ');
    Serial.print(msg.linear.y);
    Serial.print(', ');
    Serial.print(msg.linear.z);
    Serial.print('; ');

    Serial.print(msg.angular.x);
    Serial.print(', ');
    Serial.print(msg.angular.y);
    Serial.print(', ');
    Serial.println(msg.angular.z);

    #ifdef TEMP
    if (msg.angular.z < 0)
    {
      move_robot = true;

    }
    else if(msg.angular.z > 0)
    {
      move_robot = false;
      outA = 0;
      outB = 0;
    }

    if(move_robot)
    {
      if(msg.linear.x > 0)
      {
        //move_robot = true;
        InA1 = LOW;
        InB1 = not InA1;
        outA = 100;
      }
      else
      {
        InA2 = HIGH;
        InB2 = not InA2;
        outB = 100;
        //move_robot = false;
      }
    }
    #endif
    
  }
   
  ros::Subscriber<geometry_msgs::Twist> teleop_sub(SUB_TOPIC, &teleopCb);

#endif

MPU6050 mpu;
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

#define BATT_PIN 23
int battery_level = 0;

const int analogOutPin1 = 10; // Analog output pin that the LED is attached to
//int InBPin1 = 11;
//int InAPin1 = 12;

//Reversed
int InBPin1 = 12;
int InAPin1 = 11;

boolean InA1 = LOW;
boolean InB1;

const int analogOutPin2 = 30; // Analog output pin that the LED is attached to
//int InBPin2 = 26;
//int InAPin2 = 28;

//Reversed
int InBPin2 = 28;
int InAPin2 = 26;
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
//double ref_Pos[2][2]={{6000.0,0.0},{2000.0,0.0}};
double ref_Pos[2][2]={{9000.0,0.0},{6000.0,0.0}};
double HeadingX,HeadingY;
int done=0;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container

void setup() {
  delay(3000);
  ///ROS
    nh.initNode();
    nh.advertise(robot_pose);

    #ifdef ROS_MOVE_CMD
      nh.subscribe(sub);
    #endif

    #ifdef ROS_TELEOP
      nh.subscribe(teleop_sub);
    #endif
  //////
   pinMode(InAPin1, OUTPUT);
   pinMode(InBPin1, OUTPUT);
   pinMode(InAPin2, OUTPUT);
   pinMode(InBPin2, OUTPUT);

   Wire.begin();
   Wire.setClock(400000); //TODO: Comment out if having compilation difficulties
   mpu.setI2CMasterModeEnabled(false);
   mpu.setI2CBypassEnabled(true) ;
   mpu.setSleepEnabled(false);

   Serial.begin(115200);
   Serial1.begin(115200);

   // initialize device
   Serial.println("Initializing I2C devices...");
   mpu.initialize();
   mag.initialize();
   Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

   // verify connection
   Serial.println("Testing device connections...");
   Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

 #ifdef OUTPUT_QUATERNION
   Serial.println(F("Initializing DMP..."));
   devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
 #endif

   // configure Arduino LED for
   //pinMode(LED_PIN, OUTPUT);
   InB1 = not InA1;
   InB2 = not InA2;

  //Needed for GoT data
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

//  Serial.print("Out:");
//  Serial.print(wA);
//  Serial.print(',');
//  Serial.print(outA);
//  Serial.print(':');
//  Serial.print(wB);
//  Serial.print(',');
//  Serial.println(outB);

#ifdef PRINT_POS
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
#endif

//  str_msg.data = xPos;
//  chatter.publish( &str_msg );
// pose_msg.position = xPos, yPos, zPos;
 pose_msg.position.x = xPos;
 pose_msg.position.y = yPos;
 pose_msg.position.z = zPos;

 pose_msg.orientation.x = q.x;
 pose_msg.orientation.y = q.y;
 pose_msg.orientation.z = q.z;
 pose_msg.orientation.w = q.w;
 robot_pose.publish( &pose_msg );

}

void print_battery()
{
  int maximum = analogRead(BATT_PIN);
  delay(1);
  int minimum = analogRead(BATT_PIN);
  delay(1);
  for(int i=0;i<2000;i++)
  {
    int batt_lvl;
    batt_lvl = analogRead(BATT_PIN);
    delay(1);
    if (batt_lvl > maximum)
    {
      maximum = batt_lvl;
    }
    else if (batt_lvl < minimum)
    {
      minimum = batt_lvl;
    }
  }
  Serial.print(minimum);
  Serial.print('\t');
  Serial.println(maximum);
}

int get_battery()
{
    int batt_lvl;
    batt_lvl = analogRead(BATT_PIN);
    return batt_lvl;
}

void loop() {
////////////////////////
  #ifdef OUTPUT_QUATERNION
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  if (fifoCount >= 1024) 
  {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else{

        // read a packet from FIFO
  while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
  }
        #ifdef OUTPUT_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            // Serial.print("quat\t");
            // Serial.print(q.w);
            // Serial.print("\t");
            // Serial.print(q.x);
            // Serial.print("\t");
            // Serial.print(q.y);
            // Serial.print("\t");
            // Serial.println(q.z);
        #endif
    }
  #endif
  
  float crosspr,ws,wd;
  float refX=0.9848,refY=-0.1736;
  //wA,wB; //commanded wheel speeds

  battery_level = get_battery();
  //Serial.println(battery_level);
  //print_battery();
  //float outScale=60;
  float outScale = (1024/battery_level) *15;
  

 // ws=5;

   //mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
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

#ifdef PRINT_REF
 Serial.print("\t refX: ");
  Serial.print(refX);
  Serial.print("\t refY: ");
  Serial.print(refY);
  Serial.print("\t HeadinX: ");
  Serial.print(HeadingX);
  Serial.print("\t HeadinY: ");
  Serial.println(HeadingY);
#endif


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

//  Serial.print("wA: ");
//  Serial.print(wA);
//  Serial.print("\t outA: ");
//  Serial.println(outA);

//  Serial.print("wB: ");
//  Serial.print(wB);
//  Serial.print("\t outB: ");
//  Serial.println(outB);

if (battery_level < 300)
{
  outA = 0;
  outB = 0;
}


digitalWrite(InAPin1, InA1);
digitalWrite(InBPin1, InB1);
analogWrite(analogOutPin1, outA);

digitalWrite(InAPin2, InA2);
digitalWrite(InBPin2, InB2);
analogWrite(analogOutPin2, outB);

nh.spinOnce();
//delay (50);
}
