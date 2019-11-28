// This sketch code is based on the RPLIDAR driver library provided by RoboPeak
#include <RPLidar.h>

// You need to create an driver instance
RPLidar lidar;

#define RPLIDAR_MOTOR 13 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal

float distance = 0;
float angle    = 0;
//bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
//byte  quality  = lidar.getCurrentPoint().quality; //quality of the curre


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

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial1.begin(115200);

  Serial2.begin(115200); //LIDAR

  lidar.begin(Serial1);

  // set pin modes
  pinMode(RPLIDAR_MOTOR, OUTPUT);

  while(1==0)
  {
    for(int i=0;i<21;i++)
     Serial.write(test_bytes[i]);
     delay(100);
  }
}

bool compute_checksum()
{
  int i;
  long sum1=0;
  long sum2=0;
  int check1=0;
  int check2=0;
  //byte inBytess[]={163,200,19,74,88};
  //Serial.println("compute_checksum");
//  Serial.println(ByteCnt);

  for(i=1;i<ByteCnt-2;i++)
  {
    //Serial.println(inBytes[i]);
    sum1+=(int)inBytes[i] & 0xFF;
    sum1=sum1;
    sum2+=sum1;
  }
  sum1=sum1%255;
  sum2=sum2%255;
  check1 = 255 - ((sum1+sum2)%255);
  check2 = 255 - ((sum1+check1)%255);

//  Serial.println(sum1);
//  Serial.println(sum2);

  if(sum1==(int)inBytes[ByteCnt-2] && sum2==(int)inBytes[ByteCnt-1])
  {
    //Serial.println("true");
     return(true);
  }
  else
  {
    //Serial.println("false");
     return(false);
  }
}

void wr_bytes(int ByteCnt)
{
  int i;
  for (i=0;i<ByteCnt;i++)
  {
     Serial.print((int)inBytes[i]);
     Serial.print(',');
  }
  Serial.println(' ');
}

void Extract_Data()
{
//  Serial.println("Extract_Data");
 // wr_bytes(ByteCnt);
  //data_type* data_ptr;
  int Length = inBytes[0];
  int No_Of_Data = floor((Length-6)/6);
  int i;
//  Serial.println(Length);
//  Serial.println(No_Of_Data);
  for(i=0;i<No_Of_Data;i++)
  {
    data_ptr=(data_type*) &inBytes[8+i*6];
    //Estimate_position();
//    Serial.println(data_ptr->rssi);
//    Serial.println(data_ptr->TxID_Low);
//    Serial.println(data_ptr->TxID_Middle);
//    Serial.println(data_ptr->TxID_High);
//    int ID=(int)data_ptr->TxID_High;
//  ID=256*ID+(int)data_ptr->TxID_Middle;
//  ID=256*ID+(int)data_ptr->TxID_Low;
//  Serial.println(ID);
//    Serial.println(data_ptr->TxID_time_Low);
//    Serial.println(data_ptr->TxID_time_High);
//    Serial.println((((int)data_ptr->TxID_time_High)*256+(int)data_ptr->TxID_time_Low)*0.343);
  }
  ByteCnt=0;
}

//void Estimate_position(int fake_ID,double fake_dist)
void Estimate_position()
{
  //Serial.println("Estimate_position");
  //compute ID
  int ID=(int)data_ptr->TxID_High;
  ID=256*ID+(int)data_ptr->TxID_Middle;
  ID=256*ID+(int)data_ptr->TxID_Low;
//  Serial.print("ID: ");
//  Serial.println(ID);
  //ID=fake_ID;
  //find ID in IP/Pos List;
  int index=-1;
  for(int i=0;i<6;i++)
  {
    if(ID_POS_List[i][0]==ID)
    {
       index=i;
       //Serial.println(index);
//       Serial.println(i);
    }
  }
  //if found
  //Serial.println(index);
  if(index>-1)// && data_ptr->rssi>190)
  {
    //compute distance between the so far estimated pt and transmitter
    double dp=(double)pow(x_est-(double)ID_POS_List[index][1],2);
    dp+=(double)pow(y_est-(double)ID_POS_List[index][2],2);
    dp+=(double)pow(z_est-(double)ID_POS_List[index][3],2);
    dp=sqrt(dp);

    //compute measured distance
    double meas_dist=(double)data_ptr->TxID_time_High;
    meas_dist=256*meas_dist+(double)data_ptr->TxID_time_Low;
    //Serial.print("Meas_Dist: ");
    //Serial.println(meas_dist);
    meas_dist*=0.343; //Speed of light in mm pr uS
    if(meas_dist>11000 || meas_dist<1000)
       return;
    //Serial.println(meas_dist);
    //meas_dist=pow(fake_dist,2)-pow((double)ID_POS_List[index][3],2); //Henrik: skal tages væk under kørsel
    //meas_dist=sqrt(meas_dist);
    double lambda=1-dp/meas_dist;
    //update estimate
//    if(abs(lambda-1)>0.01)
//    {
    double x_est_new=x_est/(1-lambda)-(double)ID_POS_List[index][1]*lambda/(1-lambda);
    double y_est_new=y_est/(1-lambda)-(double)ID_POS_List[index][2]*lambda/(1-lambda);
    double z_est_new=z_est/(1-lambda)-(double)ID_POS_List[index][3]*lambda/(1-lambda);
    //}
    //xc=xp/(1-lambda)-y(:,j)*lambda/(1-lambda); //from MATLAB

    double dist_new=(double)pow(x_est-x_est_new,2);
    dist_new+=(double)pow(y_est-y_est_new,2);
    dist_new+=(double)pow(z_est-z_est_new,2);
    //if(dist_new<0.25e6)
    {
      x_est=x_est_new;
      y_est=y_est_new;
      z_est=z_est_new;
    }

    //print to MATLAB testprogram
 //   Serial.print(':');
//    Serial.print(ID);
//    Serial.print(',');
//    Serial.print(meas_dist);
//    Serial.print(',');
//    Serial.print(index);
//    Serial.print(',');
    Serial.print(':');
    Serial.print(x_est);
    Serial.print(',');
    Serial.print(y_est);
    Serial.print(',');
    Serial.print(z_est);
    Serial.println(";");

    Serial1.print(':');
    Serial1.print(x_est);
    Serial1.print(',');
    Serial1.print(y_est);
    Serial1.print(',');
    Serial1.print(z_est);
    Serial1.print(',');
    Serial1.print(angle);
    Serial1.print(',');
    Serial1.print(distance);
    Serial1.println(";");
   //Serial.print(',');
   //Serial.print(data_ptr->rssi);
//    Serial.print(',');
//    Serial.print(index);
//    Serial.print(',');
//    Serial.print(ID);
//    Serial.print(',');
//    Serial.print(millis());
//    Serial.print(',');
//    if(cc==true)
//       Serial.print(1);
//    else
//       Serial.print(0);
    //Serial.println(":");
  }
}


void loop() {

  if (IS_OK(lidar.waitPoint())) {
    distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    angle    = lidar.getCurrentPoint().angle; //anglue value in degree

    int int_dist=(int) (distance/10);
    int int_angle = (int) (angle);
  } else {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
       // detected...
       lidar.startScan();

       // start motor rotating at max allowed speed
       analogWrite(RPLIDAR_MOTOR, 255);
       delay(1000);
    }
  }

  char inByte;
  //bool cc;
  //Serial.println("loop");
  // read from port 1, send to port 0:
//  Serial.print(':');
//  Serial.print(1.00);
//  Serial.print(',');
//  Serial.print(10.45);
//  Serial.print(',');
//  Serial.print(45.45);
//  Serial.println(':');
//  double dp=(double)pow(x_target-ID_POS_List[0][1],2);
//  dp+=(double)pow(y_target-ID_POS_List[0][2],2);
//  dp+=(double)pow(z_target-ID_POS_List[0][3],2);
//  dp=sqrt(dp);
//  Estimate_position(1001,dp);
//  delay(500);
//  dp=(double)pow(x_target-ID_POS_List[1][1],2);
//  dp+=(double)pow(y_target-ID_POS_List[1][2],2);
//  dp+=(double)pow(z_target-ID_POS_List[1][3],2);
//  dp=sqrt(dp);
//  Estimate_position(1002,dp);
//  delay(500);
//  dp=(double)pow(x_target-ID_POS_List[2][1],2);
//  dp+=(double)pow(y_target-ID_POS_List[2][2],2);
//  dp+=(double)pow(z_target-ID_POS_List[2][3],2);
//  dp=sqrt(dp);
//  Estimate_position(1003,dp);
//  delay(500);
//  dp=(double)pow(x_target-ID_POS_List[3][1],2);
//  dp+=(double)pow(y_target-ID_POS_List[3][2],2);
//  dp+=(double)pow(z_target-ID_POS_List[3][3],2);
//  dp=sqrt(dp);
//  Estimate_position(1004,dp);
//  delay(500);
//  dp=(double)pow(x_target-ID_POS_List[4][1],2);
//  dp+=(double)pow(y_target-ID_POS_List[4][2],2);
//  dp+=(double)pow(z_target-ID_POS_List[4][3],2);
//  dp=sqrt(dp);
//  Estimate_position(1005,dp);
//  delay(500);


  if (Serial1.available()) {
    char inByte = Serial1.read();
//    Serial.print((byte)inByte,HEX);
//    Serial.print(',');
//   if(test_cnt>20)
//     while(1==1);
//   inByte=test_bytes[test_cnt];
//   test_cnt++;
   switch(State) {
     case Idle:
        //Serial.println("Idle");
        //wr_bytes(ByteCnt);
        //Serial.println((byte)inByte,HEX);
        //Serial.println("-------- ");
        ByteCnt=0;
        if(inByte==StartByte)
           State=StartByteRec;
        break;
     case StartByteRec:
        //Serial.println("StartByteRec");
        //wr_bytes(ByteCnt);
        //Serial.println("-------- ");
        switch(inByte) {
           case StartByte:
              ByteCnt=0;
              State=Idle;
           break;
           case StopByte:
              //compute checksums and extract data
              cc=compute_checksum();
              //if(cc==true)
              if(1==1)
              {
                 Extract_Data();
                 Estimate_position();
              }
                 State=Idle;
           break;
           case Escape:
              State=EscapeRec;
           break;
           default:
              inBytes[ByteCnt++]=inByte;
        }
        break;
      case EscapeRec:
         //Serial.println("EscapeRec ");
         //wr_bytes(ByteCnt);
         //Serial.println("-------- ");
         State=StartByteRec;
         inBytes[ByteCnt++]=(char)(inByte-0x20);
   }
  }
}

