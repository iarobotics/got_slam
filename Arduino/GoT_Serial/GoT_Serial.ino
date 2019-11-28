


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

  for(i=1;i<ByteCnt-2;i++)
  {
    sum1+=(int)inBytes[i] & 0xFF;
    sum1=sum1;
    sum2+=sum1;
  }
  sum1=sum1%255;
  sum2=sum2%255;
  check1 = 255 - ((sum1+sum2)%255);
  check2 = 255 - ((sum1+check1)%255);
  
  if(sum1==(int)inBytes[ByteCnt-2] && sum2==(int)inBytes[ByteCnt-1])
  {
     return(true);
  }
  else
  {
     return(false);
  }
}

void Extract_Data()
{
  int Length = inBytes[0];
  int No_Of_Data = floor((Length-6)/6);
  int i;

  for(i=0;i<No_Of_Data;i++)
  {
    data_ptr=(data_type*) &inBytes[8+i*6];
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
    Serial1.println(";");
  }    
}


void loop() {
  char inByte;
  
  if (Serial1.available()) {
    char inByte = Serial1.read();

   switch(State) {
     case Idle:
        ByteCnt=0;
        if(inByte==StartByte)
           State=StartByteRec;
        break; 
     case StartByteRec:
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
         State=StartByteRec;
         inBytes[ByteCnt++]=(char)(inByte-0x20); 
   }
  }
}

