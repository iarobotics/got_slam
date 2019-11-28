
enum Pos_Parse_State_Type {StartByteRec = 1, Idle = 0};
enum Pos_Parse_State_Type Pos_Parse_State = Idle;
char Pos_Data[50];
int Pos_Cnt=0;


float angle, distance;

void setup() {

   Serial.begin(115200);
   Serial2.begin(115200);

   Serial.println("Initializing I2C devices...");

}


void Parse_Pos_Data(char inByte)
{
  switch(Pos_Parse_State) {
     case Idle:
         if(inByte=='&')
         {
            Pos_Cnt=0;
            Pos_Data[Pos_Cnt]=inByte;
            Pos_Parse_State=StartByteRec;
            Pos_Cnt++;
         }
         break;
     case StartByteRec:
            Pos_Data[Pos_Cnt]=inByte;
            if(inByte=='$')
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

  pch = strtok (&inbuffer[1+str_ptr],",");
  str_ptr+=my_str_lgth(pch)+1;
  angle=atof(pch);
  
  pch = strtok (&inbuffer[1+str_ptr],"$");
  str_ptr+=my_str_lgth(pch)+1;
  distance=atof(pch);

              Serial.print("angle: ");
              Serial.println(angle);
              Serial.print(" distance: ");
              Serial.println(distance);
}


void loop() {
   if (Serial2.available()) {
      char inByte = Serial.read();
      Parse_Pos_Data(inByte);
   }


}

