#include "got_serial.h"

#define GET_GOT_POS

void setup() {

  delay(3000);
  Serial.begin(115200);
  #ifdef GET_GOT_POS
    Serial1.begin(115200); // Required for GOT position data

    while(1==0)
    {
      for(int i=0;i<21;i++)
       Serial.write(test_bytes[i]);
       delay(100);
    }
  #endif

}

void loop() {
  Get_Position(); //double x_est,y_est,z_est defined in got_serial.h
  print_got_pos();

}

void print_got_pos() {
   Serial.print("x_got: ");
   Serial.print(x_est);
   Serial.print("\t y_got: ");
   Serial.println(y_est);
}
