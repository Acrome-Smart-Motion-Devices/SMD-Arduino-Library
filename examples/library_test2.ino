#include "Acrome-SMD.h"

int baudrate = 115200;


Red red1(10, Serial, baudrate);
Red red2(20, Serial, baudrate);
Red red3(30, Serial, baudrate);


int torqueShow_counter = 1;


void setup() {
  Serial.begin(115200);


  red1.torqueEnable(1);
  red2.torqueEnable(1);
  red3.torqueEnable(1);
  delay(500);
  red1.torqueEnable(0);
  red2.torqueEnable(0);
  red3.torqueEnable(0);


}

void loop() {
  // put your main code here, to run repeatedly:
  while (torqueShow_counter){
    for(int i= 0; i<3 ; i++){
      red1.torqueEnable(1);
      delay(300);
      red1.torqueEnable(0);
      red2.torqueEnable(1);
      delay(300);
      red2.torqueEnable(0);
      red3.torqueEnable(1);
      delay(300);
      red3.torqueEnable(0);
    }

    for(int i= 0; i<2 ; i++){
      red1.torqueEnable(1);
      red2.torqueEnable(1);
      red3.torqueEnable(1);
      delay(600);
      red1.torqueEnable(0);
      red2.torqueEnable(0);
      red3.torqueEnable(0);
      delay(400);
    }
    red1.torqueEnable(1);
    red2.torqueEnable(1);
    red3.torqueEnable(1);
    delay(1200);

    break;
  }









}
