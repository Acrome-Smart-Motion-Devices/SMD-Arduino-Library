// ---------------------------------------------------------------------------
// Example of Acrome SMD Red Position Control with Arduino.
// ---------------------------------------------------------------------------

#include <Acrome-SMD.h>

#define BAUDRATE    115200
#define ID          0 


Red red1(ID, Serial, BAUDRATE);

void setup() {
  red1.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  red1.torqueEnable(1);
  delay(300);
  red1.torqueEnable(0);
  delay(300);
}