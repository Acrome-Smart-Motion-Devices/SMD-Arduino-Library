// ---------------------------------------------------------------------------
//  Example of Acrome SMD Red BLINK with Arduino.
//
//  Example Code by : Berat Eren Dogan / Acrome Robotics
// ---------------------------------------------------------------------------
//  WARNING: To run this code make sure you have the following hardware components:
//
//  1. SMD (Smart Motor Driver - Red): Ensure that the SMD is properly connected and powered.
//
//  2. Arduino Gateway: Connect the Arduino Gateway to your Arduino board. Note that the Gateway should NOT be attached while uploading the code.
//
//  IMPORTANT: Do not have the Gateway attached to your Arduino when uploading the code. Only attach the Gateway after the code upload is complete.
//  This is crucial for proper initialization. Once the code is uploaded, attach the Gateway and establish the connection with the SMD as required.
//
//  If you encounter any issues, refer to the documentation of your hardware components for troubleshooting steps.
// ---------------------------------------------------------------------------

#include <Acrome-SMD.h>

#define BAUDRATE    115200
#define ID          0

Red red1(ID, Serial, BAUDRATE);

void setup() {
  red1.begin(); 
}

void loop() {
  // It continuously changes the position at regular time intervals.
  red1.torqueEnable(1);
  delay(1000);
  red1.torqueEnable(0);
  delay(1000);
}