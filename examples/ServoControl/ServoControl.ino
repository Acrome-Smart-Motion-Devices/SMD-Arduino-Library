// ---------------------------------------------------------------------------
//  Example of Acrome SMD Red Position Control (Servo) with Arduino.
//
//  This code allows the Servo motor attached to the 
//  SMD-Red(Smart Motor Driver - Red) to reach predefined positions 
//  at regular time intervals, continuously repeating the specified sequence.
//
//  Example Code by : Zubeyir Bakacak / Acrome Robotics
// ---------------------------------------------------------------------------
//  WARNING: To run this code make sure you have the following hardware components:
//
//  1. SMD (Smart Motor Driver - Red): Ensure that the SMD is properly connected and powered.
//
//  2. Arduino Gateway: Connect the Arduino Gateway to your Arduino board. Note that the Gateway should NOT be attached while uploading the code.
//
//  3. Servo Motor: Connect the Servo Motor to the SMD. Check the motor specifications for proper wiring.
//
//  
//  IMPORTANT: Do not have the Gateway attached to your Arduino when uploading the code. Only attach the Gateway after the code upload is complete.
//  This is crucial for proper initialization. Once the code is uploaded, attach the Gateway and establish the connection with the SMD as required.
//
//  If you encounter any issues, refer to the documentation of your hardware components for troubleshooting steps.
// ---------------------------------------------------------------------------

#include <Acrome-SMD.h>

#define BAUDRATE    115200
#define ID          0
#define ServoID     1

Red red1(ID, Serial, BAUDRATE);

void setup()
{
    red1.begin();
    red1.scanModules();

    delay(100);
}

void loop()
{
    red1.setServo(ServoID,0);
    delay(850);

    red1.setServo(ServoID,45);
    delay(250);

    red1.setServo(ServoID,90);
    delay(250);

    red1.setServo(ServoID,135);
    delay(250);

    red1.setServo(ServoID,180);
    delay(500);
}