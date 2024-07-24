// ---------------------------------------------------------------------------
//  Example of Acrome SMD Red Velocity Control with Arduino and Distance Module.
//
//  This code setting tones on the buzzer module.
//
//  Example Code by: Zubeyir Bakacak / Acrome Robotics
// ---------------------------------------------------------------------------
//  WARNING: To run this code, ensure that you have the following hardware components:
//
//  1. SMD (Smart Motor Driver - Red): Ensure that the SMD is properly connected and powered.
//
//  2. Arduino Gateway: Connect the Arduino Gateway to your Arduino board. Note that the Gateway should NOT be attached while uploading the code.
//
//  3. Buzzer Module: Connect the buzzer module to the SMD.
//
//  IMPORTANT: Do not have the Gateway attached to your Arduino when uploading the code. Only attach the Gateway after the code upload is complete. This is crucial for proper initialization. 
//  Once the code is uploaded, attach the Gateway and establish the connection with the SMD as required.
//
//  If you encounter any issues, refer to the documentation of your hardware components for troubleshooting steps.
// ---------------------------------------------------------------------------

#include <Acrome-SMD.h>

#define BAUDRATE    115200
#define ID          0
#define BuzzerID    1

Red red1(ID, Serial3, BAUDRATE);

void setup()
{
    red1.begin();
    red1.scanModules();

    delay(100);
}

void loop()
{
    red1.setBuzzer(BuzzerID,440);
    delay(500);

    red1.setBuzzer(BuzzerID,500);
    delay(500);

    red1.setBuzzer(BuzzerID,1000);
    delay(500);

    red1.setBuzzer(BuzzerID,2000);
    delay(500);

    red1.setBuzzer(BuzzerID,3000);
    delay(500);
}