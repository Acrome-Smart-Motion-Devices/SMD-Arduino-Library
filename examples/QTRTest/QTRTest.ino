// ---------------------------------------------------------------------------
//  Example of Acrome SMD Red QTR Module reading with arduino.
//
//  This code allows to analog read to qtr module in real-time application.
//
//  Example Code by: Zubeyir Bakacak / Acrome Robotics
// ---------------------------------------------------------------------------
//  WARNING: To run this code, make sure you have the following hardware components:
//
//  1. SMD (Smart Motor Driver - Red): Ensure that the SMD is properly connected and powered.
//
//  2. Arduino Gateway: Connect the Arduino Gateway to your Arduino board. Note that the Gateway should NOT be attached while uploading the code.
//
//  3. QTR Module: Connect the QTR Module to the SMD. Select the appropriate ID pin on the module, and update the 'QTRID' variable in the code accordingly.
//
//  
//  IMPORTANT: Do not have the Gateway attached to your Arduino when uploading the code. Only attach the Gateway after the code upload is complete. This is crucial for proper initialization. 
//  Once the code is uploaded, attach the Gateway and establish the connection with the SMD as required.
//
//  If you encounter any issues, refer to the documentation of your hardware components for troubleshooting steps.
// ---------------------------------------------------------------------------

#include <Acrome-SMD.h>

#define BAUDRATE    115200
#define ID          0
#define QTRID       1


Red red1(ID, Serial3, BAUDRATE);          


void setup()
{
    red1.begin();               //Start the SMD-RED
    Serial.begin(BAUDRATE);     //Start the Serial communication
    red1.scanModules();         //Scan all modules
    delay(200);
}

void loop()
{
    QTRValues qtrValues = red1.getQTR(QTRID);        //Create QTR Object
    uint8_t QTRArray[]={qtrValues.LeftValue, qtrValues.MiddleValue, qtrValues.RightValue}; //Write QTR Values into QTR Array

    Serial.print(QTRArray[0]);          //
    Serial.print(" , ");                //
    Serial.print(QTRArray[1]);          // Print QTR Values
    Serial.print(" , ");                //
    Serial.println(QTRArray[2]);        //

    delay(50);
}