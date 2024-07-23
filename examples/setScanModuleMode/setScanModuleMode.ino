// ---------------------------------------------------------------------------
//  Example of Acrome SMD Red Scan Modules Manual with Arduino.
//
//  This code allows manual sensor definitions 
//  You can connect too many sensors on SMD-Red(Smart Motor Driver - Red) 
//  
//
//  Example Code by : Zubeyir Bakacak / Acrome Robotics
// ---------------------------------------------------------------------------
//  WARNING: To run this code make sure you have the following hardware components:
//
//  1. SMD (Smart Motor Driver - Red): Ensure that the SMD is properly connected and powered.
//
//  2. Arduino Gateway: Connect the Arduino Gateway to your Arduino board. Note that the Gateway should NOT be attached while uploading the code.
//
//  3. Modules: Connect the correct module to the SMD. Check the sensor (for ex. RGB LED Module) for proper wiring.
//
//  NOTE: Check your module and define it accordingly.
//  
//  IMPORTANT: Do not have the Gateway attached to your Arduino when uploading the code. Only attach the Gateway after the code upload is complete.
//  This is crucial for proper initialization. Once the code is uploaded, attach the Gateway and establish the connection with the SMD as required.
//
//  If you encounter any issues, refer to the documentation of your hardware components for troubleshooting steps.
// ---------------------------------------------------------------------------

#include <Acrome-SMD.h>

#define BAUDRATE    115200
#define ID          0           //ID of the SMD-RED
#define SensorID    1           //Sensor ID's (Optional)

uint8_t module_list[1] =         //List of the connected modules
{
    iRGB_1              //RGB LED Module
//    ,iLight_1           //Ambient Light Sensor Module
//    ,iDistance_1        //Ultrasonic Distance Sensor Module
//    ,iServo_1           //Servo Motor Module
//    ,iIMU_1             //IMU Sensor Module
//    ,iBuzzer_1          //Buzzer Module
};


Red red1(ID, Serial, BAUDRATE);

void setup()
{
    red1.begin();

    red1.setConnectedModules(module_list, 1);       // Point the connected sensors (Connected Module List, Number of the Connected Modules)
}

void loop()
{
  //redx.setRGB(ModuleID, Red Value, Green Value, Blue Value);
  
    red1.setRGB(SensorID, 255,0,0);         //RED
    delay(250);

    red1.setRGB(SensorID, 0,255,0);         //GREEN
    delay(250);

    red1.setRGB(SensorID, 0,0,255);         //BLUE
    delay(250);

}