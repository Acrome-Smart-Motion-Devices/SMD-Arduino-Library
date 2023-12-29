// ---------------------------------------------------------------------------
//  Example of Acrome SMD Red Getting Module Data with Arduino.
//
//  This code demonstrates how to retrieve data from various modules connected to the 
//  SMD-Red (Smart Motor Driver - Red) using the Acrome-SMD library.
//
//  Example Code by: Berat Eren Dogan / Acrome Robotics
// ---------------------------------------------------------------------------
//  WARNING: To run this code, ensure that you have the following hardware components:
//
//  1. SMD (Smart Motor Driver - Red): Ensure that the SMD is properly connected and powered.
//
//  2. Arduino Gateway: Connect the Arduino Gateway to your Arduino board.
//     Note that the Gateway should NOT be attached while uploading the code.
//
//  3. Various Modules: Connect different modules (Button, Light, Joystick, Distance, QTR, Potentiometer, IMU) to the SMD.
//     Update the respective 'ID' variables for each module in the code accordingly.
//     For simplicity, 'ModuleIDs' is set to 0 for all modules; please adjust the IDs as needed.
//
//  IMPORTANT: Do not have the Gateway attached to your Arduino when uploading the code. 
//  Only attach the Gateway after the code upload is complete. This is crucial for proper initialization.
//  Once the code is uploaded, attach the Gateway and establish the connection with the SMD as required.
//
//  If you encounter any issues, refer to the documentation of your hardware components for troubleshooting steps.
// ---------------------------------------------------------------------------

#include <Acrome-SMD.h>

#define BAUDRATE    115200
#define ID          0 

int ModuleIDs = 0;

Red red1(ID, Serial, BAUDRATE);


float notes[] = {659.25, 659.25, 659.25,
         659.25, 659.25, 659.25,
         659.25, 783.99, 523.25, 587.33, 659.25, 
         698.46, 698.46, 698.46, 698.46, 698.46, 659.25, 659.25, 659.25, 659.25, 659.25, 587.33, 587.33, 659.25, 587.33, 783.99};
float durations[] = {0.5, 0.5, 1,
             0.5, 0.5, 1,
             0.5, 0.5, 1, 0.4, 0.5,
             0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1};

void setup() {
  red1.begin();
  red1.scanModules();
  red1.printAvailableSensors(Serial);
}

void loop() {
    
    for(int i = 0; i<255; i++){
        red1.setRGB(1, i, 100, 255-i);
        delay(300);
    }
    
    /*
    while(1){
        for(int i= 0; i <= 20; i++){
            red1.setBuzzer(1,(int)notes[i]);
            delay((int)(durations[i]*250));
            red1.setBuzzer(1, 0);
            delay(0.25*1000);
        }
    }
    */
  // You can use similar statements to print values for other modules.
  
}