// ---------------------------------------------------------------------------
//  Example of Acrome SMD Red PWM Control with Button Module and Arduino.
//
//  This code allows the DC brushed motor attached to the 
//  SMD-Red (Smart Motor Driver - Red) to reach %90 duty cycles in two directions
//  at regular time intervals, continuously repeating the specified sequence.
//
//  It also integrates a Button Module for controlling the motor movement.
//
//  Example Code by: Berat Eren Dogan / Acrome Robotics
// ---------------------------------------------------------------------------
//  WARNING: To run this code, make sure you have the following hardware components:
//
//  1. SMD (Smart Motor Driver - Red): Ensure that the SMD is properly connected and powered.
//
//  2. Arduino Gateway: Connect the Arduino Gateway to your Arduino board. Note that the Gateway should NOT be attached while uploading the code.
//
//  3. DC Motor: Connect the DC Motor to the SMD. Check the motor specifications for proper wiring.
//
//  4. Button Module: Connect the Button Module to the SMD. Update the 'buttonModuleID' variable in the code according to the module's ID selection.
//
//  IMPORTANT: Do not have the Gateway attached to your Arduino when uploading the code. Only attach the Gateway after the code upload is complete. This is crucial for proper initialization.
//  Once the code is uploaded, attach the Gateway and establish the connection with the SMD as required.
//
//  If you encounter any issues, refer to the documentation of your hardware components for troubleshooting steps.
// ---------------------------------------------------------------------------

#include <Acrome-SMD.h>

#define BAUDRATE    115200
#define ID          0

int buttonModuleID = 1;

Red red1(ID, Serial, BAUDRATE);
int dutycylce = 0;

void setup() {
  red1.begin(); 
  red1.setOperationMode(PWMControl); // setting operation mode to position
  red1.torqueEnable(1);

  red1.scanModules();
  delay(50);
}

void loop() {
  // It continuously changes the pwm setpoint(duty cycle percent) at regular time intervals.
  if(red1.getButton(buttonModuleID)){
    dutycylce += 3;
    if(dutycylce > 90){
      dutycylce = 0;
    }
  }
  red1.setpoint(PWMControl,dutycylce);
}