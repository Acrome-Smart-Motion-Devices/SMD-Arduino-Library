// ---------------------------------------------------------------------------
//  Example of Acrome SMD Red Position Control with Arduino.
//
//  This code allows the DC brushed motor attached to the 
//  SMD-Red(Smart Motor Driver - Red) to reach %90 duty cylces percents in two directions
//  at regular time intervals, continuously repeating the specified sequence.
//
//  Example Code by : Berat Eren Dogan / Acrome Robotics
// ---------------------------------------------------------------------------
//  WARNING: To run this code make sure you have the following hardware components:
//
//  1. SMD (Smart Motor Driver - Red): Ensure that the SMD is properly connected and powered.
//
//  2. Arduino Gateway: Connect the Arduino Gateway to your Arduino board. Note that the Gateway should NOT be attached while uploading the code.
//
//  3. DC Motor: Connect the DC Motor to the SMD. Check the motor specifications for proper wiring.
//
//  NOTE: Please set the correct values for CPR (Counts Per Revolution) and RPM (Revolutions Per Minute) based on your specific motor. Update the 'CPR' and 'RPM' variables in the code accordingly.
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
  red1.setOperationMode(PWMControl); // setting operation mode to position
  red1.torqueEnable(1);
}

void loop() {
  // It continuously changes the pwm setpoint(duty cycle percent) at regular time intervals.
  for(int i = 0; i <= 9 ; i++){
    red1.setpoint(PWMControl, i*10);
    delay(100);
  }
  for(int i = 9; i >= -9 ; i--){
    red1.setpoint(PWMControl, i*10);
    delay(100);
  }
  for(int i = -9; i <= 0 ; i++){
    red1.setpoint(PWMControl, i*10);
    delay(100);
  }
}