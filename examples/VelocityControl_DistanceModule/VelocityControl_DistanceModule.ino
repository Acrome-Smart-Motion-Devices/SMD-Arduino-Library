// ---------------------------------------------------------------------------
//  Example of Acrome SMD Red Velocity Control with Arduino and Distance Module.
//
//  This code enables the DC brushed motor attached to the 
//  SMD-Red (Smart Motor Driver - Red) to achieve varying velocities 
//  based on the distance measured by the attached Distance Module.
//
//  Example Code by: Berat Eren Dogan / Acrome Robotics
// ---------------------------------------------------------------------------
//  WARNING: To run this code, ensure that you have the following hardware components:
//
//  1. SMD (Smart Motor Driver - Red): Ensure that the SMD is properly connected and powered.
//
//  2. Arduino Gateway: Connect the Arduino Gateway to your Arduino board. Note that the Gateway should NOT be attached while uploading the code.
//
//  3. DC Motor: Connect the DC Motor to the SMD. Check the motor specifications for proper wiring.
//
//  4. Distance Module: Connect the Distance Module to the SMD. Update the 'DistanceModuleID' variable in the code according to the module's ID selection.
//
//  NOTE: Please set the correct values for CPR (Counts Per Revolution) and RPM (Revolutions Per Minute) based on your specific motor. Update the 'CPR' and 'RPM' variables in the code accordingly.
//
//  IMPORTANT: Do not have the Gateway attached to your Arduino when uploading the code. Only attach the Gateway after the code upload is complete. This is crucial for proper initialization. 
//  Once the code is uploaded, attach the Gateway and establish the connection with the SMD as required.
//
//  If you encounter any issues, refer to the documentation of your hardware components for troubleshooting steps.
// ---------------------------------------------------------------------------

#include <Acrome-SMD.h>

#define BAUDRATE    115200
#define ID          0

float RPM = 100;
float CPR = 4741;
int DistanceModuleID = 01

Red red1(ID, Serial, BAUDRATE);
int distance = 0;

void setup() {
  red1.begin(); 
 
  red1.setMotorRPM(RPM); 
  red1.setMotorCPR(CPR);

  red1.setOperationMode(VelocityControl); // Setting operation mode to velocity control
  red1.setControlParameters(VelocityControl, 10, 1, 0, 0, 5); // Setting PID and other control parameters of velocity control mode.
  red1.torqueEnable(1);

  red1.scanModules();
  delay(50);
}

void loop() {
  // It continuously changes the motor velocity based on the distance measured by the Distance Module.
  if(red1.getDistance(DistanceModuleID) > 150){
    red1.setpoint(VelocityControl, RPM*0.9);
  }
  else if(red1.getDistance(DistanceModuleID) > 80){
    red1.setpoint(VelocityControl, RPM*0.75);
  }
  else if(red1.getDistance(DistanceModuleID) > 40){
    red1.setpoint(VelocityControl, RPM*0.50);
  }
  else if(red1.getDistance(DistanceModuleID) > 10){
    red1.setpoint(VelocityControl, RPM*0.25);
  }
  else{
    red1.setpoint(VelocityControl, 0);
  }
  
  delay(50);
}


