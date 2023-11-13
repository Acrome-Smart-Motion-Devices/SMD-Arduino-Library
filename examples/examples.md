<!--This is the initializer for Master class which controls the serial bus.`portname` argument is the serial/COM port of the host computer which is connected to the Acrome Smart Motor Drivers via Mastercard. User may change this value to something between 3.053 KBits/s and 12.5 MBits/s. However, it is up to the user to select a value which is supported by the user's host computer.-->

### Example of Typical Begin
```cpp
    #include <SMDRed.h>
    uint8_t ID = 0;      //ID of used SMD
    int buttonID = 2;       //ID of used SMD button sensor module
    float PWM = 70.0;       //Duty Cycle of DC motor.
    int baudrate = 115200;  //baudrate of communication

    Red myRed(ID,Serial);

    void setup()
    {
    myRed.begin(baudrate);              //Initialize communication with the SMD at the specified baud rate
    myRed.setOperationMode(PWMControl); //set the SMD operation mode
    }

    void loop(){
    myRed.setpoint(PWMControl, PWM);         
    if(myRed.getButton(buttonID)){
      myRed.torqueEnable(1);
    }

    else{
      myRed.torqueEnable(0);
    }
    }
```

### Example of Setting CPR and RPM Values
```cpp
#include <SMDRed.h>
uint8_t ID = 0;
float cpr = 64.0;
float rpm = 10000.0;

  ### Example of typical begin
  ```cpp
  #include <SMDRed.h>
  uint8_t ID = 0;

  Red myRed(ID,Serial);

  void setup()
  {
  myRed.begin(115200);
  }

  void loop(){}
  ```

  ### Example of start and tune the motor.
  ```cpp
  #include <SMDRed.h>
  uint8_t ID = 0;
  float cpr = 64.0;
  float rpm = 10000.0;
  
  Red myRed(ID,Serial);
  
  void setup()
  {
  myRed.begin(115200);
  myRed.setMotorCPR(cpr);
  myRed.setMotorRPM(rpm);
  
  }
  void loop(){}
  ```

  ### Example of Position Control
  ```cpp
  #include <SMDRed.h>
  uint8_t ID = 0;

  Red myRed(ID, Serial);

  void setup()
  {
      myRed.begin(115200);
      delay(5);
      myRed.setPositionLimits(0, 10000); // Sets position limits from 0 to 10,000 encoder ticks
  }
  void loop(){}
  ```
  


  ### Example of Velocity Control
  ```cpp
  #include <SMDRed.h>
  uint8_t ID = 0;
  
  Red myRed(ID, Serial);
  
  void setup()
  {
      myRed.begin(115200);
      delay(5);
      myRed.setVelocityLimit(1000); // Sets velocity limit to 1000rpm
  }
  void loop(){
    

  }
  ```


  ### Example of Torque Control
  
  ```cpp
  #include <SMDRed.h>
  uint8_t ID = 0;
  
  Red myRed(ID, Serial);
  
  void setup()
  {
      myRed.begin(115200);
      delay(5);
      myRed.setTorqueLimit(200); // Sets torque limit to 200mA
  }
  void loop(){}
  ```
