# Overview
Introducing the **Acrome Smart Motor Driver (SMD)** Arduino Library, a comprehensive and versatile tool tailored to provide seamless control and communication with the SMD. The SMD Arduino Library simplifies the control of SMD with Arduino, allowing you to focus on the core functions of your projects. Designed with precision and care, this custom driver board is not only a motor control powerhouse but also boasts the capability to seamlessly interface with the SMD Sensor modules making it the ultimate solution for your Arduino projects.

-  **Effortless Motor Control with Arduino:** The hallmark feature of the SMD Arduino Library is its ability to simplify motor control. Designed for Arduino, this library streamlines the process of controlling motors. With Arduino's intuitive programming environment and the SMD library, you can effortlessly command motors, making it an ideal choice for both beginners and experienced developers. Whether you need basic speed control or position control, you can effortlessly implement various motor control strategies using Arduino and this library.
- **Seamless Sensor Integration:** In addition to motor control, this library seamlessly interfaces with SMD Sensor modules. This integration empowers you to gather data from modules effortlessly, enhancing your project's functionality without the need for complex coding or external libraries.


# Installation

To get started with the Smart Motor Driver Arduino Library, follow these steps:

1. Download the Smart Motor Driver Library ZIP file from the following link: [Acrome-SMD Arduino Library ZIP](buraya-indirme-linkini-ekleyin).

2. Open your Arduino IDE.

3. In the Arduino IDE, go to `Sketch` -> `Include Library` -> `"Add (SMD)".ZIP Library`.

4. A file dialog will appear. Browse to the location where you downloaded the SMD Library ZIP file and select it. Then, click the `Open` button.

5. The Arduino IDE will automatically install the library from the ZIP file. You'll see a message in the status bar at the bottom of the IDE indicating that the library has been successfully added.

# Usage
 When using the SMDRed library for motor control, follow these essential steps:

   1. Include the SMDRed library at the beginning of your Arduino sketch.
   
   2. Define the following variables:
      - `ID`: The ID of the connected SMD (Smart Motor Driver). This ID identifies the specific motor driver you are using. (Note: The ID of an SMD that has not been used before and whose ID has not been changed is 0.)
      - `Serial`: The serial communication interface you are using.
      - `baudrate`: The baud rate for serial communication. (Baudrate should be 115200 if not set before)
   
   3. Create an instance of the `Red` class by initializing it with the `ID` and the `Serial`. This instance represents your motor driver and allows you to control it.

   4. In the `setup()` function:
       - Initialize communication with the SMD using `myRed.begin()`. Make sure to use the appropriate baud rate for your setup.
      - Set the SMD's operation mode based on your application requirements. For example, use `myRed.setOperationMode(PWMControl)` for PWM motor control.

   5. In the `loop()` function or your program logic:
      - Implement motor control based on your needs. For PWM motor control, set the desired duty cycle using `PWM` and adjust it as necessary to control the motor's speed.
      - Utilize functions such as `myRed.torqueEnable()` to enable or disable motor control as needed.

   These steps provide a fundamental guideline for using the SMDRed library to control motors with an SMD. Be sure to consult the library's documentation for detailed information on available functions and features specific to your motor control application.


### Example of typical PWM control with button module
```cpp
    #include <Acrome-SMD.h>
    uint8_t ID = 0x00;      // ID of used SMD
    int buttonID = 2;       // ID of used SMD button sensor module
    float PWM = 70.0;       // Duty Cycle of DC motor.
    int baudrate = 115200;  // Baudrate of communication

    Red myRed(ID,Serial, baudrate);

    void setup()
    {
    myRed.begin();              // Initialize communication with the SMD at the specified baud rate
    myRed.setOperationMode(PWMControl); // Set the SMD operation mode
    }

    void loop(){
    myRed.setpoint(PWMControl, PWM);    // Set the PWM value for motor control
    if(myRed.getButton(buttonID)){
      myRed.torqueEnable(1);            // Enable torque (motor control) when the button is pressed
    }

    else{
      myRed.torqueEnable(0);            // Disable torque (stop the motor) when the button is not pressed
    }
    }
```

For a detailed guide on library functions and usage, please review the full documentation. [Usage Examples](#usage-examples) are at the end of the documentation.

## SMD Main and Motor Control Methods

  - ### `Red - begin()`
      This is the initializer for the serial bus. It starts the communication with Smart Motor Drivers(SMD).
      ### Syntax
      #### `myRed.begin();`
      - `myRed`: a variable of type `Red`

      **`Return:`** *None*

  - ### `Red - end()`
    Disables serial communication, allowing the RX and TX pins to be used for general input and output. 
      ### Syntax
      #### `myRed.end();`
      - `myRed`: a variable of type `Red`

      **`Return:`** *None*

  - ### `Red - Ping()`
    sends a ping to SMD board. Communicate with SMD board and returns `true` if the SMD available.
    ### Syntax
    #### `myRed.Ping():`
    - `myRed`: a variable of type `Red`

    **`Return:`** *`bool`* (`true` or `false`)
    ### Example
    ```cpp
      #include <Acrome-SMD.h>
      #define BAUDRATE  115200
      uint8_t ID = 0x00;
      Red myRed(ID,Serial, BAUDRATE);

      void setup()
      {
        myRed.begin();
        delay(5);
      }
      void loop()
      {
        if(myRed.Ping()){
          //do
        }
        
      }
      ```

  - ### `Red - setBaudrate()`
    This method updates the baudrate of the SMD's serial port. It is necessary to write to EEPROM and Reboot the SMD after this method called to connect the SMD with updated baudrate.
      ### Syntax
      #### `myRed.setBaudrate(baudrate);`
      - `myRed`: a variable of type `Red`
      - `baudrate` argument specifies the baudrate of the serial port to set

      **`Return:`** *None*
      ### Example
      ```cpp
      #include <Acrome-SMD.h>
      #define CURRENT_BAUDRATE  9600
      #define NEW_BAUDRATE      115200
      uint8_t ID = 0x00;
      Red myRed(ID,Serial, CURRENT_BAUDRATE);

      void setup()
      {
        myRed.begin();
        delay(5);
        myRed.setBaudrate(NEW_BAUDRATE);
        delay(5);
        myRed.EEPROMWrite();
        delay(5);
        myRed.Reboot();
      }
      void loop(){}
      ```

  - ### `Red - FactoryReset()`
    Resets all variables and memory on the SMD board, including the EEPROM. Restores the SMD to factory settings.
      ### Syntax
      #### `myRed.FactoryReset():`
      - `myRed`: a variable of type `Red`

      **`Return:`** *None*


  - ### `Red - Reboot()`
    The method is designed to reboot the SMD board with which the Arduino interacts. If there is a problem in communication between Arduino and SMD board, SMD cannot restart itself.

    The primary purpose of the `Reboot()` method is to initiate a system reboot on the target SMD board. This action is often required to apply configuration changes or resolve issues without manual intervention.

      ### Syntax
      #### `myRed.Reboot()`
    - `myRed`: a variable of type `Red`

    **`Return:`** *None*

  - ### `Red - EEPROMWrite()`
    The 'EEPROMWrite()' method is utilized to store specific device configuration parameters in the SMD's EEPROM. This ensures that certain recorded data will not be lost when the SMD is powered off and then powered on again. Some of this data is crucial for record-keeping purposes, including the SMD device ID, operating mode, characteristics of the motor connected to the SMD, limits, and PID constants.
    
    Here is the list of all data saved in the EEPROM using this method:

  - Device Identifier (ID)
  - Communication Baud Rate
  - Motor Encoder Counts per Revolution (CPR)
  - Motor Revolutions per Minute (RPM)
  - Motor Operation Mode
  - Minimum Motor Position Limit
  - Maximum Motor Position Limit
  - Motor Torque Limit
  - Motor Velocity Limit
  - Position Control Feedforward Gain
  - Velocity Control Feedforward Gain
  - Torque Control Feedforward Gain
  - Position Control Deadband
  - Velocity Control Deadband
  - Torque Control Deadband
  - Position Control Scaler Gain
  - Velocity Control Scaler Gain
  - Torque Control Scaler Gain
  - Position Control Proportional Gain (P-Gain)
  - Position Control Integral Gain (I-Gain)
  - Position Control Derivative Gain (D-Gain)
  - Velocity Control Proportional Gain (P-Gain)
  - Velocity Control Integral Gain (I-Gain)
  - Velocity Control Derivative Gain (D-Gain)
  - Torque Control Proportional Gain (P-Gain)
  - Torque Control Integral Gain (I-Gain)
  - Torque Control Derivative Gain (D-Gain)

    The EEPROMWrite() method essentially saves these parameter values to the EEPROM, allows the device to retain its configuration even after power cycles or resets.
      ### Syntax
      #### `myRed.EEPROMWrite()`
      - `myRed`: a variable of type `Red`

      **`Return:`** *None*

  - ### `Red - setMotorCPR()`
    Sets the CPR of motor. It is very important to read encoder if you use. Make sure you entered the parameter correctly. Otherwise, the calculations will not work properly.
      ### Syntax
      #### `myRed.setMotorCPR(cpr)`
      - `myRed` a variable of type `Red`
      - `cpr` a variable of type `float`

      **`Return:`** *None*
      

  - ### `Red - setMotorRPM()`
    Sets the RPM of motor. It is very important to read encoder and drive motor. Make sure you entered the parameter correctly. Otherwise, the calculations will not work properly.
      ### Syntax
      #### `myRed.setMotorRPM(rpm)`
      - `myRed` a variable of type `Red`
      - `rpm` a variable of type `float`

      **`Return:`** *None*
      ### Example
      ```cpp
      #include <Acrome-SMD.h>
      uint8_t ID = 0x00;
      float cpr = 64.0;
      float rpm = 10000.0;

      Red myRed(ID,Serial, 115200);

      void setup()
      {
        myRed.begin();
        myRed.setMotorCPR(cpr);
        myRed.setMotorRPM(rpm);
      }
      void loop(){}
      ```

  - ### `Red - setControlParameters()`
      Sets the control parameters of given control modes. This method does not set the control mode of SMD, but the parameters of given mode. This method can set these control parameters: PID control constants, Feedforward and Deadband.
      ### Syntax
      #### `myRed.setControlParameters(mode, P, I, D, FF, DB);`
      - `myRed`: a variable of type `Red`
      - `mode`: a variable of type `tOperationMode`
      - `P`:  Proportional constant of PID control (type `float`)
      - `I`:  Integral constant of PID control (type `float`)
      - `D`:  Derivative constant of PID control (type `float`)
      - `FF`: Feedforward of PID control (type `float`)
      - `Db`: Deadband of PID control (type `float`)

      **`Return:`** *None*

      ### Example
      ```cpp
      #include <Acrome-SMD.h>
      uint8_t ID = 0x00;

      Red myRed(ID,Serial, 115200);

      void setup()
      {
        myRed.begin();
        delay(5);
        myRed.setControlParameters(VelocityControl, 0.4, 1.2, 0.7);
      }
      void loop(){}
      ```
  
  - ### `Red - getControlParameters()`
      Gets the requested control parameter of the given control mode. This method can return these parameters of any chosen control mode: 
      - P gain, I gain, D gain, feedforward value, deadband value.
      ### Syntax
      #### `myRed.getControlParameters(port, mode, param)`
      - `myRed`: a variable type of `Red`
      - `port`: port adress that parameter will be returned (type `HardwareSerial`)
      - `mode`: selected operation mode (type `tOperationMode`)
      - `param`: requested parameter to be returned (type `float`)

      **`Return:`** `float`
      
      ### Example
      ```cpp
      #include <Acrome-SMD.h>
      uint8_t ID = 0x00;

      Red myRed(ID, Serial, 115200);

      void setup()
      {
        myRed.begin();

        Serial.println(myRed.getControlParameters(Serial, PositionControl, Pgain));
      }

      void loop() {}
      ```

  - ### `Red - setOperationMode()`
      Sets the operation mode of SMD by given mode. Available operation modes:
      - PWMControl, PositionControl, VelocityControl, TorqueControl.
      ### Syntax
      #### `myRed.setOperationMode(mode);`
      - `myRed`: a variable of type `Red`
      - `mode`: a variable of type `tOperationMode`

      **`Return:`** *None*

  - ### `Red - setpoint()`
    Sets the setpoint of the operation mode by given setpoint and mode. Depending on which mode's setpoint you enter, the unit of the setpoint you will enter changes. Setpoint units:

    - PWMControl: `setpoint` is duty cycle(0 to 100).
    - VelocityControl: `setpoint` is RPM.
    - PositionControl: `setpoint` is encoder tick count.
    - TorqueControl: `setpoint` is current(mA).
    
    ### Syntax
    #### `myRed.setpoint(mode, setpoint);`
    - `mode` parameter is a variable of type `OperationMode`
    - `setpoint` parameter is a variable of type `float`

    **`Return:`** *None*

    ### Example
    ```cpp
      #include <Acrome-SMD.h>
      uint8_t ID = 0x00;
      float PWMsetpoint = 60.0; // represents %60 duty cyle since operation mode is "PWM Control" 
      Red myRed(ID,Serial, 115200);

      void setup()
      {
        myRed.begin();
        delay(5);
        myRed.setOperationMode(PWMControl);
        myRed.setpoint(PWMsetpoint);
        myRed.torqueEnable(1);
      }
      void loop(){}
    ```

  - ### `Red - setPositionLimits()`
    Sets the position limits of the SMD. You should set the limits if you drive a motor as positional control.
    
    ### Syntax
    #### `myRed.setPositionLimits(min, max);`
    - `myRed`: a variable of type `Red`
    - `min`: Minimum position limit (type: `uint32_t`)
    - `max`: Maximum position limit represents encoder tick count (type: `uint32_t`)
    
    **`Return:`** *None*

  - ### `Red - setVelocityLimit()`
    Sets the velocity limits of the SMD. You should set the limits if you drive a motor as velocity control.
    
    ### Syntax
    #### `myRed.setVelocityLimit(limit);`
    - `myRed`: a variable of type `Red`
    - `limit`: Velocity limit (RPM) (type: `uint16_t`)
    
    **`Return:`** *None*

  - ### `Red - setTorqueLimit()`
    Sets the torque limit of the SMD. You should set the limits if you drive a motor as torque control.
    
    ### Syntax
    #### `myRed.setTorqueLimit(limit);`
    - `myRed`: a variable of type `Red`
    - `limit`: torque limit (mA) (type: `uint16_t`)
    
    **`Return:`** *None*

  - ### `Red - torqueEnable()`
    The method enables or disables the torque of a motor based on the provided parameter. This method is used to control whether the motor is allowed to rotate.

    - When the parameter is set to 1, the motor's torque is enabled, allowing it to rotate and perform mechanical work.
    - When the parameter is set to 0, the motor's torque is disabled, causing the motor to remain stationary or free-wheel.

    Note: If torque is not enabled, Smart Motor Drivers(SMD) will not drive motor no matter what. 
    
    ### Syntax
    #### `myRed.torqueEnable(en);`
    - `myRed`: a variable of type `Red`
    - `en`: torque enable (type: `uint8_t`)
    
    **`Return:`** *None*

  - ### `Red - tune()`
    The method activates the autotune task on a specific card and initiates the autotune process. SMD board will set the PID constants after this auto-tuning task.
    
    The duration of the autotune task may vary depending on the motor installed in the SMD card. After this method is used, the SMD does not perform the operations related to driving until the tune task is completed. Communication with the SMD can be continued, but the card does not drive the motor in any way other than the tune function.
    
    Note: when tune method is called, torque enable becomes "1" so motor can rotate.

    
    ### Syntax
    #### `myRed.tune();`
    - `myRed`: a variable of type `Red`
    
    **`Return:`** *None*

    
    ### Example
    ```cpp
    #include <Acrome-SMD.h>
    uint8_t ID = 0x00;

    Red myRed(ID, Serial, 115200);

    void setup()
    {
      myRed.begin();
      myRed.setMotorCPR(cpr);
      myRed.setMotorRPM(rpm);
      myRed.tune(); // starts the tuning task
    }
    void loop(){}
    ```

  - ### `Red - setDeviceID()`
    The method is a crucial function that allows you to modify the identification (ID) of the connected SMD board. This function is particularly useful when you need to customize the unique identifier associated with your hardware.
    
    Note: The ID of an SMD that has not been used before and whose ID has not been changed is 0.

    ### Syntax
    #### `myRed.setDeviceID(ID);`
    - `myRed`: a variable of type `Red`
    - `ID`: the one will that replace the previous ID (type: `uint8_t`)
    
    **`Return:`** *None*

  - ### `Red - setTimeout()`
    The method is a function designed to set the communication timeout value for the connected device or board. This function plays role in managing the time frame within which communication with the hardware should occur.

    ### Syntax
    #### `myRed.setTimeout(timeout);`
    - `myRed`: a variable of type `Red`
    - `timeout`: A value representing the desired timeout period in milliseconds. (type: `uint16_t`)
    
    **`Return:`** *None*

[//]: # (Getting Method----------------------------)

  - ### `Red - getPosition()`
    This method returns how much the motor has rotated. It returns a `float` value based on how many ticks the motor's encoder has counted. This value represents the measurement of the distance or rotation the motor has undergone.

    ### Syntax
    #### `myRed.getPosition();`
    - `myRed`: a variable of type `Red`

    **`Return:`** `float`


  - ### `Red - getCurrent()`
    This method returns the amount of current that the motor has drawn while rotating, measured in milliamperes (mA). It returns a `float` value representing the current consumption of the motor.

    ### Syntax
    #### `myRed.getCurrent();`
    - `myRed`: a variable of type `Red`

    **`Return:`** `float`

  - ### `Red - getAnalogPort()`
    This method reads and returns the connected module's analog signal from the analog port.

    ### Syntax
    #### `myRed.getAnalogPort();`
    - `myRed`: a variable of type `Red`

    **`Return:`** `uint16_t`





## SMD Sensor Module Methods

  - ### `Red - scanModules()`
    The scanModules() method is used to scan and save information about connected modules. It saves ID's and types of connected sensor modules.

    ### Syntax
    #### `myRed.scanModules();`
    - `myRed`: a variable of type `Red`

    **`Return:`** `uint8_t*`:
    This method returns a pointer to an 8-element `uint8_t` array where each bit represents the status of the scanned modules. Each bit indicates whether a module is present or not.

  - ### `Red - printAvailableModules()`
    This method prints the connected modules and their IDs to the Arduino's serial monitor, taking from the information recorded in the last scan.
    
    Ensure that you have executed the `scanModules()` method before calling `printAvailableModules()` to have up-to-date module information.

    ### Syntax
    #### `myRed.printAvailableModules(port);`
    - `myRed`: a variable of type `Red`
    - `port`: port to be scanned for modules (type `HardwareSerial`)

    **`Return:`** `None`

    ### Serial Monitor Prints Example
    - Button    : 1
    - Button    : 3
    - Joystick  : 5
    - RGB       : 2

    This print says 4 sensor modules connected to the SMD. It also shows types and IDs of these modules.

  - ### `Red - setBuzzer()`
    The setBuzzer() method is used to set a buzzer module to the given frequency.

    ### Syntax
    #### `myRed.setBuzzer(buzzerID, freq);`
    - `myRed`: a variable of type `Red`
    - `buzzerID`: a variable of type `int`
    - `freq`: a variable of type `uint32_t`. The frequency value desired to set.


  - ### `Red - setServo()`
    The setServo() method is used to control a servo motor connected to the Servo Sensor Module.

    ### Syntax
    #### `myRed.setServo(servoID, ctrl);`
    - `myRed`: a variable of type `Red`
    - `servoID`: a variable of type `int` 
    - `ctrl` : a variable of type `uint8_t`. It's PWM value for servo.

    **`Return:`** `None`

  - ### `Red - setRGB()`
    The setRGB() method is used to control an RGB (Red, Green, Blue) LED module by specifying the intensity of each three color.

    ### Syntax
    #### `myRed.setRGB(RGB_ID, red, green, blue);`
    - `myRed`: a variable of type `Red`
    - `RGB_ID`: a variable of type `int`
    - `red`: a variable of type `uint8_t`
    - `green`: a variable of type `uint8_t`
    - `blue`: a variable of type `uint8_t`

    **`Return:`** `None`

  - ### `Red - getButton()`
    This method gets the button value from button module.
    It returns 0 (zero) when not pressed and 1 (one) when pressed.

    ### Syntax
    #### `myRed.getButton(buttonID);`
    - `myRed`: a variable of type `Red`
    - `buttonID`: a variable of type `int`

    **`Return:`** `uint8_t`
  
  - ### `Red - getDistance()`
    This method is used to obtain distance data from ultrasonic distance sensor module.

    `return` the measured distance as an unsigned 16-bit integer (uint16_t) value in centimeters (cm).

    ### Syntax
    #### `myRed.getDistance(distanceID);`
    - `myRed`: a variable of type `Red`
    - `distanceID`: a variable of type `int`

    **`Return:`** `uint16_t`

  - ### `Red - getLight()`
    This method gets the ambient light module data with given ID.

    ### Syntax
    #### `myRed.getLight(lightID);`
    - `myRed`: a variable of type `Red`
    - `lightID`: a variable of type `int`

    **`Return:`** `uint16_t` unit of Lux


  - ### `Red - getQTR()`
    This method gets the QTR module data with given IDs.

    ### Syntax
    #### `myRed.getQTR(QTRID);`
    - `myRed`: a variable of type `Red`
    - `QTRID`: a variable of type `int`

    **`Return:`** `float`

  - ### `Red - getJoystickX()`
    It gets the X analog value from joystick modules with given module IDs.

    ### Syntax
    #### `myRed.getJoystickX(joystickID);`
    - `myRed`: a variable of type `Red`
    - `joystickID`: a variable of type `int`

    **`Return:`** `float`

    `return` 
  - ### `Red - getJoystickY()`
    It gets the Y analog value from joystick modules with given module IDs.

    ### Syntax
    #### `myRed.getJoystickY(joystickID);`
    - `myRed`: a variable of type `Red`
    - `joystickID`: a variable of type `int`

    **`Return:`** `float`

  - ### `Red - getJoystickButton()`
    It gets the button value from joystick modules with given module IDs.

    ### Syntax
    #### `myRed.getJoystickButton(joystickID);`
    - `myRed`: a variable of type `Red`
    - `joystickID`: a variable of type `int`

    **`Return:`** `uint8_t`

  - ### `Red - getPotentiometer()`
    This method gets the analog value from potentiometer modules with given IDs.

    ### Syntax
    #### `myRed.getPotentiometer(potentiometerID);`
    - `myRed`: a variable of type `Red`
    - `potentiometerID`: a variable of type `int`

    **`Return:`** `uint8_t`


  - ### `Red - getRollAngle()`
    This method gets the roll angle data from IMU modules with given IDs.

    ### Syntax
    #### `myRed.getRollAngle(iIMU_ID);`
    - `myRed`: a variable of type `Red`
    - `iIMU_ID`: a variable of type `int`

    **`Return:`** `float`

  - ### `Red - getPitchAngle()`
    This method gets the roll angle data from IMU modules with given IDs.

    ### Syntax
    #### `myRed.getPitchAngle(iIMU_ID);`
    - `myRed`: a variable of type `Red`
    - `iIMU_ID`: a variable of type `int`

    **`Return:`** `float`



# Usage Examples


  ### Example of typical begin
  ```cpp
  #include <Acrome-SMD.h>
  uint8_t ID = 0x00;

  Red myRed(ID,Serial, 115200);

  void setup()
  {
  myRed.begin();
  }

  void loop(){}
  ```

  ### Example of start and tune the motor.
  ```cpp
  #include <Acrome-SMD.h>
  uint8_t ID = 0x00;
  float cpr = 64.0;
  float rpm = 10000.0;
  
  Red myRed(ID,Serial, 115200);
  
  void setup()
  {
  myRed.begin();
  myRed.setMotorCPR(cpr);
  myRed.setMotorRPM(rpm);
  myRed.tune();
  }
  void loop(){}
  ```
  ### Example of PWM Control
  ```cpp
  #include <Acrome-SMD.h>
  uint8_t ID = 0x00; //ID of using SMD

  int dutycyle = 50; //it represents %50 percentage duty cycle
  
  Red myRed(ID, Serial, 115200);
  void setup()
  {
    myRed.begin();   

   
    myRed.setOperationMode(PWMControl); //selecting mode
    myRed.setpoint(PWMControl, dutycycle); //setting PWM to %50 duty cyle
    myRed.torqueEnable(1);  //torque enable.
  }
  void loop(){
  }
  
  ```

  ### Example of Position Control
   ```cpp
  #include <Acrome-SMD.h>
  uint8_t ID = 0x00; //ID of using SMD

  float CPR = 64;
  float RPM = 10000; 

  int position = 1000; //it represents 1000 rpm
  
  Red myRed(ID, Serial, 115200);
  
  void setup()
  {
    myRed.begin();

    myRed.setMotorCPR(CPR);
    myRed.setMotorRPM(RPM);    

    myRed.setControlParameters(PositionControl, 10.0, 0, 0.1, 0, 0);//use this method or just tune the SMD before drive at position control.
    myRed.setPositionLimits(10000, -10000); // Sets position limits to 10000 encoder ticks to -10000 encoder ticks
    myRed.setOperationMode(PositionControl); //selecting mode
    myRed.setpoint(PositionControl, positionsetpoint); //setting position by setting position setpoint
    myRed.torqueEnable(1);  //torque enable.
  }
  void loop(){
  }
  ```

  ### Example of Velocity Control
  ```cpp
  #include <Acrome-SMD.h>
  uint8_t ID = 0x00; //ID of using SMD

  float CPR = 64;
  float RPM = 10000; 

  int velocitySetpoint = 1000; //it represents 1000 rpm
  
  Red myRed(ID, Serial, 115200);
  
  void setup()
  {
    myRed.begin();

    myRed.setMotorCPR(CPR);
    myRed.setMotorRPM(RPM);

    myRed.setControlParameters(VelocityControl, 10.0, 0.1, 0, 0, 0);//use this method or just tune the SMD before drive at velocity control.
    myRed.setVelocityLimit(1000); // Sets velocity limit to 1000rpm
    myRed.setOperationMode(VelocityControl); //selecting mode
    myRed.setpoint(VelocityControl, velocitySetpoint); //setting velocity by setting velocity setpoint
    myRed.torqueEnable(1);  //torque enable.
  }
  void loop(){
  }
  ```


  ### Example of Torque Control
  
  ```cpp
  #include <Acrome-SMD.h>
  uint8_t ID = 0x00; //ID of using SMD

  float CPR = 64;
  float RPM = 10000; 

  int torqueSetpoint = 100; //it represents 100 milliamps(mA)
  
  Red myRed(ID, Serial, 115200);
  
  void setup()
  {
    myRed.begin();

    myRed.setMotorCPR(CPR);
    myRed.setMotorRPM(RPM);

    myRed.setControlParameters(TorqueControl, 10.0, 0.1, 0, 0, 0);//use this method before drive at Torque control.
    myRed.setTorqueLimit(200); // Sets Torque limit to 200mA
    myRed.setOperationMode(TorqueControl); //selecting mode
    myRed.setpoint(TorqueControl, torqueSetpoint); //setting torque by setting torque setpoint
    myRed.torqueEnable(1);  //torque enable.
  }
  void loop(){
  }
  ```
