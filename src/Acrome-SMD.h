#ifndef ACROME_SMD_H
#define ACROME_SMD_H

#include "Arduino.h"
#include "stdint.h"
#include "string.h"

#define ID_RNG_BUTTON_L			(1)
#define ID_RNG_BUTTON_H			(5)

#define ID_RNG_LIGHT_L			(6)
#define ID_RNG_LIGHT_H			(10)

#define ID_RNG_BUZZER_L			(11)
#define ID_RNG_BUZZER_H			(15)

#define ID_RNG_JOYSTICK_L		(16)
#define ID_RNG_JOYSTICK_H		(20)

#define ID_RNG_DISTANCE_L		(21)
#define ID_RNG_DISTANCE_H		(25)

#define ID_RNG_QTR_L			  (26)
#define ID_RNG_QTR_H			  (30)

#define ID_RNG_SERVO_L			(31)
#define ID_RNG_SERVO_H			(35)

#define ID_RNG_POT_L			  (36)
#define ID_RNG_POT_H			  (40)

#define ID_RNG_RGB_L			  (41)
#define ID_RNG_RGB_H			  (45)

#define ID_RNG_IMU_L			  (46)
#define ID_RNG_IMU_H			  (50)

typedef enum {
  PWMControl = 0,
  PositionControl = 1,
  VelocityControl = 2,
  TorqueControl = 3,
} tOperationMode;

typedef enum Index {
  iHeader,
  iDeviceID,
  iDeviceFamily,
  iPackageSize,
  iCommand,
  iStatus,
  iHardwareVersion,
  iSoftwareVersion,
  iBaudrate,
  iWritableStart = iBaudrate,
  iOperationMode,
  iTorqueEnable,
  iMotorCPR,
  iMotorRPM,
  iUserIndicator,
  iMinimumPositionLimit,
  iMaximumPositionLimit,
  iTorqueLimit,
  iVelocityLimit,
  iPositionFF,
  iVelocityFF,
  iTorqueFF,
  iPositionDeadband,
	iVelocityDeadband,
	iTorqueDeadband,
	iPositionOutputLimit,
	iVelocityOutputLimit,
	iTorqueOutputLimit,
  iPositionScalerGain,
  iPositionPGain,
  iPositionIGain,
  iPositionDGain,
  iVelocityScalerGain,
  iVelocityPGain,
  iVelocityIGain,
  iVelocityDGain,
  iTorqueScalerGain,
  iTorquePGain,
  iTorqueIGain,
  iTorqueDGain,
  iSetPosition,
  iPositionControlMode,
  iSCurvesetpoint,
  iSCurveAccel,
  iSCurveMaxVelocity,
  iSCurveTime,
  iSetVelocity,
  iSetVelocityAcceleration,
  iSetTorque,
  iSetDutyCycle,
  iSetScanModuleMode,
  iSetManualBuzzer,
  iSetManualServo,
  iSetManualRGB,
  iSetManualButton,
  iSetManualLight,
  iSetManualJoystick,
  iSetManualDistance,
  iSetManualQTR,
  iSetManualPot,
  iSetManualIMU,
  iBuzzer_1,
	iBuzzer_2,
	iBuzzer_3,
	iBuzzer_4,
	iBuzzer_5,
	iServo_1,
	iServo_2,
	iServo_3,
	iServo_4,
	iServo_5,
	iRGB_1,
	iRGB_2,
	iRGB_3,
	iRGB_4,
	iRGB_5,
  iReadOnlyStart = iRGB_5,
  iPresentPosition,
	iPresentVelocity,
	iMotorCurrent,
	iAnalogPort,
	iButton_1,
	iButton_2,
	iButton_3,
	iButton_4,
	iButton_5,
	iLight_1,
	iLight_2,
	iLight_3,
	iLight_4,
	iLight_5,
	iJoystick_1,
	iJoystick_2,
	iJoystick_3,
	iJoystick_4,
	iJoystick_5,
	iDistance_1,
	iDistance_2,
	iDistance_3,
	iDistance_4,
	iDistance_5,
	iQTR_1,
	iQTR_2,
	iQTR_3,
	iQTR_4,
	iQTR_5,
	iPot_1,
	iPot_2,
	iPot_3,
	iPot_4,
	iPot_5,
	iIMU_1,
	iIMU_2,
	iIMU_3,
	iIMU_4,
	iIMU_5,
  ConnectedBitfield,
	iCRCValue,
} tProtocolIndex;

typedef enum{
  BUTTON = 0,
  LIGHT,
  BUZZER,
  JOYSTICK,
  DISTANCE,
  QTR,
  SERVO,
  POTANTIOMETER,
  RGB,
  IMU
}tSensors;


// for rgb sensor.

typedef enum{
  NOCOLOR = 0,
  RED,
  GREEN,
  BLUE,
} tColors;

typedef struct{
  uint8_t LeftValue;
  uint8_t MiddleValue;
  uint8_t RightValue;
}QTRValues;


/*
typedef enum{
  NO_COLOR = 0,
  RED,
  GREEN,
  BLUE,
  WHITE,
  YELLOW,
  CYAN,
  MAGENTA,
  ORANGE,
  PURPLE,
  PINK,
  AMBER,
  TEAL,
  INDIGO
} tColors;
*/
// one smd tests    = $
// multi smd tests  = &


class Red {
  public:
    Red(uint8_t ID, HardwareSerial &port, uint32_t baudrate);
    void begin();                                                                                   //$?
    void end();
    bool Ping();                                                                                   
    void FactoryReset();                                                                            //$
    void Reboot();                                                                                  //$
    void EEPROMWrite();                                                                             //$

    uint32_t getSoftwareVersion();                                                                  //$
    uint32_t getHardwareVersion();                                                                  //$

    //MOTOR
    void setControlParameters(tOperationMode mode, float P, float I, float D, float FF, float DB);  //$
    void setpoint(tOperationMode mode, float setpoint);                                             //$
    void setPositionLimits(uint32_t min, uint32_t max);                                             //$
    void setVelocityLimit(uint16_t limit);                                                          //$
    void setTorqueLimit(uint16_t limit);                                                            //$
    void tune();                                                                                    //$
    void setMotorCPR(float cpr);                                                                    //$
    void setMotorRPM(float rpm);                                                                    //$
    void torqueEnable(uint8_t en);                                                                  //$
    void setOperationMode(tOperationMode mode);                                                     //$
    void setBaudrate(uint32_t baud);
    void setDeviceID(uint8_t ID);                                                                   //$
    void setTimeout(uint16_t timeout);                                                              //$
    
    void goTo(float setPoint, float SCurveTimeSet, float SCurveMaxVelocity, float SCurveAccel);
    void Red::setVelocityAccel(uint16_t accel);

    float getPosition();                                                                            //$
    float getVelocity();                                                                            //$
    float getTorque();                                                                              //$
    uint16_t getAnalogPort();                                                                       //+

    // MODULES
    uint8_t* scanModules();                                                                         //$
    void printAvailableSensors(HardwareSerial &port);                                               //$

    // Set
    void setBuzzer(int buzzerID, uint32_t frequency);                 //$
    void setServo(int servoID, uint8_t ctrl);                         //$
    void setRGB(int rgbID, uint8_t red, uint8_t green, uint8_t blue); //$

    // Get
    uint8_t   getButton(int buttonID);      //$
    uint16_t  getLight(int lightID);        //$

    int32_t   getJoystickX(int joystickID);    //$
    int32_t   getJoystickY(int joystickID);    //$
    uint8_t   getJoystickButton(int joystickID);  //$
                                           
    uint16_t  getDistance(int distanceID);  //$
    QTRValues getQTR(int qtrID);            //$?
    uint8_t   getPotentiometer(int potentiometerID); //$

    float     getRollAngle(int iIMU_ID);  //$
    float     getPitchAngle(int iIMU_ID); //$

    

  private:
    uint8_t _devId;
    HardwareSerial* _port;
    uint32_t _baudrate;
    uint16_t _timeout;
    void _write2serial(uint8_t debug = 0);
    bool _readFromSerial(uint8_t expected_bytes, uint8_t debug = 0);
    unsigned int _delayMicros;
    uint8_t _sensors[8];
};


#endif