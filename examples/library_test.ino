#include "Acrome-SMD.h"
uint8_t ID = 0x00;

union{
  uint64_t _u64;
  uint8_t  _u8[8];
}conv;

uint64_t* ptr64;

Red myRed(ID, Serial);

uint64_t sensorsBIT= 0;

#define POSITION_CONTROL    0
#define VELOCITY_CONTROL    1

float cpr = 4741.0;
float rpm = 100.0;

float analog;
float positionSetpoint = 40000.0;
float velocitySetpoint = 100.0;

float position;
float prevPosition = 0;
float velocity;
float prevVelocity = 0;
int i = 0;
int flag = POSITION_CONTROL;

uint16_t distance = 0;

uint8_t* ur8;
uint32_t deneme = 895;
void setup() {
    myRed.begin(115200);
    delay(5);
    
    myRed.setMotorCPR(cpr);
    delay(5);
    myRed.setMotorRPM(rpm);
    delay(50);

    //myRed.tune();
    //delay(30000);

    myRed.setOperationMode(VelocityControl);
    delay(5);
    myRed.setpoint(VelocityControl, velocitySetpoint);
    myRed.torqueEnable(1);
    delay(50);
    //myRed.availableSensors();

    myRed.scanSensors();


    Serial.println("\n");
    myRed.availableSensors();
}
void loop() {
  distance = myRed.getDistance(iDistance_2);

  Serial.println();
  Serial.println(distance);
  Serial.println();

}
