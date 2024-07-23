#include "Acrome-SMD.h"
#include "string.h"

#define DEBUG_EN (0)

#define DEVICE_FAMILY (0xBA)
#define HEADER (0x55)
#define CONSTANT_REG_SIZE (10)
#define MAX_PACKAGE_SIZE (1024)

#define BUS_TIMEOUT (50)

//#define PING                (0x00)   //   Re-Defined in Arduino Mega and Mega 2560
#define WRITE               (0x01)
#define WRITE_ACK           (0x80 | 0x01)
#define READ                (0x02)
#define EEPROM_WRITE        (0x03)
#define EEPROM_WRITE_ACK    (0xFF)
#define SCAN_MODULES        (0x04)
#define REBOOT              (0x05)
#define RESET_ENC           (0x06)
#define TUNE                (0x07)

#define FACTORY_RESET       (0x17)
#define ERROR_CLEAR         (0x18)
#define ACK                 (0x80)
#define SYNC_WRITE          (0x40 | 0x01)

#define DATA(X) (CONSTANT_REG_SIZE - sizeof(uint32_t) + X)

uint8_t _data[1024] = {0};

String sensorTypes[] = {
    "BUTTON",
    "LIGHT",
    "BUZZER",
    "JOYSTICK",
    "DISTANCE",
    "QTR",
    "SERVO",
    "POTANTIOMETER",
    "RGB",
    "IMU"
    };

union converter
{
    float _float;
    uint32_t _u64;
    uint32_t _u32;
    int32_t _i32;
    uint16_t _u16[2];
    uint8_t _u8[8];
} conv;

inline void _addFloat(uint8_t index, float val)
{
    conv._float = val;
    _data[index] = conv._u8[0];
    _data[index + 1] = conv._u8[1];
    _data[index + 2] = conv._u8[2];
    _data[index + 3] = conv._u8[3];
}

inline void _addU32(uint8_t index, uint32_t val)
{
    conv._u32 = val;
    _data[index] = conv._u8[0];
    _data[index + 1] = conv._u8[1];
    _data[index + 2] = conv._u8[2];
    _data[index + 3] = conv._u8[3];
}

inline void _addi32(uint8_t index, uint32_t val)
{
    conv._i32 = val;
    _data[index] = conv._u8[0];
    _data[index + 1] = conv._u8[1];
    _data[index + 2] = conv._u8[2];
    _data[index + 3] = conv._u8[3];
}

inline void _addU16(uint8_t index, uint16_t val)
{
    conv._u16[0] = val;
    _data[index] = conv._u8[0];
    _data[index + 1] = conv._u8[1];
}

inline float _getFloat(uint8_t index)
{
    conv._u8[0] = _data[index];
    conv._u8[1] = _data[index + 1];
    conv._u8[2] = _data[index + 2];
    conv._u8[3] = _data[index + 3];

    return conv._float;
}

inline uint8_t *_getU8_ptr(uint8_t index)
{
    conv._u8[0] = _data[index];
    conv._u8[1] = _data[index + 1];
    conv._u8[2] = _data[index + 2];
    conv._u8[3] = _data[index + 3];
    conv._u8[4] = _data[index + 4];
    conv._u8[5] = _data[index + 5];
    conv._u8[6] = _data[index + 6];
    conv._u8[7] = _data[index + 7];

    return conv._u8;
}

inline uint32_t _getU32(uint8_t index)
{
    conv._u8[0] = _data[index];
    conv._u8[1] = _data[index + 1];
    conv._u8[2] = _data[index + 2];
    conv._u8[3] = _data[index + 3];

    return conv._u32;
}

inline uint16_t _getU16(uint8_t index)
{
    conv._u8[0] = _data[index];
    conv._u8[1] = _data[index + 1];

    return conv._u16[0];
}

inline bool _checkModuleID(uint8_t id){
    if( (id > 5) || (id < 0) ){
        return false;
    }
    else{
        return true;
    }
}

uint32_t _CRC32(uint8_t *data, uint8_t len)
{
    uint32_t i, j;
    uint32_t crc, msb;

    crc = 0xFFFFFFFF;
    for (i = 0; i < len; i++)
    {
        crc ^= (((uint32_t)data[i]) << 24);
        for (j = 0; j < 8; j++)
        {
            msb = crc >> 31;
            crc <<= 1;
            crc ^= (0 - msb) & 0x04C11DB7;
        }
    }
    return crc;
}

void _addCRC()
{
    conv._u32 = _CRC32(_data, _data[iPackageSize] - sizeof(conv._u32));
    _data[_data[iPackageSize] - 4] = conv._u8[0];
    _data[_data[iPackageSize] - 3] = conv._u8[1];
    _data[_data[iPackageSize] - 2] = conv._u8[2];
    _data[_data[iPackageSize] - 1] = conv._u8[3];
}

inline bool _checkCRC()
{
    uint32_t calc = _CRC32(_data, (_data[iPackageSize] - sizeof(uint32_t)));
    conv._u8[0] = _data[_data[iPackageSize] - 4];
    conv._u8[1] = _data[_data[iPackageSize] - 3];
    conv._u8[2] = _data[_data[iPackageSize] - 2];
    conv._u8[3] = _data[_data[iPackageSize] - 1];
    if (conv._u32 == calc)
    {
        return true;
    }
    else
    {
        return false;
    }
}

inline bool headerCheck(uint8_t ID, uint8_t Command)
{
    if (_data[iHeader] == 0x55)
        if (_data[iPackageSize] >= CONSTANT_REG_SIZE && _data[iPackageSize] <= MAX_PACKAGE_SIZE)
            if (_data[iDeviceID] == ID)
                if (_data[iDeviceFamily] == 0xBA)
                    if (_data[iCommand] == Command)
                        return true;
    return false;
}

void Red::_write2serial(uint8_t debug)
{ // tested +
    if (debug == 0)
    {
        _port->write(_data, _data[iPackageSize]);
    }
    else
    {
        for (uint8_t i = 0; i < _data[iPackageSize]; i++)
        {
            _port->print(_data[i]);
            _port->print(',');
        }
        _port->println();
    }
    delayMicroseconds(_delayMicros);
}

bool Red::_readFromSerial(uint8_t expected_bytes, uint8_t debug)
{
    if (debug == 0)
    {
        memset(&(_data[expected_bytes - sizeof(uint32_t)]), 0, sizeof(uint32_t));
        uint32_t start = millis();
        do
        {
            if (_port->available() >= expected_bytes)
            {
                _port->readBytes(_data, expected_bytes);
                return true;
            }
        } while ((millis() - start) < _timeout);
    }
    return false;
}

void Red::begin()
{
    _port->begin(_baudrate);
}


void Red::end()
{ // tested +
    _port->end();
}

Red::Red(uint8_t ID, HardwareSerial &port, uint32_t baudrate)
{ // tested $
    _devId = ID;
    _timeout = BUS_TIMEOUT;
    _port = &port;
    _baudrate = baudrate;
    _delayMicros = (120000000U / baudrate); // updated. ====//==== _delayMicros = (105000000U / baudrate);  1 One byte long delay time + %5 for safety
}

bool Red::Ping()
{ // tested +
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE;
    _data[iCommand] = PING;
    _data[iStatus] = 0x00;

    _addCRC();
    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, PING) == true))
        {
            return true;
        }
    }
    return false;
}

void Red::FactoryReset()
{ // tested $
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE;
    _data[iCommand] = FACTORY_RESET;
    _data[iStatus] = 0x00;

    _addCRC();

    _write2serial();
}

void Red::Reboot()
{ // tested $
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE;
    _data[iCommand] = REBOOT;
    _data[iStatus] = 0x00;

    _addCRC();

    _write2serial();
}

void Red::EEPROMWrite()
{ // tested $
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE;
    _data[iCommand] = EEPROM_WRITE;
    _data[iStatus] = 0x00;

    _addCRC();

    _write2serial();
}

void Red::tune()
{ // tested +
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint8_t) + 1);
    _data[iCommand] = TUNE;
    _data[iStatus] = 0x00;

    _addCRC();

    _write2serial();
}

void Red::setMotorCPR(float cpr)
{ // tested $
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(float) + 1);
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iMotorCPR;

    _addFloat(DATA(1), cpr);

    _addCRC();

    _write2serial();
}

void Red::setMotorRPM(float rpm)
{ // tested $
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(float) + 1);
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iMotorRPM;

    _addFloat(DATA(1), rpm);

    _addCRC();

    _write2serial();
}

void Red::setControlParameters(tOperationMode mode, float P, float I, float D, float FF, float DB)
{ // tested $
    // updated.
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(float) + 1) * 5;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    switch (mode & 0x7F)
    {
    case PositionControl:
        _data[DATA(0)] = iPositionPGain;
        _data[DATA(5)] = iPositionIGain;
        _data[DATA(10)] = iPositionDGain;
        _data[DATA(15)] = iPositionFF;
        _data[DATA(20)] = iPositionDeadband;
        break;
    case VelocityControl:
        _data[DATA(0)] = iVelocityPGain;
        _data[DATA(5)] = iVelocityIGain;
        _data[DATA(10)] = iVelocityDGain;
        _data[DATA(15)] = iVelocityFF;
        _data[DATA(20)] = iVelocityDeadband;
        break;
    case TorqueControl:
        _data[DATA(0)] = iTorquePGain;
        _data[DATA(5)] = iTorqueIGain;
        _data[DATA(10)] = iTorqueDGain;
        _data[DATA(15)] = iTorqueFF;
        _data[DATA(20)] = iTorqueDeadband;
        break;
    }

    _addFloat(DATA(1), P);
    _addFloat(DATA(6), I);
    _addFloat(DATA(11), D);
    _addFloat(DATA(16), FF);
    _addFloat(DATA(21), DB);

    _addCRC();

    _write2serial();
}

void Red::setpoint(tOperationMode mode, float setpoint)
{ // tested $
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + sizeof(float) + 1;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    switch (mode & 0x7F)
    {
    case PositionControl:
        _data[DATA(0)] = iSetPosition;
        break;
    case VelocityControl:
        _data[DATA(0)] = iSetVelocity;
        break;
    case TorqueControl:
        _data[DATA(0)] = iSetTorque;
        break;
    case PWMControl:
        _data[DATA(0)] = iSetDutyCycle;
        break;
    default:
        break;
    }

    _addFloat(DATA(1), setpoint);

    _addCRC();

    _write2serial();
}

void Red::setPositionLimits(uint32_t min, uint32_t max)
{ // tested +
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint32_t) + 1) * 2;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iMinimumPositionLimit;
    _addU32(DATA(1), min);
    _data[DATA(5)] = iMaximumPositionLimit;
    _addU32(DATA(6), max);

    _addCRC();

    _write2serial();
}
void Red::setVelocityLimit(uint16_t limit)
{ // tested +
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint16_t) + 1) * 1;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iVelocityLimit;
    _addU16(DATA(1), limit);

    _addCRC();

    _write2serial();
}
void Red::setTorqueLimit(uint16_t limit)
{ // tested +
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint16_t) + 1) * 1;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iTorqueLimit;
    _addU16(DATA(1), limit);

    _addCRC();

    _write2serial();
}

void Red::torqueEnable(uint8_t en)
{ // tested $
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint8_t) + 1) * 1;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iTorqueEnable;
    _data[DATA(1)] = en;

    _addCRC();

    _write2serial();
}

void Red::setOperationMode(tOperationMode mode)
{ // tested +
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint8_t) + 1) * 1;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iOperationMode;
    _data[DATA(1)] = mode;

    _addCRC();

    _write2serial();
}

void Red::goTo(float setPoint, float SCurveTimeSet, float SCurveMaxVelocity, float SCurveAccel)
{ // Tested +
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(float)* 4 + 4);
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(2)]=iSCurvesetpoint;
    _addFloat(DATA(3),setPoint);

    _data[DATA(7)]=iSCurveAccel;
    _addFloat(DATA(8),SCurveAccel);

    _data[DATA(12)]=iSCurveMaxVelocity;
    _addFloat(DATA(13),SCurveMaxVelocity);

    _data[DATA(17)]=iSCurveTime;
    _addFloat(DATA(18),SCurveTimeSet);

    _addCRC();

    _write2serial();
}

void Red::setVelocityAccel(uint16_t accel)
{ // Tested +
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint16_t) + 1) * 1;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iSetVelocityAcceleration;
    _addU16(DATA(1), accel);

    _addCRC();

    _write2serial();
}


void Red::setDeviceID(uint8_t ID)
{ // tested $
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint8_t) + 1) * 1;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iDeviceID;
    _data[DATA(1)] = ID;

    _addCRC();

    _write2serial();

    _devId = ID;   // update devID to keep communicate
}

void Red::setTimeout(uint16_t timeout)
{
    _timeout = timeout;
}

void Red::setBaudrate(uint32_t baud)
{ // tested +
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint32_t) + 1) * 1;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iBaudrate;
    _addU32(DATA(1), baud);

    _addCRC();

    _write2serial();
}

float Red::getPosition()
{
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iPresentPosition;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(float) + 1) * 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == iPresentPosition)
            {
                return _getFloat(DATA(1));
            }
        }
    }
    return 0;
}

float Red::getVelocity()
{
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iPresentVelocity;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(float) + 1) * 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == iPresentVelocity)
            {
                return _getFloat(DATA(1));
            }
        }
    }
    return 0;
}

float Red::getTorque()
{
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iMotorCurrent;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(float) + 1) * 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == iMotorCurrent)
            {
                return _getFloat(DATA(1));
            }
        }
    }
    return 0;
}

uint16_t Red::getAnalogPort()
{
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iAnalogPort;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(uint16_t) + 1) * 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == iAnalogPort)
            {
                return _getU16(DATA(1));
            }
        }
    }
    return 0;
}

uint8_t *Red::scanModules()
{
    //Tested (Succesful)

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint8_t) + 1) * 1;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iSetScanModuleMode;
    _data[DATA(1)] = 0;

    _addCRC();
    _write2serial();

    delay(50);
    
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE;
    _data[iCommand] = SCAN_MODULES;
    _data[iStatus] = 0x00;

    _addCRC();
    _write2serial();

    delay(5500);

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint8_t) * 1);
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iConnectedBitfield;
    

    _addCRC();
    _write2serial();

    // package size , SCAN_MODULES , 64bit
    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(uint64_t) + 1) * 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == iConnectedBitfield)
            {
                // update and fill local sensor value
                _sensors[0] = _data[DATA(1)];
                _sensors[1] = _data[DATA(2)];
                _sensors[2] = _data[DATA(3)];
                _sensors[3] = _data[DATA(4)];
                _sensors[4] = _data[DATA(5)];
                _sensors[5] = _data[DATA(6)];
                _sensors[6] = _data[DATA(7)];
                _sensors[7] = _data[DATA(8)];

                return _sensors;
            }
        }
    }
    return 0;
}

void Red::printAvailableSensors(HardwareSerial &port)
{
    uint8_t IDs[100] = {0};
    uint8_t SENSOR_TYPE = 0;
    uint8_t SENSOR_ID = 0;
    uint8_t sensor_count = 0;
    for (int byte = 0; byte <= 7; byte++)
    {
        for (int bit = 0; bit <= 7; bit++)
        {
            if ((_sensors[byte] & (1 << bit)) == (1 << bit))
            {
                int cursor = bit + (byte << 3);
                if (cursor >= ID_RNG_IMU_L)
                {
                    SENSOR_TYPE = IMU;
                    SENSOR_ID = cursor - ID_RNG_IMU_L +1;
                }
                else if (cursor >= ID_RNG_RGB_L)
                {
                    SENSOR_TYPE = RGB;
                    SENSOR_ID = cursor - ID_RNG_RGB_L +1;
                }
                else if (cursor >= ID_RNG_POT_L)
                {
                    SENSOR_TYPE = POTANTIOMETER;
                    SENSOR_ID = cursor - ID_RNG_POT_L +1;
                }
                else if (cursor >= ID_RNG_SERVO_L)
                {
                    SENSOR_TYPE = SERVO;
                    SENSOR_ID = cursor - ID_RNG_SERVO_L +1;
                }
                else if (cursor >= ID_RNG_QTR_L)
                {
                    SENSOR_TYPE = QTR;
                    SENSOR_ID = cursor - ID_RNG_QTR_L +1;
                }
                else if (cursor >= ID_RNG_DISTANCE_L)
                {
                    SENSOR_TYPE = DISTANCE;
                    SENSOR_ID = cursor - ID_RNG_DISTANCE_L +1;
                }
                else if (cursor >= ID_RNG_JOYSTICK_L)
                {
                    SENSOR_TYPE = JOYSTICK;
                    SENSOR_ID = cursor - ID_RNG_JOYSTICK_L +1;
                }
                else if (cursor >= ID_RNG_BUZZER_L)
                {
                    SENSOR_TYPE = BUZZER;
                    SENSOR_ID = cursor - ID_RNG_BUZZER_L +1;
                }
                else if (cursor >= ID_RNG_LIGHT_L)
                {
                    SENSOR_TYPE = LIGHT;
                    SENSOR_ID = cursor - ID_RNG_LIGHT_L +1;
                }
                else
                {
                    SENSOR_TYPE = BUTTON;
                    SENSOR_ID = cursor - ID_RNG_BUTTON_L + 1;
                }
                IDs[sensor_count * 2] = SENSOR_TYPE;
                IDs[sensor_count * 2 + 1] = SENSOR_ID;
                sensor_count++;
            }
        }
    }

    //printing
    port.print("Scanned Modules:\n");
    for (int j = 0; j < sensor_count; j++)
    {
        port.println();
        port.print(sensorTypes[IDs[j * 2]]);
        port.print("::");
        port.print(IDs[j * 2 + 1]);
    }
    port.println();
}

void Red::setConnectedModules(uint8_t sensors[], uint8_t number_of_connected_sensors){
    // Tested ++

    uint8_t ManualBuzzerByte = 0;
    uint8_t ManualServoByte= 0;
    uint8_t ManualRGBByte= 0;
    uint8_t ManualButtonByte = 0;
    uint8_t ManualLightByte = 0;
    uint8_t ManualJoystickByte = 0;
    uint8_t ManualQTRByte = 0;
    uint8_t ManualDistanceByte = 0;
    uint8_t ManualPotByte = 0;
    uint8_t ManualIMUByte = 0;

    for (int i = 0; i < number_of_connected_sensors; i++)
    {
        switch (sensors[i]){
            case iBuzzer_1 ... iBuzzer_5 :
                ManualBuzzerByte |= 1<<(sensors[i] - iBuzzer_1);
                break;

            case iServo_1 ... iServo_5 :
                ManualServoByte |= 1<<(sensors[i] - iServo_1);
                break;

            case iRGB_1 ... iRGB_5 :
                ManualRGBByte |= 1<<(sensors[i] - iRGB_1);
                break;
            
            case iButton_1 ... iButton_5 :
                ManualButtonByte |= 1<<(sensors[i] - iButton_1);
                break;
            
            case iLight_1  ...  iLight_5:
                ManualLightByte |= 1<<(sensors[i] - iLight_1);
                break;

            case iJoystick_1 ... iJoystick_5 :
                ManualJoystickByte |= 1<<(sensors[i] - iJoystick_1);
                break;

            case iQTR_1 ... iQTR_5 :
                ManualQTRByte |= 1<<(sensors[i] - iQTR_1);
                break;
            
            case iDistance_1 ... iDistance_5 :
                ManualDistanceByte |= 1<<(sensors[i] - iDistance_1);
                break;
            
            case iPot_1 ... iPot_5 :
                ManualPotByte |= 1<<(sensors[i] - iPot_1);
                break;
            
            case iIMU_1 ... iIMU_5 :
                ManualIMUByte |= 1<<(sensors[i] - iIMU_1);
                break;
        }
    }

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint8_t) + 1) * 11;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iSetScanModuleMode;
    _data[DATA(1)] = 1;

    _data[DATA(2)] = iSetManualBuzzer;
    _data[DATA(3)] = ManualBuzzerByte;

    _data[DATA(4)] = iSetManualServo;
    _data[DATA(5)] = ManualServoByte;

    _data[DATA(6)] = iSetManualRGB;
    _data[DATA(7)] = ManualRGBByte;

    _data[DATA(8)] = iSetManualButton;
    _data[DATA(9)] = ManualButtonByte;

    _data[DATA(10)] = iSetManualLight;
    _data[DATA(11)] = ManualLightByte;

    _data[DATA(12)] = iSetManualJoystick;
    _data[DATA(13)] = ManualJoystickByte;

    _data[DATA(14)] = iSetManualQTR;
    _data[DATA(15)] = ManualQTRByte;

    _data[DATA(16)] = iSetManualDistance;
    _data[DATA(17)] = ManualDistanceByte;

    _data[DATA(18)] = iSetManualPot;
    _data[DATA(19)] = ManualPotByte;

    _data[DATA(20)] = iSetManualIMU;
    _data[DATA(21)] = ManualIMUByte;

    _addCRC();
    _write2serial();

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint8_t) + 1) * 1;
    _data[iCommand] = SCAN_MODULES;
    _data[iStatus] = 0x00;

    _addCRC();
    _write2serial();
}

uint8_t Red::getButton(int buttonID)
{
    if(!_checkModuleID(buttonID)){
        return 0;
    }
    uint8_t ProtocolID = iButton_1 + buttonID -1;

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(uint8_t) + 1) * 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == ProtocolID)
            {
                return _data[DATA(1)];
            }
        }
    }
    return 0;
}

uint16_t Red::getLight(int lightID)
{
    if(!_checkModuleID(lightID)){
        return 0;
    }
    uint8_t ProtocolID = iLight_1 + lightID -1;

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(uint16_t) + 1) * 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == ProtocolID)
            {
                return _getU16(DATA(1));
            }
        }
    }
    return 0;
}

void Red::setBuzzer(int buzzerID, uint32_t frequency)
{
    if(!_checkModuleID(buzzerID)){
        return;
    }
    uint8_t ProtocolID = iBuzzer_1 + buzzerID -1;


    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint32_t) + 1);
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;
    _addU32(DATA(1), frequency);

    _addCRC();
    _write2serial();
}

uint8_t Red::getJoystickButton(int joystickID)
{
    if(!_checkModuleID(joystickID)){
        return 0;
    }
    uint8_t ProtocolID = iJoystick_1 + joystickID -1;

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(int32_t))* 2 + sizeof(uint8_t) + 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == ProtocolID)
            {
                return _data[DATA((sizeof(int32_t) * 2 + 1))];
            }
        }
    }
    return 0;
}

int32_t Red::getJoystickX(int joystickID)
{
    if(!_checkModuleID(joystickID)){
        return 0;
    }
    uint8_t ProtocolID = iJoystick_1 + joystickID -1;

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(int32_t))* 2 + sizeof(uint8_t) + 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == ProtocolID)
            {
                int32_t returning = _getU32(DATA(1));
                return returning;
            }
        }
    }
    return 0;
}

int32_t Red::getJoystickY(int joystickID)
{
    if(!_checkModuleID(joystickID)){
        return 0;
    }
    uint8_t ProtocolID = iJoystick_1 + joystickID -1;

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(int32_t))* 2 + sizeof(uint8_t) + 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == ProtocolID)
            {
                int32_t returning = _getU32(DATA(1 + sizeof(int32_t)));
                return returning;
            }
        }
    }
    return 0;
}

void Red::setServo(int servoID, uint8_t angle)
{
    if(!_checkModuleID(servoID)){
        return;
    }
    uint8_t ProtocolID = iServo_1 + servoID -1;


    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 2;
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;
    _data[DATA(1)] = angle;

    _addCRC();
    _write2serial();
}

float Red::getRollAngle(int IMU_ID)
{
    if(!_checkModuleID(IMU_ID)){
        return 0;
    }
    uint8_t ProtocolID = iIMU_1 + IMU_ID -1;

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(float)) * 2 + 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == ProtocolID)
            {
                return _getFloat(DATA(1));
            }
        }
    }
    return 0;
}

float Red::getPitchAngle(int IMU_ID)
{
    if(!_checkModuleID(IMU_ID)){
        return 0;
    }
    uint8_t ProtocolID = iIMU_1 + IMU_ID -1;

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(float)) * 2 + 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == ProtocolID)
            {
                return _getFloat(DATA(sizeof(float) + 1));
            }
        }
    }
    return 0;
}



uint16_t Red::getDistance(int distanceID)
{
    if(!_checkModuleID(distanceID)){
        return 0;
    }
    uint8_t ProtocolID = iDistance_1 + distanceID -1;

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(uint16_t) + 1) * 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == ProtocolID)
            {
                return _getU16(DATA(1));
            }
        }
    }
    return 0;
}

QTRValues Red::getQTR(int qtrID)
{
    if(!_checkModuleID(qtrID)){
        return {0};
    }
    uint8_t ProtocolID = iQTR_1 + qtrID -1;

    QTRValues qtrlist;

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(uint8_t) + 1) * 3) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == ProtocolID)
            {
                qtrlist.LeftValue =    _data[DATA(1)];
                qtrlist.MiddleValue =  _data[DATA(2)];
                qtrlist.RightValue =   _data[DATA(3)];
                
                return qtrlist;
            }
        }
    }

    return qtrlist;
}



uint8_t Red::getPotentiometer(int potentiometerID)
{
    if(!_checkModuleID(potentiometerID)){
        return 0;
    }
    uint8_t ProtocolID = iPot_1 + potentiometerID -1;

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + 1;
    _data[iCommand] = READ;
    _data[iStatus]  = 0x00;

    _data[DATA(0)] = ProtocolID;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + (sizeof(uint8_t) + 1) * 1) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if (_data[DATA(0)] == ProtocolID)
            {
                return _data[DATA(1)];
            }
        }
    }
    return 0;

}

void Red::setRGB(int rgbID, uint8_t red, uint8_t green, uint8_t blue)
{
    if(!_checkModuleID(rgbID)){
        return;
    }
    uint8_t ProtocolID = iRGB_1 + rgbID -1;
    
    uint32_t u_red = red;
    uint32_t u_green = green;
    uint32_t u_blue = blue;
    
    uint32_t color = u_red + (u_green << 8) + (u_blue << 8*2);

    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint32_t) + 1);
    _data[iCommand] = WRITE;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = ProtocolID;
    _addU32(DATA(1), color);

    _addCRC();
    _write2serial();
}


uint32_t Red::getSoftwareVersion()
{
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint8_t) * 1);
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iSoftwareVersion;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + sizeof(uint8_t) * 1 + sizeof(uint32_t), DEBUG_EN) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if ((_data[DATA(0)] == iSoftwareVersion))
            {
                return _getU32(DATA(1));
            }
        }
    }

    return 0;
}

uint32_t Red::getHardwareVersion()
{
    _data[iHeader] = HEADER;
    _data[iDeviceID] = _devId;
    _data[iDeviceFamily] = DEVICE_FAMILY;
    _data[iPackageSize] = CONSTANT_REG_SIZE + (sizeof(uint8_t) * 1);
    _data[iCommand] = READ;
    _data[iStatus] = 0x00;

    _data[DATA(0)] = iHardwareVersion;

    _addCRC();

    _write2serial();

    if (_readFromSerial(CONSTANT_REG_SIZE + sizeof(uint8_t) * 1 + sizeof(uint32_t), DEBUG_EN) == true)
    {
        if ((_checkCRC() == true) && (headerCheck(_devId, READ) == true))
        {
            if ((_data[DATA(0)] == iHardwareVersion))
            {
                return _getU32(DATA(1));
            }
        }
    }

    return 0;
}

