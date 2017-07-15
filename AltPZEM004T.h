#ifndef AltPZEM004T_H
#define AltPZEM004T_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <AltSoftSerial.h>
#include <IPAddress.h>

 #define INPUT_CAPTURE_PIN       2 // receive
 #define OUTPUT_COMPARE_A_PIN        3 // transmit


struct PZEMCommand {
    uint8_t command;
    uint8_t addr[4];
    uint8_t data;
    uint8_t crc;
};

class AltPZEM004T
{
public:
    AltPZEM004T(uint8_t receivePin, uint8_t transmitPin);
    AltPZEM004T(HardwareSerial *port);
    ~AltPZEM004T();

    void setReadTimeout(unsigned long msec);
    unsigned long readTimeout() {return _readTimeOut;}

    float voltage(const IPAddress &addr);
    float current(const IPAddress &addr);
    float power(const IPAddress &addr);
    float energy(const IPAddress &addr);

    bool setAddress(const IPAddress &newAddr);
    bool setPowerAlarm(const IPAddress &addr, uint8_t threshold);

private:
    Stream *serial;

    unsigned long _readTimeOut;
    bool _isSoft;

    void send(const IPAddress &addr, uint8_t cmd, uint8_t data = 0);
    bool recieve(uint8_t resp, uint8_t *data = 0);

    uint8_t crc(uint8_t *data, uint8_t sz);
};

#endif // AltPZEM004T_H
