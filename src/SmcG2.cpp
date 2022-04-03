#include "SmcG2.h"


void SmcG2I2C::command(uint8_t cmd) 
{
    _i2c->beginTransmission(_address);
    _i2c->write(cmd);  // Exit safe start
    _lastError = _i2c->endTransmission();

}