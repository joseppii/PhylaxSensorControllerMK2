#include "SmcG2.h"

void SmcG2I2C::command(uint8_t cmd) 
{
    _i2c->beginTransmission(_address);
    _i2c->write(cmd);
    _lastError = _i2c->endTransmission();
}

void SmcG2I2C::write(uint8_t reg, uint8_t *buff, uint32_t len)
{
    _i2c->beginTransmission(_address);
    _i2c->write(reg);
    _i2c->write(buff, (uint8_t)len);
    _lastError = _i2c->endTransmission();
}

void SmcG2I2C::read(uint8_t reg, uint8_t *buff, uint32_t len)
{
  _i2c->beginTransmission(_address);
  _i2c->write(reg);
  _i2c->endTransmission(false); // Send repeated start

  uint32_t num_received = _i2c->requestFrom(_address, len);

  if (num_received == len)
  {
    for (uint8_t i = 0; i < len; i++)
    {
      buff[i] = _i2c->read();
    }
    _lastError = 0;
  }
  else
  {
    _lastError = 1;
  }

  if (len != 0)
  {
    _lastError = 1;
  }
  _lastError = 0;    
}