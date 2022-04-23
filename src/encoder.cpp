#include "encoder.h"


PololuEncoder::PololuEncoder(int gearing, int cpr, float radius, uint8_t negative, uint8_t positive):
  _encoderCountPrev(0),
  _gears(gearing),
  _cpr(cpr),
  _wheelRadius(radius),
  _negativePin(negative),
  _positivePin(positive),
  _pathDistance(0),
  _dPhiL(0),
  _dPhiR(0),
  _th(0),
  _x(0),
  _y(0)
{
  _encoder = new ESP32Encoder();
}

void PololuEncoder::init()
{
  // Enable the weak pull up resistors
  ESP32Encoder::useInternalWeakPullResistors=UP;
  
  _encoder->attachFullQuad(_negativePin, _positivePin);

  //Clear encoder count
  _encoder->clearCount();
 
  _enc2rev = 1.0/1632.0;
  _enc2rad = _enc2rev * 2 * PI;
  _enc2wheel = _enc2rad * _wheelRadius; 
}

void PololuEncoder::update(uint32_t &encoderCount)
{
  encoderCount = _encoder->getCount();
}