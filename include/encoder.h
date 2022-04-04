#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>
#include <ESP32Encoder.h>

class PololuEncoder
{
  public:
    uint32_t _encoderCountPrev;
   
    PololuEncoder(int gearing, int cpr, float radius);
    
    void init(uint8_t neg, uint8_t pos);
    void update(uint32_t &encoderCount);
    
  //private:
    float _gears; 
    float _cpr; 
    float _wheelRadius;
    float _pathDistance;    

    float _dPhiL;
    float _dPhiR;

    float _th;
    float _x;
    float _y;

    float _enc2rev;
    float _enc2rad;
    float _enc2wheel;

    ESP32Encoder *_encoder;
};

#endif