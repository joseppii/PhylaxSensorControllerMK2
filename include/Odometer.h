#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

class PololuEncoder;

class Odometer
{
public:
    Odometer(PololuEncoder& encoder1, PololuEncoder& encoder2);
    void init(float track);
    void updateEncoder();
    void evaluateRobotPose(unsigned long diff_time);

private:

    PololuEncoder* const encoder1_;
    PololuEncoder* const encoder2_;
    uint32_t encoder1CountPrev_;
    uint32_t encoder2CountPrev_;

    float track_; 
    float dPhiL_;
    float dPhiR_;

    float th_;
    float x_;
    float y_;
    float vx_;
    float vTh_;
    float pathDistance_;   
};

#endif /* ODOMETRY_H */