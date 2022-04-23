#include "Odometer.h"
#include "encoder.h"

Odometer::Odometer(PololuEncoder& encoder1, PololuEncoder& encoder2) : encoder1_(&encoder1), encoder2_(&encoder2)
{

}

void Odometer::init(/*ros::NodeHandle &nh,*/ float track)
{
  track_ = track;
}

void Odometer::updateEncoder()
{
  uint32_t encoder1Count;
  uint32_t encoder2Count;

  encoder1_->update(encoder1Count);
  encoder2_->update(encoder2Count);

  int32_t dEncoder1 = (encoder1Count - encoder1CountPrev_);
  int32_t dEncoder2 = (encoder2Count - encoder2CountPrev_);

  //update the angle increment in radians
  float dphi1 = ((float)dEncoder1 * encoder1_->_enc2rad);
  float dphi2 = ((float)dEncoder2 * encoder2_->_enc2rad);
  
  //for encoder index and motor position switching (Right is 1, Left is 2)
  dPhiR_ = dphi1;
  dPhiL_ = dphi2;
  
  encoder1CountPrev_ = encoder1Count;
  encoder2CountPrev_ = encoder2Count;
}

void Odometer::evaluateRobotPose(unsigned long diff_time)
{
  float dTh = encoder1_->_wheelRadius/(track_) *(dPhiR_ - dPhiL_);
  float dist = encoder1_->_wheelRadius *(dPhiR_ + dPhiL_) / 2;
  float dx = encoder1_->_wheelRadius/2 * (cos(th_)*dPhiR_ + cos(th_)*dPhiL_);
  float dy = encoder1_->_wheelRadius/2 * (sin(th_)*dPhiR_+ + sin(th_)*dPhiL_);
  long dt = float(diff_time)/1000;
  
  th_+= dTh;
  x_+=dx;
  y_+=dy;
  vx_ = dist/dt;
  vTh_ = dTh/dt;
  pathDistance_ = pathDistance_ + sqrt(dx*dx + dy*dy);

  Serial.println("Math stuff = "+String((int32_t)pathDistance_));
}