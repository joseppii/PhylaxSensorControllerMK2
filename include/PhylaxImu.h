
#include "RosImu.h"
#include "ICM_20948.h"

class PhylaxImu : public RosImu
{
public:
    PhylaxImu() {};
    virtual bool startImu(TwoWire &i2c, uint8_t ad0);
    virtual void readData();
private:
   ICM_20948_I2C myICM_; //Store an ICM_20948_I2C object

    bool configure();
};