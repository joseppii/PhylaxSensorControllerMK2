// Copyright (C) Joseph Piperakis See LICENSE.txt for details.

/// \file SmcG2.h
///
/// This is the main header file for the Pololu Simple Motor Controller G2 library
/// for Arduino/ESP32.
///
/// For more information about the library, see the main repository at:
/// https://github.com/joseppii/smc-g2-arduino

#pragma once

#include <Arduino.h>
#include <Wire.h>

/// This enum defines the Smc G2 command bytes which used for both serial and
/// I2C interfaces.  
enum class SmcG2Command:uint8_t
{
    exitSafeStart           = 0x83,
    setTargetSpeedFwd       = 0x85,
    setTargetSpeedRev       = 0x86,
};

/// This is a base class used to represent a connection to a Pololu Simple Motor 
/// Controller (Smc) G2. This class provides high-level functions for sending 
/// commands to the Smc and reading data from it.
///
/// See the subclasses of this class, SmcG2Serial and SmcG2I2C.
class SmcG2Base
{
public:
    /// Returns 0 if the last communication with the device was successful, and
    /// non-zero if there was an error.
    uint8_t getLastError()
    {
        return _lastError;
    }

    void exitSafeStart()
    {
        command(SmcG2Command::exitSafeStart);
    }

protected:
    /// Zero if the last communication with the device was successful, non-zero
    /// otherwise.
    uint8_t _lastError = 0;

private:
    // Convenience functions for type casting a SmcG2Command to a uint8_t.
    void command(SmcG2Command cmd)
    {
        command((uint8_t)cmd);
    }

    //methods implemented by subclasses
    virtual void command(uint8_t cmd) = 0;
};

/// Implements an I2C connection to a Smc G2.
///
/// For the high-level commands you can use on this object, see JrkG2Base.
class SmcG2I2C : public SmcG2Base
{
public:
    /// Creates a new SmcG2I2C object that will use the `TwoWire` object to
    /// communicate with the Smc over I2C.
    ///
    /// The `address` parameter specifies the 7-bit I2C address to use, and it
    /// must match the Smc's "Device number" setting.  It defaults to 11.
    ///
    /// This constructor only uses the least-significat 7 bits of the address,
    /// since I2C addresses can only go up to 127 and we want things to always
    /// just work as long as the address is the same as the "Device number".
    SmcG2I2C(uint8_t address, TwoWire& i2c) : _address(address & 0x7F), _i2c(&i2c)
    {
    }

     /// Returns the I2C address of the object.
    uint8_t getAddress() { return _address; }

private:
    const uint8_t _address;
    TwoWire* const _i2c;

    void command(uint8_t cmd);
};