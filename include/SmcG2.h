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

/// This enum defines the Smc G2 command bytes which used for botj serial and
/// I2C interfaces.  
enum class SmcG2Command
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
protected:
  /// Zero if the last communication with the device was successful, non-zero
  /// otherwise.
  uint8_t _lastError = 0;
  
private:
    virtual void command(uint8_t cmd) = 0;
};