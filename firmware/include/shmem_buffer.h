// AUTOGENERATED FILE. DO NOT MODIFY.
// Generated via `npm run gen-shmem`

// Instance: b716f7f0-1c8e-4af6-8271-386c9c6c1276

#pragma once
#include <stdint.h>

#define FIRMWARE_IDENT 118

struct Data {
  uint16_t ioConfig;
  uint8_t firmwareIdent;
  uint8_t status;
  bool heartbeat;
  uint8_t builtinConfig;
  bool builtinDioValues[4];
  int16_t extIoValues[5];
  uint16_t analog[2];
  int16_t leftMotor;
  int16_t rightMotor;
  uint16_t batteryMillivolts;
  bool resetLeftEncoder;
  bool resetRightEncoder;
  int16_t leftEncoder;
  int16_t rightEncoder;
  uint16_t ultrasonicDist1;
  uint16_t ultrasonicDist2;
};
