// Copyright 2019 David Conran

/// @file
/// @brief Support for Soleus protocols.
/// Analysis by crankyoldgit & AndreyShpilevoy
/// @see https://github.com/crankyoldgit/IRremoteESP8266/issues/764
/// @see https://drive.google.com/file/d/1kjYk4zS9NQcMQhFkak-L4mp4UuaAIesW/view

// Supports:
//   Brand: Soleus,  Model: NS-09AHTI A/C
//   Brand: Soleus,  Model: ZH/TY-01 remote

#ifndef IR_SOLEUS_H_
#define IR_SOLEUS_H_

#define __STDC_LIMIT_MACROS
#include <stdint.h>
#ifndef UNIT_TEST
#include <Arduino.h>
#endif
#include "IRremoteESP8266.h"
#include "IRsend.h"
#ifdef UNIT_TEST
#include "IRsend_test.h"
#endif

// Constants
// state[1]
const uint8_t kSoleus8CHeatOffset = 1;
const uint8_t kSoleusIonOffset = 2;
// state[3]
const uint8_t kSoleusLightOffset = 0;
const uint8_t kSoleusHoldOffset = 2;
const uint8_t kSoleusTurboOffset = 3;
const uint8_t kSoleusEyeOffset = 6;
// state[5]
const uint8_t kSoleusFreshOffset = 7;
const uint8_t kSoleusButtonOffset = 0;
const uint8_t kSoleusButtonSize = 5;
const uint8_t kSoleusButtonPower =    0x00;
const uint8_t kSoleusButtonMode =     0x01;
const uint8_t kSoleusButtonTempUp =   0x02;
const uint8_t kSoleusButtonTempDown = 0x03;
const uint8_t kSoleusButtonSwing =    0x04;
const uint8_t kSoleusButtonFanSpeed = 0x05;
const uint8_t kSoleusButtonAirFlow =  0x07;
const uint8_t kSoleusButtonHold =     0x08;
const uint8_t kSoleusButtonSleep =    0x09;
const uint8_t kSoleusButtonTurbo =    0x0A;
const uint8_t kSoleusButtonLight =    0x0B;
const uint8_t kSoleusButtonEye =      0x0E;
const uint8_t kSoleusButtonFollow =   0x13;
const uint8_t kSoleusButtonIon =      0x14;
const uint8_t kSoleusButtonFresh =    0x15;
const uint8_t kSoleusButton8CHeat =   0x1D;
// state[7]
const uint8_t kSoleusSleepOffset = 0;
const uint8_t kSoleusPowerOffset = 1;
const uint8_t kSoleusSwingVOffset = 2;
const uint8_t kSoleusSwingVSize = 2;  // Bits
const uint8_t kSoleusSwingVOn =   0b01;
const uint8_t kSoleusSwingVOff =  0b10;
const uint8_t kSoleusSwingHOffset = 4;
const uint8_t kSoleusFanOffest = 5;
const uint8_t kSoleusFanSize = 2;
const uint8_t kSoleusFanAuto =     0b00;
const uint8_t kSoleusFanHigh =     0b01;
const uint8_t kSoleusFanMed =      0b10;
const uint8_t kSoleusFanLow =      0b11;
// state[8]
const uint8_t kSoleusFollowMe = 0x5D;  // Also 0x5F
// state[9]
const uint8_t kSoleusTempOffset = 0;
const uint8_t kSoleusTempSize = 5;  // Bits
const uint8_t kSoleusMinTemp = 16;   // 16C
const uint8_t kSoleusMaxTemp = 32;   // 32C
const uint8_t kSoleusModeOffset = 5;
const uint8_t kSoleusAuto =     0b000;
const uint8_t kSoleusCool =     0b001;
const uint8_t kSoleusDry =      0b010;
const uint8_t kSoleusFan =      0b011;
const uint8_t kSoleusHeat =     0b100;

// Classes
/// Class for handling detailed Soleus A/C messages.
class IRSoleusAc {
 public:
  explicit IRSoleusAc(const uint16_t pin, const bool inverted = false,
                        const bool use_modulation = true);
  void stateReset(void);
#if SEND_SOLEUS
  void send(const uint16_t repeat = kSoleusMinRepeat);
  /// Run the calibration to calculate uSec timing offsets for this platform.
  /// @return The uSec timing offset needed per modulation of the IR Led.
  /// @note This will produce a 65ms IR signal pulse at 38kHz.
  ///   Only ever needs to be run once per object instantiation, if at all.
  int8_t calibrate(void) { return _irsend.calibrate(); }
#endif  // SEND_SOLEUS
  void begin(void);
  void setButton(const uint8_t button);
  uint8_t getButton(void);
  void on(void);
  void off(void);
  void setPower(const bool on);
  bool getPower(void);
  void setMode(const uint8_t mode);
  uint8_t getMode(void);
  void setTemp(const uint8_t temp);
  uint8_t getTemp(void);
  void setFan(const uint8_t speed);
  uint8_t getFan(void);
  void setSwingV(const bool on);
  bool getSwingV(void);
  void setSwingH(const bool on);
  bool getSwingH(void);
  void setSleep(const bool on);
  bool getSleep(void);
  void setTurbo(const bool on);
  bool getTurbo(void);
  void setFresh(const bool on);
  bool getFresh(void);
  void setHold(const bool on);
  bool getHold(void);
  void setIon(const bool on);
  bool getIon(void);
  void setLight(const bool on);
  bool getLight(void);
  void set8CHeat(const bool on);
  bool get8CHeat(void);
  void setEye(const bool on);
  bool getEye(void);
  // DISABLED: See TODO in ir_Soleus.cpp
  // void setFollow(const bool on);
  bool getFollow(void);
  uint8_t* getRaw(void);
  void setRaw(const uint8_t new_code[],
              const uint16_t length = kSoleusStateLength);
  static bool validChecksum(const uint8_t state[],
                            const uint16_t length = kSoleusStateLength);
  static uint8_t calcChecksum(const uint8_t state[],
                              const uint16_t length = kSoleusStateLength);
  String toString(void);
  uint8_t convertMode(const stdAc::opmode_t mode);
  uint8_t convertFan(const stdAc::fanspeed_t speed);
  static stdAc::opmode_t toCommonMode(const uint8_t mode);
  static stdAc::fanspeed_t toCommonFanSpeed(const uint8_t speed);
  stdAc::state_t toCommon(void);
#ifndef UNIT_TEST

 private:
  IRsend _irsend;  ///< Instance of the IR send class
#else  // UNIT_TEST
  /// @cond IGNORE
  IRsendTest _irsend;  ///< Instance of the testing IR send class
  /// @endcond
#endif  // UNIT_TEST
  uint8_t remote_state[kSoleusStateLength];  ///< State of the remote in code.
  void checksum(const uint16_t length = kSoleusStateLength);
};

#endif  // IR_SOLEUS_H_
