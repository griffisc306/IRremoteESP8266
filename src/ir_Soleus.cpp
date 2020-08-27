// Copyright 2019 David Conran

/// @file
/// @brief Support for Soleus protocols.
/// Analysis by crankyoldgit & AndreyShpilevoy
/// Code by crankyoldgit
/// @see https://github.com/crankyoldgit/IRremoteESP8266/issues/764
/// @see https://drive.google.com/file/d/1kjYk4zS9NQcMQhFkak-L4mp4UuaAIesW/view

#include "ir_Soleus.h"
#include <algorithm>
#include <cstring>
#include "IRrecv.h"
#include "IRsend.h"
#include "IRtext.h"
#include "IRutils.h"

// Constants
const uint16_t kSoleusHdrMark = 6112;
const uint16_t kSoleusHdrSpace = 7391;
const uint16_t kSoleusBitMark = 537;
const uint16_t kSoleusOneSpace = 1651;
const uint16_t kSoleusZeroSpace = 571;
const uint32_t kSoleusMinGap = kDefaultMessageGap;

using irutils::addBoolToString;
using irutils::addFanToString;
using irutils::addIntToString;
using irutils::addLabeledString;
using irutils::addModeToString;
using irutils::addTempToString;
using irutils::setBit;
using irutils::setBits;

#if SEND_SOLEUS
/// Send a Soleus message.
/// Status: STABLE / Known to be working.
/// @param[in] data The message to be sent.
/// @param[in] nbytes The number of bytes of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
void IRsend::sendSoleus(const unsigned char data[], const uint16_t nbytes,
                          const uint16_t repeat) {
  // Set IR carrier frequency
  enableIROut(38);

  for (uint16_t i = 0; i <= repeat; i++) {
    sendGeneric(kSoleusHdrMark, kSoleusHdrSpace,
                kSoleusBitMark, kSoleusOneSpace,
                kSoleusBitMark, kSoleusZeroSpace,
                kSoleusBitMark, kSoleusHdrSpace,
                data, nbytes, 38000, false, 0,  // Repeats are already handled.
                50);
     // Extra footer.
     mark(kSoleusBitMark);
     space(kSoleusMinGap);
  }
}
#endif  // SEND_SOLEUS

/// Class constructor
/// @param[in] pin GPIO to be used when sending.
/// @param[in] inverted Is the output signal to be inverted?
/// @param[in] use_modulation Is frequency modulation to be used?
IRSoleusAc::IRSoleusAc(const uint16_t pin, const bool inverted,
                           const bool use_modulation)
    : _irsend(pin, inverted, use_modulation) {
  this->stateReset();
}

/// Reset the state of the remote to a known good state/sequence.
void IRSoleusAc::stateReset(void) {
  static const uint8_t kReset[kSoleusStateLength] = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6A, 0x00, 0x2A, 0xA5};
  setRaw(kReset);
}

/// Set up hardware to be able to send a message.
void IRSoleusAc::begin(void) { _irsend.begin(); }

/// Calculate the checksum for a given state.
/// @param[in] state The array to calc the checksum of.
/// @param[in] length The length/size of the array.
/// @return The calculated checksum value.
uint8_t IRSoleusAc::calcChecksum(const uint8_t state[],
                                   const uint16_t length) {
  if (length == 0) return state[0];
  return sumBytes(state, length - 1);
}

/// Verify the checksum is valid for a given state.
/// @param[in] state The array to verify the checksum of.
/// @param[in] length The length/size of the array.
/// @return true, if the state has a valid checksum. Otherwise, false.
bool IRSoleusAc::validChecksum(const uint8_t state[], const uint16_t length) {
  if (length < 2)
    return true;  // No checksum to compare with. Assume okay.
  return (state[length - 1] == calcChecksum(state, length));
}

/// Calculate & update the checksum for the internal state.
/// @param[in] length The length/size of the internal state.
void IRSoleusAc::checksum(uint16_t length) {
  if (length < 2) return;
  remote_state[length - 1] = calcChecksum(remote_state, length);
}

#if SEND_SOLEUS
/// Send the current internal state as an IR message.
/// @param[in] repeat Nr. of times the message will be repeated.
void IRSoleusAc::send(const uint16_t repeat) {
  _irsend.sendSoleus(getRaw(), kSoleusStateLength, repeat);
}
#endif  // SEND_SOLEUS

/// Get a PTR to the internal state/code for this protocol.
/// @return PTR to a code for this protocol based on the current internal state.
uint8_t *IRSoleusAc::getRaw(void) {
  this->checksum();
  return remote_state;
}

/// Set the internal state from a valid code for this protocol.
/// @param[in] new_code A valid code for this protocol.
/// @param[in] length The length/size of the new_code array.
void IRSoleusAc::setRaw(const uint8_t new_code[], const uint16_t length) {
  memcpy(remote_state, new_code, std::min(length, kSoleusStateLength));
}

/// Set the Button/Command pressed setting of the A/C.
/// @param[in] button The value of the button/command that was pressed.
void IRSoleusAc::setButton(const uint8_t button) {
  switch (button) {
    case kSoleusButtonPower:
    case kSoleusButtonMode:
    case kSoleusButtonTempUp:
    case kSoleusButtonTempDown:
    case kSoleusButtonSwing:
    case kSoleusButtonFanSpeed:
    case kSoleusButtonAirFlow:
    case kSoleusButtonHold:
    case kSoleusButtonSleep:
    case kSoleusButtonLight:
    case kSoleusButtonEye:
    case kSoleusButtonFollow:
    case kSoleusButtonIon:
    case kSoleusButtonFresh:
    case kSoleusButton8CHeat:
    case kSoleusButtonTurbo:
      setBits(&remote_state[5], kSoleusButtonOffset, kSoleusButtonSize,
              button);
      break;
    default:
      this->setButton(kSoleusButtonPower);
  }
}

/// Get the Button/Command setting of the A/C.
/// @return The value of the button/command that was pressed.
uint8_t IRSoleusAc::getButton(void) {
  return GETBITS8(remote_state[5], kSoleusButtonOffset, kSoleusButtonSize);
}

/// Set the requested power state of the A/C to on.
void IRSoleusAc::on(void) { this->setPower(true); }

/// Set the requested power state of the A/C to off.
void IRSoleusAc::off(void) { this->setPower(false); }

/// Change the power setting.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRSoleusAc::setPower(const bool on) {
  this->setButton(kSoleusButtonPower);
  setBit(&remote_state[7], kSoleusPowerOffset, on);
}

/// Get the value of the current power setting.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getPower(void) {
  return GETBIT8(remote_state[7], kSoleusPowerOffset);
}

/// Set the operating mode of the A/C.
/// @param[in] mode The desired operating mode.
void IRSoleusAc::setMode(const uint8_t mode) {
  switch (mode) {
    case kSoleusDry:
      // In this mode fan speed always LOW
      this->setFan(kSoleusFanLow);
      // FALL THRU
    case kSoleusAuto:
    case kSoleusCool:
    case kSoleusFan:
    case kSoleusHeat:
      setBits(&remote_state[9], kSoleusModeOffset, kModeBitsSize, mode);
      this->setButton(kSoleusButtonMode);
      break;
    default:
      // If we get an unexpected mode, default to AUTO.
      this->setMode(kSoleusAuto);
  }
}

/// Get the operating mode setting of the A/C.
/// @return The current operating mode setting.
uint8_t IRSoleusAc::getMode(void) {
  return GETBITS8(remote_state[9], kSoleusModeOffset, kModeBitsSize);
}

/// Convert a stdAc::opmode_t enum into its native mode.
/// @param[in] mode The enum to be converted.
/// @return The native equivilant of the enum.
uint8_t IRSoleusAc::convertMode(const stdAc::opmode_t mode) {
  switch (mode) {
    case stdAc::opmode_t::kCool: return kSoleusCool;
    case stdAc::opmode_t::kHeat: return kSoleusHeat;
    case stdAc::opmode_t::kDry:  return kSoleusDry;
    case stdAc::opmode_t::kFan:  return kSoleusFan;
    default:                     return kSoleusAuto;
  }
}

/// Convert a native mode into its stdAc equivilant.
/// @param[in] mode The native setting to be converted.
/// @return The stdAc equivilant of the native setting.
stdAc::opmode_t IRSoleusAc::toCommonMode(const uint8_t mode) {
  switch (mode) {
    case kSoleusCool: return stdAc::opmode_t::kCool;
    case kSoleusHeat: return stdAc::opmode_t::kHeat;
    case kSoleusDry:  return stdAc::opmode_t::kDry;
    case kSoleusFan:  return stdAc::opmode_t::kFan;
    default:            return stdAc::opmode_t::kAuto;
  }
}

/// Set the temperature.
/// @param[in] temp The temperature in degrees celsius.
void IRSoleusAc::setTemp(const uint8_t temp) {
  uint8_t oldtemp = this->getTemp();
  uint8_t newtemp = std::max(kSoleusMinTemp, temp);
  newtemp = std::min(kSoleusMaxTemp, newtemp);
  if (oldtemp > newtemp)
    this->setButton(kSoleusButtonTempDown);
  else if (newtemp > oldtemp)
    this->setButton(kSoleusButtonTempUp);
  setBits(&remote_state[9], kSoleusTempOffset, kSoleusTempSize,
          newtemp - kSoleusMinTemp);
}

/// Get the current temperature setting.
/// @return The current setting for temp. in degrees celsius.
uint8_t IRSoleusAc::getTemp(void) {
  return GETBITS8(remote_state[9], kSoleusTempOffset, kSoleusTempSize) +
      kSoleusMinTemp;
}

/// Set the speed of the fan.
/// @param[in] speed The desired setting. 0-3, 0 is auto, 1-3 is the speed
void IRSoleusAc::setFan(const uint8_t speed) {
  switch (speed) {
    case kSoleusFanAuto:
    case kSoleusFanHigh:
    case kSoleusFanMed:
      if (this->getMode() == kSoleusDry) {  // Dry mode only allows low speed.
        this->setFan(kSoleusFanLow);
        return;
      }
      // FALL-THRU
    case kSoleusFanLow:
      setBits(&remote_state[7], kSoleusFanOffest, kSoleusFanSize, speed);
      this->setButton(kSoleusButtonFanSpeed);
      break;
    default:
      // If we get an unexpected speed, default to Auto.
      this->setFan(kSoleusFanAuto);
  }
}

/// Get the current fan speed setting.
/// @return The current fan speed/mode.
uint8_t IRSoleusAc::getFan(void) {
  return GETBITS8(remote_state[7], kSoleusFanOffest, kSoleusFanSize);
}

/// Convert a stdAc::fanspeed_t enum into it's native speed.
/// @param[in] speed The enum to be converted.
/// @return The native equivilant of the enum.
uint8_t IRSoleusAc::convertFan(const stdAc::fanspeed_t speed) {
  switch (speed) {
    case stdAc::fanspeed_t::kMin:
    case stdAc::fanspeed_t::kLow:    return kSoleusFanLow;
    case stdAc::fanspeed_t::kMedium: return kSoleusFanMed;
    case stdAc::fanspeed_t::kHigh:
    case stdAc::fanspeed_t::kMax:    return kSoleusFanHigh;
    default:                         return kSoleusFanAuto;
  }
}

/// Convert a native fan speed into its stdAc equivilant.
/// @param[in] speed The native setting to be converted.
/// @return The stdAc equivilant of the native setting.
stdAc::fanspeed_t IRSoleusAc::toCommonFanSpeed(const uint8_t speed) {
  switch (speed) {
    case kSoleusFanHigh: return stdAc::fanspeed_t::kMax;
    case kSoleusFanMed:  return stdAc::fanspeed_t::kMedium;
    case kSoleusFanLow:  return stdAc::fanspeed_t::kMin;
    default:               return stdAc::fanspeed_t::kAuto;
  }
}

/// Set the Sleep setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRSoleusAc::setSleep(const bool on) {
  this->setButton(kSoleusButtonSleep);
  setBit(&remote_state[7], kSoleusSleepOffset, on);
}

/// Get the Sleep setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getSleep(void) {
  return GETBIT8(remote_state[7],  kSoleusSleepOffset);
}

/// Set the vertical swing setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRSoleusAc::setSwingV(const bool on) {
  this->setButton(kSoleusButtonSwing);
  setBits(&remote_state[7], kSoleusSwingVOffset, kSoleusSwingVSize,
          on ? kSoleusSwingVOn : kSoleusSwingVOff);
}

/// Get the vertical swing setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getSwingV(void) {
  return GETBITS8(remote_state[7], kSoleusSwingVOffset,
                  kSoleusSwingVSize) == kSoleusSwingVOn;
}

/// Set the horizontal swing setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRSoleusAc::setSwingH(const bool on) {
  this->setButton(kSoleusButtonAirFlow);
  setBit(&remote_state[7], kSoleusSwingHOffset, !on);  // Cleared when `on`
}

/// Get the horizontal swing (Air Flow) setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getSwingH(void) {
  return !GETBIT8(remote_state[7], kSoleusSwingHOffset);
}

/// Set the Turbo setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRSoleusAc::setTurbo(const bool on) {
  this->setButton(kSoleusButtonTurbo);
  setBit(&remote_state[3], kSoleusTurboOffset, on);
}

/// Get the Turbo setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getTurbo(void) {
  return GETBIT8(remote_state[3], kSoleusTurboOffset);
}

/// Set the Fresh (air) setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRSoleusAc::setFresh(const bool on) {
  this->setButton(kSoleusButtonFresh);
  setBit(&remote_state[5], kSoleusFreshOffset, on);
}

/// Get the Frsh (air) setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getFresh(void) {
  return GETBIT8(remote_state[5], kSoleusFreshOffset);
}

/// Set the Hold setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRSoleusAc::setHold(const bool on) {
  this->setButton(kSoleusButtonHold);
  setBit(&remote_state[3], kSoleusHoldOffset, on);
}

/// Get the Hold setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getHold(void) {
  return GETBIT8(remote_state[3], kSoleusHoldOffset);
}

/// Set the Ion (filter) setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRSoleusAc::setIon(const bool on) {
  this->setButton(kSoleusButtonIon);
  setBit(&remote_state[1], kSoleusIonOffset, on);
}

/// Get the Ion (filter) setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getIon(void) {
  return GETBIT8(remote_state[1], kSoleusIonOffset);
}

/// Set the Light(LED display) setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRSoleusAc::setLight(const bool on) {
  this->setButton(kSoleusButtonLight);
  setBit(&remote_state[3], kSoleusLightOffset, on);
}

/// Get the Light (LED display) setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getLight(void) {
  return GETBIT8(remote_state[3], kSoleusLightOffset);
}

/// Set the 8°C Heat setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
/// @note This feature maintains the room temperature steadily at 8°C and
///   prevents the room from freezing by activating the heating operation
///   automatically when nobody is at home over a longer period during severe
///   winter.
void IRSoleusAc::set8CHeat(const bool on) {
  this->setButton(kSoleusButton8CHeat);
  setBit(&remote_state[1], kSoleus8CHeatOffset, on);
}

/// Get the 8°C Heat setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::get8CHeat(void) {
  return GETBIT8(remote_state[1], kSoleus8CHeatOffset);
}

/// Set the Eye (Sensor) setting of the A/C.
/// @param[in] on true, the setting is on. false, the setting is off.
void IRSoleusAc::setEye(const bool on) {
  this->setButton(kSoleusButtonEye);
  setBit(&remote_state[3], kSoleusEyeOffset, on);
}

/// Get the Eye (Sensor) setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getEye(void) {
  return GETBIT8(remote_state[3], kSoleusEyeOffset);
}

/* DISABLED
   TODO(someone): Work out why "on" is either 0x5D or 0x5F
void IRSoleusAc::setFollow(const bool on) {
  this->setButton(kSoleusButtonFollow);
  if (on)
    remote_state[8] = kSoleusFollowMe;
  else
    remote_state[8] = 0;
}
*/

/// Get the Follow Me setting of the A/C.
/// @return true, the setting is on. false, the setting is off.
bool IRSoleusAc::getFollow(void) {
  return (remote_state[8] & kSoleusFollowMe) == kSoleusFollowMe;
}

/// Convert the current internal state into its stdAc::state_t equivilant.
/// @return The stdAc equivilant of the native settings.
stdAc::state_t IRSoleusAc::toCommon(void) {
  stdAc::state_t result;
  result.protocol = decode_type_t::SOLEUS;
  result.model = -1;  // No models used.
  result.power = this->getPower();
  result.mode = this->toCommonMode(this->getMode());
  result.celsius = true;
  result.degrees = this->getTemp();
  result.fanspeed = this->toCommonFanSpeed(this->getFan());
  result.swingv = this->getSwingV() ? stdAc::swingv_t::kAuto
                                    : stdAc::swingv_t::kOff;
  result.swingh = this->getSwingH() ? stdAc::swingh_t::kAuto
                                    : stdAc::swingh_t::kOff;
  result.turbo = this->getTurbo();
  result.light = this->getLight();
  result.filter = this->getIon();
  result.sleep = this->getSleep() ? 0 : -1;
  // Not supported.
  result.quiet = false;
  result.econo = false;
  result.clean = false;
  result.beep = false;
  result.clock = -1;
  return result;
}

/// Convert the current internal state into a human readable string.
/// @return A human readable string.
String IRSoleusAc::toString(void) {
  String result = "";
  result.reserve(100);  // Reserve some heap for the string to reduce fragging.
  result += addBoolToString(getPower(), kPowerStr, false);
  result += addModeToString(getMode(), kSoleusAuto, kSoleusCool,
                            kSoleusHeat, kSoleusDry, kSoleusFan);
  result += addTempToString(getTemp());
  result += addFanToString(getFan(), kSoleusFanHigh, kSoleusFanLow,
                           kSoleusFanAuto, kSoleusFanAuto, kSoleusFanMed);
  result += addBoolToString(getSwingV(), kSwingVStr);
  result += addBoolToString(getSwingH(), kSwingHStr);
  result += addBoolToString(getSleep(), kSleepStr);
  result += addBoolToString(getTurbo(), kTurboStr);
  result += addBoolToString(getHold(), kHoldStr);
  result += addBoolToString(getIon(), kIonStr);
  result += addBoolToString(getEye(), kEyeStr);
  result += addBoolToString(getLight(), kLightStr);
  result += addBoolToString(getFollow(), kFollowStr);
  result += addBoolToString(get8CHeat(), k8CHeatStr);
  result += addBoolToString(getFresh(), kFreshStr);
  result += addIntToString(getButton(), kButtonStr);
  result += kSpaceLBraceStr;
  switch (this->getButton()) {
    case kSoleusButtonPower:    result += kPowerStr; break;
    case kSoleusButtonMode:     result += kModeStr; break;
    case kSoleusButtonTempUp:   result += kTempUpStr; break;
    case kSoleusButtonTempDown: result += kTempDownStr; break;
    case kSoleusButtonSwing:    result += kSwingStr; break;
    case kSoleusButtonFanSpeed: result += kFanStr; break;
    case kSoleusButtonAirFlow:  result += kAirFlowStr; break;
    case kSoleusButtonHold:     result += kHoldStr; break;
    case kSoleusButtonSleep:    result += kSleepStr; break;
    case kSoleusButtonLight:    result += kLightStr; break;
    case kSoleusButtonEye:      result += kEyeStr; break;
    case kSoleusButtonFollow:   result += kFollowStr; break;
    case kSoleusButtonIon:      result += kIonStr; break;
    case kSoleusButtonFresh:    result += kFreshStr; break;
    case kSoleusButton8CHeat:   result += k8CHeatStr; break;
    case kSoleusButtonTurbo:    result += kTurboStr; break;
    default:
      result += kUnknownStr;
  }
  result += ')';
  return result;
}

#if DECODE_SOLEUS
/// Decode the supplied Soleus message.
/// Status: STABLE / Known working
/// @param[in,out] results Ptr to the data to decode & where to store the result
/// @param[in] offset The starting index to use when attempting to decode the
///   raw data. Typically/Defaults to kStartOffset.
/// @param[in] nbits The number of data bits to expect.
/// @param[in] strict Flag indicating if we should perform strict matching.
/// @return True if it can decode it, false if it can't.
bool IRrecv::decodeSoleus(decode_results *results, uint16_t offset,
                            const uint16_t nbits, const bool strict) {
  // Compliance
  if (strict && nbits != kSoleusBits)
    return false;  // Incorrect nr. of bits per spec.

  // Match Main Header + Data + Footer
  uint16_t used;
  used = matchGeneric(results->rawbuf + offset, results->state,
                      results->rawlen - offset, nbits,
                      kSoleusHdrMark, kSoleusHdrSpace,
                      kSoleusBitMark, kSoleusOneSpace,
                      kSoleusBitMark, kSoleusZeroSpace,
                      kSoleusBitMark, kSoleusHdrSpace, false,
                      _tolerance, 0, false);
  if (!used) return false;
  offset += used;
  // Extra footer.
  uint64_t unused;
  if (!matchGeneric(results->rawbuf + offset, &unused,
                    results->rawlen - offset, 0, 0, 0, 0, 0, 0, 0,
                    kSoleusBitMark, kSoleusHdrSpace, true)) return false;

  // Compliance
  if (strict) {
    // Check we got a valid checksum.
    if (!IRSoleusAc::validChecksum(results->state, nbits / 8)) return false;
  }

  // Success
  results->decode_type = decode_type_t::SOLEUS;
  results->bits = nbits;
  // No need to record the state as we stored it as we decoded it.
  // As we use result->state, we don't record value, address, or command as it
  // is a union data type.
  return true;
}
#endif  // DECODE_SOLEUS
