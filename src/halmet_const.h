#ifndef HALMET_SRC_HALMET_CONST_H_
#define HALMET_SRC_HALMET_CONST_H_

#include <Arduino.h>

namespace sensesp {

const int kSDAPin = 21;
const int kSCLPin = 22;

const int kADS1115Address_0 = 0x4b;
const int kADS1115Address_1 = 0x48;
const int kADS1115Address_2 = 0x49;

const int kBNO055Address = 0x28;

// CAN bus (NMEA 2000) pins on HALMET
const gpio_num_t kCANRxPin = GPIO_NUM_18;
const gpio_num_t kCANTxPin = GPIO_NUM_19;

// HALMET digital input pins
const int kDigitalInputPin1 = GPIO_NUM_23;
const int kDigitalInputPin2 = GPIO_NUM_25;
const int kDigitalInputPin3 = GPIO_NUM_27;
const int kDigitalInputPin4 = GPIO_NUM_26;

// ESP32 Serial2 pins (AIS gateway) - default ESP32 pins
const int kSerial2RxPin = 16;
const int kSerial2TxPin = 17;

// Test output pin for debugging
const int kTestOutputPin = GPIO_NUM_33;
const int kTestOutputFrequency = 380;

// ADS1115 configuration
// Available gain values:
//   GAIN_TWOTHIRDS = ±6.144V range
//   GAIN_ONE       = ±4.096V range (default)
//   GAIN_TWO       = ±2.048V range  
//   GAIN_FOUR      = ±1.024V range
//   GAIN_EIGHT     = ±0.512V range
//   GAIN_SIXTEEN   = ±0.256V range
// Higher gain = better resolution but smaller voltage range
const adsGain_t kADS1115Gain = GAIN_ONE;

}  // namespace sensesp

#endif /* HALMET_SRC_HALMET_CONST_H_ */
