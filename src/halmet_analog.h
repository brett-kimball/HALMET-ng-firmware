#ifndef HALMET_ANALOG_H_
#define HALMET_ANALOG_H_

#include <Adafruit_ADS1X15.h>
#include <map>
#include <string>

#include "sensesp/sensors/sensor.h"
#include "sensesp/system/valueconsumer.h"
#include "sensesp/ui/status_page_item.h"
#include "sensesp_base_app.h"

namespace halmet {

// ========================================================================
// GLOBAL CONFIG
// ========================================================================
extern bool g_enable_calibration;

// ========================================================================
// EXTERNAL DECLARATIONS FOR GLOBAL MAPS
// ========================================================================
extern std::map<std::string, float> raw_sensor_values;

// ========================================================================
// CALIBRATION STATUS PAGE ITEM
// ========================================================================
// Custom StatusPageItem that only shows in web UI when calibration is enabled
template <typename T>
class CalibrationStatusPageItem : public sensesp::StatusPageItem<T> {
 public:
  CalibrationStatusPageItem(String name, const T& value, String group, int order)
      : sensesp::StatusPageItem<T>(name, value, group, order) {}

 protected:
  virtual JsonDocument as_json() override {
    // Only return JSON data if calibration mode is enabled
    if (halmet::g_enable_calibration) {
      JsonDocument obj;
      obj["name"] = this->name_;
      obj["value"] = this->get();
      obj["group"] = this->group_;
      obj["order"] = this->order_;
      return obj;
    } else {
      // Return empty JSON when calibration is disabled
      return JsonDocument();
    }
  }
};

// ========================================================================
// EXTERNAL DECLARATIONS FOR GLOBAL MAPS (continued)
// ========================================================================
extern std::map<std::string, CalibrationStatusPageItem<float>*> raw_sensor_status_items;

// ========================================================================
// ANALOG SENSOR TYPES
// ========================================================================
enum AnalogSensorType {
    TEMPERATURE,
    PRESSURE,
    FUEL_LEVEL,
    WATER_LEVEL,
    BLACK_WATER_LEVEL,
    GRAY_WATER_LEVEL,
    BATTERY_VOLTAGE,
    EXHAUST_TEMPERATURE,
    BILGE_LEVEL,
    RUDDER_ANGLE,
    TRIM_ANGLE,
    TRANSMISSION_GEAR,
    THROTTLE_POSITION,
    GENERIC_VOLTAGE,
    GENERIC_RESISTANCE,
    GENERIC_CURRENT,
    GENERIC_TEMPERATURE,
    GENERIC_PRESSURE
};

// ========================================================================
// HALMET ANALOG INPUT HELPERS
// ========================================================================

// --------------------------------------------------------------------
// VOLTAGE DIVIDER SCALE FACTOR
// --------------------------------------------------------------------
// HALMET uses a 33.3kΩ / 3.3kΩ divider → input voltage = ADC * 11.0
// This factor converts post-divider voltage back to actual sender voltage.
const float kVoltageDividerScale = 33.3 / 3.3;

// --------------------------------------------------------------------
// SENSOR CONNECTION FUNCTIONS
// --------------------------------------------------------------------
// These functions create complete signal chains:
//   ADS1115 → Voltage → Curve → Signal K + Web UI
//
// Each returns a FloatProducer (the final calibrated value).

sensesp::FloatProducer* ConnectTankSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output = true
);

sensesp::FloatProducer* ConnectTemperatureSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output = true
);

sensesp::FloatProducer* ConnectPressureSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output = true
);

sensesp::FloatProducer* ConnectRudderAngleSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output = true
);

sensesp::FloatProducer* ConnectTrimAngleSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output = true
);

sensesp::FloatProducer* ConnectTransmissionGearSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output = true
);

// ========================================================================
// UNIFIED ANALOG SENDER FUNCTION
// ========================================================================
sensesp::FloatProducer* ConnectAnalogSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    AnalogSensorType type,
    const String& instance,
    const String& hardware_id,
    int sort_order,
    bool enable_signalk_output = true,
    bool enable_raw_sk = true,
    float offset = 0.0,
    float multiplier = 1.0
);

// ========================================================================
// ADS1115 VOLTAGE INPUT SENSOR
// ========================================================================

/**
 * @brief Low-level ADS1115 voltage sensor with calibration
 *
 * Reads a single-ended channel, applies voltage divider scale,
 * and supports per-channel calibration via web UI.
 */
class ADS1115VoltageInput : public sensesp::FloatSensor {
 public:
  ADS1115VoltageInput(
      Adafruit_ADS1115* ads1115,
      int channel,
      const String& config_path,
      unsigned int read_interval = 500,
      float calibration_factor = 1.0
  )
      : sensesp::FloatSensor(config_path),
        ads1115_{ads1115},
        channel_{channel},
        read_interval_{read_interval},
        calibration_factor_{calibration_factor}
  {
    load();
    repeat_event_ = set_repeat_event(read_interval_);
  }

  // --------------------------------------------------------------------
  // SENSOR UPDATE
  // --------------------------------------------------------------------
  void update() {
    int16_t adc_output = ads1115_->readADC_SingleEnded(channel_);
    float adc_output_volts = ads1115_->computeVolts(adc_output);
    float scaled_voltage = calibration_factor_ * kVoltageDividerScale * adc_output_volts;
    this->emit(scaled_voltage);
  }

  // --------------------------------------------------------------------
  // CONFIGURATION PERSISTENCE
  // --------------------------------------------------------------------
  virtual bool to_json(JsonObject& root) override {
    root["calibration_factor"] = calibration_factor_;
    return true;
  }

  virtual bool from_json(const JsonObject& config) override {
    if (config["calibration_factor"].is<float>()) {
      calibration_factor_ = config["calibration_factor"];
      return true;
    }
    return false;
  }

 protected:
  reactesp::RepeatEvent* repeat_event_ = nullptr;

  // --------------------------------------------------------------------
  // REPEAT EVENT MANAGEMENT
  // --------------------------------------------------------------------
  reactesp::RepeatEvent* set_repeat_event(unsigned int read_interval) {
    if (repeat_event_ != nullptr) {
      repeat_event_->remove(sensesp::event_loop());
    }

    repeat_event_ = sensesp::event_loop()->onRepeat(
        read_interval,
        [this]() { this->update(); }
    );
    return repeat_event_;
  }

 private:
  Adafruit_ADS1115* ads1115_;
  int channel_;
  unsigned int read_interval_;
  float calibration_factor_;
};

// --------------------------------------------------------------------
// CONFIG SCHEMA
// --------------------------------------------------------------------
inline const String ConfigSchema(const ADS1115VoltageInput& obj) {
  const char SCHEMA[] = R"###({
    "type": "object",
    "properties": {
      "calibration_factor": {
        "title": "Calibration factor",
        "type": "number",
        "description": "Multiplier to apply to the raw input value"
      }
    }
  })###";

  return SCHEMA;
}

// --------------------------------------------------------------------
// RESTART REQUIRED ON CONFIG CHANGE
// --------------------------------------------------------------------
inline const bool ConfigRequiresRestart(const ADS1115VoltageInput& obj) {
  return true;
}

class RawValueConsumer : public sensesp::ValueConsumer<float> {
 public:
  RawValueConsumer(const char* id) : id_(id) {}
  void set_input(float input, uint8_t input_channel = 0) {
    // Update global raw_sensor_values map
    raw_sensor_values[id_] = input;
    // Update status page item if it exists and calibration is enabled
    if (g_enable_calibration) {
      auto it = raw_sensor_status_items.find(id_);
      if (it != raw_sensor_status_items.end()) {
        debugD("Updating StatusPageItem for sensor %s with value %f", id_.c_str(), input);
        it->second->set(input);
      } else {
        debugW("StatusPageItem not found for sensor %s", id_.c_str());
      }
    }
  }
 private:
  std::string id_;
};

}  // namespace halmet

#endif  // HALMET_ANALOG_H_