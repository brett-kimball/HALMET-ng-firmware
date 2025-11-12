// halmet_analog.cpp — FINAL WITH OIL & TEMP CURVES IN WEB UI
#include "halmet_analog.h"
#include <N2kMessages.h>
#include <map>
#include <string>
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"
extern std::map<std::string, float> raw_sensor_values;

// Status page items for displaying raw sensor values on web UI
extern std::map<std::string, halmet::CalibrationStatusPageItem<float>*> raw_sensor_status_items;

// Raw sensor value producers for calibration mode
// extern std::map<std::string, sensesp::ObservableValue<float>*> raw_sensor_producers;

namespace halmet {

const float kMeasurementCurrent = 0.01;

// ========================================================================
// GLOBAL CONFIG
// ========================================================================
bool g_enable_calibration = true;
bool g_single_trim_sensor = true;

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
    bool enable_signalk_output,
    bool enable_raw_sk,
    float offset,
    float multiplier
) {
  const uint read_delay = 500;
  bool is_active = false;  // passive by default
  String measurement_type = "voltage";
  String output_unit = "";
  String sk_path = "";
  String raw_sk_path = "";
  String curve_title = "";
  String curve_description = "";
  String config_path = "";
  sensesp::CurveInterpolator::Sample default_samples[3];

  // Set up based on type
  switch (type) {
    case TEMPERATURE:
      is_active = false;
      measurement_type = "voltage";
      output_unit = "°F";
      sk_path = "propulsion." + instance + ".coolantTemperature";
      raw_sk_path = "sensors." + hardware_id + ".voltage";
      curve_title = instance + " Temperature Curve (°F)";
      curve_description = "Map voltage to °F";
      config_path = "/Temp/" + instance + "/Fahrenheit Curve";
      default_samples[0] = {0.5, 77.0};
      default_samples[1] = {2.0, 140.0};
      default_samples[2] = {3.5, 194.0};
      break;
    case PRESSURE:
      is_active = false;
      measurement_type = "voltage";
      output_unit = "PSI";
      sk_path = "propulsion." + instance + ".oilPressure";
      raw_sk_path = "sensors." + hardware_id + ".voltage";
      curve_title = instance + " Oil Pressure Curve";
      curve_description = "Map voltage to PSI";
      config_path = "/Pressure/" + instance + "/PSI Curve";
      default_samples[0] = {0.5, 0.0};
      default_samples[1] = {2.5, 50.0};
      default_samples[2] = {4.5, 100.0};
      break;
    case FUEL_LEVEL:
      is_active = true;  // assuming resistive, may need excitation
      measurement_type = "resistance";
      output_unit = "%";
      sk_path = "tanks." + instance + ".fuel.currentLevel";
      raw_sk_path = "sensors." + hardware_id + ".resistance";
      curve_title = instance + " Fuel Level Curve";
      curve_description = "Map resistance to %";
      config_path = "/Fuel/" + instance + "/Level Curve";
      default_samples[0] = {0.0, 100.0};
      default_samples[1] = {50.0, 50.0};
      default_samples[2] = {100.0, 0.0};
      break;
    case WATER_LEVEL:
      is_active = true;  // assuming resistive, may need excitation
      measurement_type = "resistance";
      output_unit = "%";
      sk_path = "tanks." + instance + ".water.currentLevel";
      raw_sk_path = "sensors." + hardware_id + ".resistance";
      curve_title = instance + " Water Level Curve";
      curve_description = "Map resistance to %";
      config_path = "/Water/" + instance + "/Level Curve";
      default_samples[0] = {0.0, 100.0};
      default_samples[1] = {50.0, 50.0};
      default_samples[2] = {100.0, 0.0};
      break;
    case BLACK_WATER_LEVEL:
      is_active = true;  // assuming resistive, may need excitation
      measurement_type = "resistance";
      output_unit = "%";
      sk_path = "tanks." + instance + ".blackWater.currentLevel";
      raw_sk_path = "sensors." + hardware_id + ".resistance";
      curve_title = instance + " Black Water Level Curve";
      curve_description = "Map resistance to %";
      config_path = "/BlackWater/" + instance + "/Level Curve";
      default_samples[0] = {0.0, 100.0};
      default_samples[1] = {50.0, 50.0};
      default_samples[2] = {100.0, 0.0};
      break;
    case GRAY_WATER_LEVEL:
      is_active = true;  // assuming resistive, may need excitation
      measurement_type = "resistance";
      output_unit = "%";
      sk_path = "tanks." + instance + ".grayWater.currentLevel";
      raw_sk_path = "sensors." + hardware_id + ".resistance";
      curve_title = instance + " Gray Water Level Curve";
      curve_description = "Map resistance to %";
      config_path = "/GrayWater/" + instance + "/Level Curve";
      default_samples[0] = {0.0, 100.0};
      default_samples[1] = {50.0, 50.0};
      default_samples[2] = {100.0, 0.0};
      break;
    case RUDDER_ANGLE:
      is_active = true;
      measurement_type = "resistance";
      output_unit = "°";
      sk_path = "steering.rudderAngle";
      raw_sk_path = "sensors." + hardware_id + ".resistance";
      curve_title = instance + " Rudder Angle Curve";
      curve_description = "Map resistance to angle";
      config_path = "/Rudder/" + instance + "/Angle Curve";
      default_samples[0] = {0.0, -45.0};
      default_samples[1] = {95.0, 0.0};
      default_samples[2] = {190.0, 45.0};
      break;
    case TRIM_ANGLE:
      is_active = true;
      measurement_type = "resistance";
      output_unit = "°";
      sk_path = "steering.trimTab." + instance;
      raw_sk_path = "sensors." + hardware_id + ".resistance";
      curve_title = instance + " Trim Angle Curve";
      curve_description = "Map resistance to trim angle";
      config_path = "/Trim/" + instance + "/Angle Curve";
      default_samples[0] = {0.0, -10.0};
      default_samples[1] = {2500.0, 0.0};
      default_samples[2] = {5000.0, 10.0};
      break;
    case TRANSMISSION_GEAR:
      is_active = true;
      measurement_type = "resistance";
      output_unit = "";  // gear state is unitless
      sk_path = "propulsion." + instance + ".transmission.gear";
      raw_sk_path = "sensors." + hardware_id + ".resistance";
      curve_title = instance + " Shifter Curve";
      curve_description = "Map resistance to shifter position";
      config_path = "/Transmission/" + instance + "/Shifter Curve";
      default_samples[0] = {0.0, -1.0};  // Reverse
      default_samples[1] = {1667.0, 0.0};  // Neutral
      default_samples[2] = {3333.0, 1.0};  // Forward
      break;
    case THROTTLE_POSITION:
      is_active = true;
      measurement_type = "resistance";
      output_unit = "%";
      sk_path = "propulsion." + instance + ".throttleState";
      raw_sk_path = "sensors." + hardware_id + ".resistance";
      curve_title = instance + " Throttle Curve";
      curve_description = "Map resistance to %";
      config_path = "/Throttle/" + instance + "/Position Curve";
      default_samples[0] = {0.0, 0.0};
      default_samples[1] = {2500.0, 50.0};
      default_samples[2] = {5000.0, 100.0};
      break;
    case GENERIC_VOLTAGE:
      is_active = false;
      measurement_type = "voltage";
      output_unit = "V";
      sk_path = "sensors.generic." + instance + ".voltage";
      raw_sk_path = "sensors." + hardware_id + ".voltage";
      curve_title = instance + " Generic Voltage Curve";
      curve_description = "Map voltage to output";
      config_path = "/Generic/" + instance + "/Voltage Curve";
      default_samples[0] = {0.0, 0.0};
      default_samples[1] = {2.5, 2.5};
      default_samples[2] = {5.0, 5.0};
      break;
    case GENERIC_RESISTANCE:
      is_active = true;
      measurement_type = "resistance";
      output_unit = "Ω";
      sk_path = "sensors.generic." + instance + ".resistance";
      raw_sk_path = "sensors." + hardware_id + ".resistance";
      curve_title = instance + " Generic Resistance Curve";
      curve_description = "Map resistance to output";
      config_path = "/Generic/" + instance + "/Resistance Curve";
      default_samples[0] = {0.0, 0.0};
      default_samples[1] = {2500.0, 2500.0};
      default_samples[2] = {5000.0, 5000.0};
      break;
  }

  // Create the sensor
  auto* sensor = new sensesp::RepeatSensor<float>(
      read_delay,
      [ads1115, channel, is_active]() {
        int16_t raw = ads1115->readADC_SingleEnded(channel);
        float volts = ads1115->computeVolts(raw);
        if (is_active) {
          return kVoltageDividerScale * volts / kMeasurementCurrent;  // resistance
        } else {
          return kVoltageDividerScale * volts;  // voltage
        }
      }
  );

  // Update raw sensor values for status display and create StatusPageItem
  if (g_enable_calibration) {
    // Create StatusPageItem and connect it directly to the sensor
    auto* status_item = new CalibrationStatusPageItem<float>(
        hardware_id.c_str(), 0.0f, "Calibration", 4000 + sort_order % 100
    );
    sensor->connect_to(status_item);
    
    // Store in global map for reference
    raw_sensor_status_items[hardware_id.c_str()] = status_item;
    
    sensor->connect_to(new halmet::RawValueConsumer(hardware_id.c_str()));
  }

  // Raw output
  if (g_enable_calibration) {
    auto* raw_sk_out = new sensesp::SKOutputFloat(
        raw_sk_path, "",
        new sensesp::SKMetadata(measurement_type == "voltage" ? "V" : "Ω", hardware_id + " " + measurement_type)
    );
    sensor->connect_to(raw_sk_out);
  }

  // Curve
  auto* curve = new sensesp::CurveInterpolator(nullptr, config_path);
  curve->set_input_title(measurement_type + " (" + (measurement_type == "voltage" ? "V" : "Ω") + ")");
  curve->set_output_title("Output (" + output_unit + ")");
  ConfigItem(curve)
      ->set_title(curve_title)
      ->set_description(curve_description)
      ->set_sort_order(sort_order);
  if (curve->get_samples().empty()) {
    curve->clear_samples();
    for (auto& sample : default_samples) {
      curve->add_sample(sample);
    }
  }
  sensor->connect_to(curve);

  // Calibration: offset and multiplier
  auto* calibrated = new sensesp::LambdaTransform<float, float>(
      [offset, multiplier](float value) {
        return value * multiplier + offset;
      }
  );
  curve->connect_to(calibrated);

  // Signal K output
  if (enable_signalk_output && !sk_path.isEmpty()) {
    sensesp::SKMetadata* metadata = nullptr;
    if (type == TEMPERATURE) {
      metadata = new sensesp::SKMetadata("K", instance + " Coolant Temp");
      auto* to_kelvin = new sensesp::LambdaTransform<float, double>(
          [](float f) { return (f - 32.0) * 5.0 / 9.0 + 273.15; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_kelvin)->connect_to(sk_out);
    } else if (type == PRESSURE) {
      metadata = new sensesp::SKMetadata("Pa", instance + " Oil Pressure");
      auto* to_pascal = new sensesp::LambdaTransform<float, double>(
          [](float psi) { return psi * 6894.76; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_pascal)->connect_to(sk_out);
    } else if (type == RUDDER_ANGLE) {
      metadata = new sensesp::SKMetadata("rad", "Rudder Angle");
      auto* to_rad = new sensesp::LambdaTransform<float, double>(
          [](float deg) { return deg * DEG_TO_RAD; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_rad)->connect_to(sk_out);
    } else if (type == FUEL_LEVEL) {
      metadata = new sensesp::SKMetadata("ratio", instance + " Fuel Level");
      auto* to_ratio = new sensesp::LambdaTransform<float, double>(
          [](float pct) { return pct / 100.0; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_ratio)->connect_to(sk_out);
    } else if (type == WATER_LEVEL) {
      metadata = new sensesp::SKMetadata("ratio", instance + " Water Level");
      auto* to_ratio = new sensesp::LambdaTransform<float, double>(
          [](float pct) { return pct / 100.0; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_ratio)->connect_to(sk_out);
    } else if (type == BLACK_WATER_LEVEL) {
      metadata = new sensesp::SKMetadata("ratio", instance + " Black Water Level");
      auto* to_ratio = new sensesp::LambdaTransform<float, double>(
          [](float pct) { return pct / 100.0; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_ratio)->connect_to(sk_out);
    } else if (type == GRAY_WATER_LEVEL) {
      metadata = new sensesp::SKMetadata("ratio", instance + " Gray Water Level");
      auto* to_ratio = new sensesp::LambdaTransform<float, double>(
          [](float pct) { return pct / 100.0; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_ratio)->connect_to(sk_out);
    } else if (type == BATTERY_VOLTAGE) {
      metadata = new sensesp::SKMetadata("V", instance + " Battery Voltage");
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(sk_out);
    } else if (type == EXHAUST_TEMPERATURE) {
      metadata = new sensesp::SKMetadata("K", instance + " Exhaust Temp");
      auto* to_kelvin = new sensesp::LambdaTransform<float, double>(
          [](float f) { return (f - 32.0) * 5.0 / 9.0 + 273.15; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_kelvin)->connect_to(sk_out);
    } else if (type == BILGE_LEVEL) {
      metadata = new sensesp::SKMetadata("ratio", instance + " Bilge Level");
      auto* to_ratio = new sensesp::LambdaTransform<float, double>(
          [](float pct) { return pct / 100.0; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_ratio)->connect_to(sk_out);
    } else if (type == GENERIC_CURRENT) {
      metadata = new sensesp::SKMetadata("A", instance + " Current");
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(sk_out);
    } else if (type == GENERIC_TEMPERATURE) {
      metadata = new sensesp::SKMetadata("K", instance + " Temperature");
      auto* to_kelvin = new sensesp::LambdaTransform<float, double>(
          [](float f) { return (f - 32.0) * 5.0 / 9.0 + 273.15; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_kelvin)->connect_to(sk_out);
    } else if (type == GENERIC_PRESSURE) {
      metadata = new sensesp::SKMetadata("Pa", instance + " Pressure");
      auto* to_pascal = new sensesp::LambdaTransform<float, double>(
          [](float psi) { return psi * 6894.76; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_pascal)->connect_to(sk_out);
    } else if (type == TRIM_ANGLE) {
      if (g_single_trim_sensor) {
        // Port
        sensesp::SKMetadata* metadata_port = new sensesp::SKMetadata("rad", "Port Trim Tab");
        auto* to_rad_port = new sensesp::LambdaTransform<float, double>([](float deg) { return deg * DEG_TO_RAD; });
        auto* sk_out_port = new sensesp::SKOutputFloat("propulsion.port.trimState", "", metadata_port);
        calibrated->connect_to(to_rad_port)->connect_to(sk_out_port);
        // Stbd
        sensesp::SKMetadata* metadata_stbd = new sensesp::SKMetadata("rad", "Stbd Trim Tab");
        auto* to_rad_stbd = new sensesp::LambdaTransform<float, double>([](float deg) { return deg * DEG_TO_RAD; });
        auto* sk_out_stbd = new sensesp::SKOutputFloat("propulsion.stbd.trimState", "", metadata_stbd);
        calibrated->connect_to(to_rad_stbd)->connect_to(sk_out_stbd);
      } else {
        metadata = new sensesp::SKMetadata("rad", instance + " Trim Tab");
        auto* to_rad = new sensesp::LambdaTransform<float, double>([](float deg) { return deg * DEG_TO_RAD; });
        auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
        calibrated->connect_to(to_rad)->connect_to(sk_out);
      }
    } else if (type == THROTTLE_POSITION) {
      metadata = new sensesp::SKMetadata("ratio", instance + " Throttle Position");
      auto* to_ratio = new sensesp::LambdaTransform<float, double>(
          [](float pct) { return pct / 100.0; }
      );
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(to_ratio)->connect_to(sk_out);
    } else if (type == TRANSMISSION_GEAR) {
      metadata = new sensesp::SKMetadata("", instance + " Transmission Gear");
      auto* round_transform = new sensesp::LambdaTransform<float, float>([](float input) { return roundf(input); });
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(round_transform)->connect_to(sk_out);
    } else {
      metadata = new sensesp::SKMetadata(output_unit.c_str(), instance + " " + curve_title);
      auto* sk_out = new sensesp::SKOutputFloat(sk_path, "", metadata);
      calibrated->connect_to(sk_out);
    }
  }

  return calibrated;
}
} // namespace halmet
