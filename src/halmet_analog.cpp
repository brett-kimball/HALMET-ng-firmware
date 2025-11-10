#include "halmet_analog.h"

#include "config_flags.h"

#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/ui_controls.h"

namespace halmet {

std::shared_ptr<sensesp::PersistingObservableValue<bool>> global_sk_enable = nullptr;

const float kMeasurementCurrent = 0.01;

sensesp::FloatProducer* ConnectAnalogInput(Adafruit_ADS1115* ads1115,
                                           int channel, const String& name,
                                           const String& sk_id, int sort_order,
                                           bool enable_signalk_output) {
  const uint ads_read_delay = 500;

  // === Input Mode ===
  char mode_config_path[80];
  snprintf(mode_config_path, sizeof(mode_config_path), "/Analog/%s/Input Mode", name.c_str());
  std::vector<String> mode_options = {"active", "passive"};
#if !DISABLE_LEGACY_CONFIG_ITEMS
  sensesp::SelectConfig* input_mode = new sensesp::SelectConfig("active", "Input Mode", mode_config_path, mode_options, sensesp::SelectType::kSelect);
  ConfigItem(input_mode)->set_title(name + " Input Mode")->set_description("Active (10mA) or Passive (voltage)")->set_sort_order(sort_order - 1);
#else
  // When legacy UI is disabled, create the config object with an empty path
  // so it does not register with the saveable/config system. Use the same
  // type to keep the rest of the logic unchanged. Allocate the empty path
  // on the heap so it can bind to String& parameters.
  auto* _cfg_empty_path_1 = new String("");
  sensesp::SelectConfig* input_mode = new sensesp::SelectConfig("active", "Input Mode", *_cfg_empty_path_1, mode_options, sensesp::SelectType::kSelect);
#endif

  // === Raw Input ===
  auto* raw_input = new sensesp::RepeatSensor<float>(ads_read_delay, [ads1115, channel, input_mode]() {
    int16_t adc = ads1115->readADC_SingleEnded(channel);
    float volts = ads1115->computeVolts(adc) * kVoltageDividerScale;
    return (input_mode->get_value() == "active") ? volts / kMeasurementCurrent : volts;
  });

  // === Sensor Type ===
  char type_config_path[80];
  snprintf(type_config_path, sizeof(type_config_path), "/Analog/%s/Sensor Type", name.c_str());
  std::vector<String> type_options = {"tank_level", "temperature", "pressure", "rudder_angle", "trim_tab", "transmission_gear"};
#if !DISABLE_LEGACY_CONFIG_ITEMS
  sensesp::SelectConfig* sensor_type = new sensesp::SelectConfig("tank_level", "Sensor Type", type_config_path, type_options, sensesp::SelectType::kSelect);
  ConfigItem(sensor_type)->set_title(name + " Sensor Type")->set_description("Type of measurement")->set_sort_order(sort_order - 2);
#else
  auto* _cfg_empty_path_2 = new String("");
  sensesp::SelectConfig* sensor_type = new sensesp::SelectConfig("tank_level", "Sensor Type", *_cfg_empty_path_2, type_options, sensesp::SelectType::kSelect);
#endif

  // === Calibration Curve ===
  char curve_config_path[80];
  snprintf(curve_config_path, sizeof(curve_config_path), "/Analog/%s/Curve", name.c_str());
#if !DISABLE_LEGACY_CONFIG_ITEMS
  auto* calibrated = new sensesp::CurveInterpolator(nullptr, curve_config_path);
  ConfigItem(calibrated)->set_title(name + " Calibration Curve")->set_sort_order(sort_order);
#else
  auto* _cfg_empty_path_3 = new String("");
  auto* calibrated = new sensesp::CurveInterpolator(nullptr, *_cfg_empty_path_3);
#endif

  // Default curve
  if (calibrated->get_samples().empty()) {
    calibrated->clear_samples();
    String type = sensor_type->get_value();
    if (type == "tank_level") {
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(0, 0));
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(180, 1));
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(1000, 1));
    } else if (type == "temperature") {
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(0, 0));
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(3.3, 100));
    } else if (type == "pressure") {
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(0, 0));
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(3.3, 10));
    } else if (type == "rudder_angle") {
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(0, -45));
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(3.3, 45));
    } else if (type == "trim_tab") {
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(0, -30));
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(3.3, 30));
    } else if (type == "transmission_gear") {
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(0, 0));
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(1.65, 1));
      calibrated->add_sample(sensesp::CurveInterpolator::Sample(3.3, 2));
    }
  }

  raw_input->connect_to(calibrated);

  // === Scale Factor ===
  char scale_config_path[80];
  snprintf(scale_config_path, sizeof(scale_config_path), "/Analog/%s/Scale", name.c_str());
#if !DISABLE_LEGACY_CONFIG_ITEMS
  auto* physical = new sensesp::Linear(1.0f, 0.0f, scale_config_path);
  ConfigItem(physical)->set_title(name + " Scale Factor")->set_sort_order(sort_order + 1);
#else
  auto* _cfg_empty_path_4 = new String("");
  auto* physical = new sensesp::Linear(1.0f, 0.0f, *_cfg_empty_path_4);
#endif

// Raw SK Path
char raw_sk_path_config[80];
snprintf(raw_sk_path_config, sizeof(raw_sk_path_config), "/Analog/%s/Raw SK Path", name.c_str());
String raw_sk_path_str = "sensors." + sk_id + ".raw";
String raw_sk_path_config_str = raw_sk_path_config;
#if !DISABLE_LEGACY_CONFIG_ITEMS
  auto* raw_sk_path = new sensesp::StringConfig(raw_sk_path_str, raw_sk_path_config_str);
  ConfigItem(raw_sk_path)
    ->set_title(name + " Raw SK Path")
    ->set_description("Leave blank or 'null' to disable")
    ->set_sort_order(sort_order + 1);
#else
  auto* _cfg_empty_path_5 = new String("");
  auto* raw_sk_path = new sensesp::StringConfig(raw_sk_path_str, *_cfg_empty_path_5);
#endif

// Calibrated SK Path
char cal_sk_path_config[80];
snprintf(cal_sk_path_config, sizeof(cal_sk_path_config), "/Analog/%s/Calibrated SK Path", name.c_str());
String cal_sk_path_str = "sensors." + sk_id + ".calibrated";
String cal_sk_path_config_str = cal_sk_path_config;
#if !DISABLE_LEGACY_CONFIG_ITEMS
  auto* cal_sk_path = new sensesp::StringConfig(cal_sk_path_str, cal_sk_path_config_str);
  ConfigItem(cal_sk_path)
    ->set_title(name + " Calibrated SK Path")
    ->set_description("Leave blank or 'null' to disable")
    ->set_sort_order(sort_order + 2);
#else
  auto* _cfg_empty_path_6 = new String("");
  auto* cal_sk_path = new sensesp::StringConfig(cal_sk_path_str, *_cfg_empty_path_6);
#endif

// Physical SK Path
char phys_sk_path_config[80];
snprintf(phys_sk_path_config, sizeof(phys_sk_path_config), "/Analog/%s/Physical SK Path", name.c_str());
String phys_sk_path_str = "sensors." + sk_id + ".physical";
String phys_sk_path_config_str = phys_sk_path_config;
#if !DISABLE_LEGACY_CONFIG_ITEMS
  auto* phys_sk_path = new sensesp::StringConfig(phys_sk_path_str, phys_sk_path_config_str);
  ConfigItem(phys_sk_path)
    ->set_title(name + " Physical SK Path")
    ->set_description("Leave blank or 'null' to disable")
    ->set_sort_order(sort_order + 3);
#else
  auto* _cfg_empty_path_7 = new String("");
  auto* phys_sk_path = new sensesp::StringConfig(phys_sk_path_str, *_cfg_empty_path_7);
#endif
  calibrated->connect_to(physical);

// === Signal K Output ===
if (global_sk_enable && global_sk_enable->get()) {
  String raw_path = raw_sk_path->get_value();
  if (!raw_path.isEmpty() && raw_path != "null") {
    auto* raw_sk_output = new sensesp::SKOutputFloat(
        raw_path, raw_sk_path_config,
        new sensesp::SKMetadata("ohm", name + " Raw"));
    raw_input->connect_to(raw_sk_output);  // <-- ADD THIS
  }

  String cal_path = cal_sk_path->get_value();
  if (!cal_path.isEmpty() && cal_path != "null") {
    String unit = "ratio";
    String type = sensor_type->get_value();
    if (type == "temperature") unit = "K";
    else if (type == "pressure") unit = "Pa";
    else if (type == "rudder_angle") unit = "rad";
    else if (type == "trim_tab") unit = "rad";

    auto* cal_sk_output = new sensesp::SKOutputFloat(
        cal_path, cal_sk_path_config,
        new sensesp::SKMetadata(unit, name + " Calibrated"));
    calibrated->connect_to(cal_sk_output);  // <-- ADD THIS
  }

  String phys_path = phys_sk_path->get_value();
  if (!phys_path.isEmpty() && phys_path != "null") {
    String unit = "cubicMeters";
    String type = sensor_type->get_value();
    if (type == "temperature") unit = "K";
    else if (type == "pressure") unit = "Pa";
    else if (type == "rudder_angle") unit = "rad";
    else if (type == "trim_tab") unit = "rad";

    auto* phys_sk_output = new sensesp::SKOutputFloat(
        phys_path, phys_sk_path_config,
        new sensesp::SKMetadata(unit, name + " Physical"));
    physical->connect_to(phys_sk_output);  // <-- ADD THIS
  }
}

  return physical;
}

}  // namespace halmet