#include "halmet_digital.h"
#include "halmet_analog.h"

#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/ui/config_item.h"

using namespace sensesp;

// Default: 1 pulse per revolution
const float kDefaultPulsesPerRev = 1.0;

namespace halmet {

FloatProducer* ConnectTachoSender(int pin, String name) {
  char config_path[80];
  char sk_path[80];
  char config_title[80];
  char config_description[80];

  // Input pin
  snprintf(config_path, sizeof(config_path), "/Tacho/%s/Pin", name.c_str());
  snprintf(config_title, sizeof(config_title), "Tacho %s Pin", name.c_str());
  snprintf(config_description, sizeof(config_description), "Input pin for %s tacho", name.c_str());

  auto* tacho_input = new DigitalInputCounter(pin, INPUT_PULLUP, RISING, 500, config_path);
  ConfigItem(tacho_input)
      ->set_title(config_title)
      ->set_description(config_description);

  // Pulses per Revolution (1.0 = 1 pulse per rev)
  snprintf(config_path, sizeof(config_path), "/Tacho/%s/Pulses per Rev", name.c_str());
  snprintf(config_title, sizeof(config_title), "Tacho %s Pulses/Rev", name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Number of pulses per engine revolution for %s", name.c_str());

  auto* tacho_frequency = new Frequency(1.0 / kDefaultPulsesPerRev, config_path);
  ConfigItem(tacho_frequency)
      ->set_title(config_title)
      ->set_description(config_description);

  tacho_input->connect_to(tacho_frequency);

#ifdef ENABLE_SIGNALK
  // Signal K: revolutions (Hz)
  snprintf(config_path, sizeof(config_path), "/Tacho/%s/SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "propulsion.%s.revolutions", name.c_str());
  snprintf(config_title, sizeof(config_title), "Tacho %s Signal K Path", name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Signal K path for %s engine revolutions", name.c_str());

  auto* sk_out = new SKOutputFloat(sk_path, config_path,
      new SKMetadata("Hz", name + " Engine RPM", "Engine revolutions per second"));
  ConfigItem(sk_out)
      ->set_title(config_title)
      ->set_description(config_description);

  tacho_frequency->connect_to(sk_out);
#endif

  return tacho_frequency;
}

BoolProducer* ConnectAlarmSender(int pin, String name) {
  char config_path[80];
  char sk_path[80];
  char config_title[80];
  char config_description[80];

  auto* alarm_input = new DigitalInputState(pin, INPUT_PULLUP, 100);

#ifdef ENABLE_SIGNALK
  snprintf(config_path, sizeof(config_path), "/Alarm/%s/SK Path", name.c_str());
  snprintf(sk_path, sizeof(sk_path), "alarm.%s", name.c_str());
  snprintf(config_title, sizeof(config_title), "Alarm %s Signal K Path", name.c_str());
  snprintf(config_description, sizeof(config_description),
           "Signal K path for %s alarm", name.c_str());

  auto* sk_out = new SKOutputBool(sk_path, config_path);
  ConfigItem(sk_out)
      ->set_title(config_title)
      ->set_description(config_description);

  alarm_input->connect_to(sk_out);
#endif

  // Create raw value consumer for calibration mode
  if (g_enable_calibration) {
    // Convert boolean to float for calibration display
    auto* bool_to_float = new sensesp::LambdaTransform<bool, float>(
        [](bool value) { return value ? 1.0f : 0.0f; }
    );
    alarm_input->connect_to(bool_to_float);
    
    // Create StatusPageItem and connect it to the float converter
    auto* status_item = new CalibrationStatusPageItem<float>(
        name.c_str(), 0.0f, "Calibration", 4100
    );
    bool_to_float->connect_to(status_item);
    
    // Store in global map for reference
    raw_sensor_status_items[name.c_str()] = status_item;
    
    bool_to_float->connect_to(new RawValueConsumer(name.c_str()));
    
    // Also create raw SignalK output for digital inputs in calibration mode
    char raw_sk_path[80];
    snprintf(raw_sk_path, sizeof(raw_sk_path), "sensors.%s", name.c_str());
    auto* raw_sk_out = new sensesp::SKOutputFloat(
        raw_sk_path, "",
        new sensesp::SKMetadata("ratio", String(name.c_str()) + " Digital Input Raw")
    );
    bool_to_float->connect_to(raw_sk_out);
  }

  return alarm_input;
}

}  // namespace halmet