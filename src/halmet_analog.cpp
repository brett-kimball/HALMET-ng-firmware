// halmet_analog.cpp — FINAL WITH OIL & TEMP CURVES IN WEB UI
#include "halmet_analog.h"
#include <N2kMessages.h>
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/ui/config_item.h"
namespace halmet {
const float kMeasurementCurrent = 0.01;
// ========================================================================
// TEMPERATURE SENDER (PASSIVE MODE)
// ========================================================================
sensesp::FloatProducer* ConnectTemperatureSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output
) {
  const uint read_delay = 500;
  auto* voltage = new sensesp::RepeatSensor<float>(
      read_delay,
      [ads1115, channel]() {
        int16_t raw = ads1115->readADC_SingleEnded(channel);
        float volts = ads1115->computeVolts(raw);
        return kVoltageDividerScale * volts;
      }
  );

  // === RAW VOLTAGE OUTPUT (PASSIVE) ===
  if (enable_signalk_output) {
    char voltage_sk_path[80];
    snprintf(voltage_sk_path, sizeof(voltage_sk_path), "vessels.self.local.sensors.temp.%s.voltage", sk_id.c_str());
    auto* voltage_sk_out = new sensesp::SKOutputFloat(
        voltage_sk_path, "/Temp/" + name + "/Voltage SK Path",
        new sensesp::SKMetadata("V", name + " Sender Voltage")
    );
    ConfigItem(voltage_sk_out)
        ->set_title(name + " Voltage SK Path")
        ->set_sort_order(sort_order + 1);
    voltage->connect_to(voltage_sk_out);
  }

  char curve_path[80];
  snprintf(curve_path, sizeof(curve_path), "/Temp/%s/Fahrenheit Curve", name.c_str());
  auto* temp_f = new sensesp::CurveInterpolator(nullptr, curve_path);
  temp_f->set_input_title("Voltage (V)");
  temp_f->set_output_title("Temperature (°F)");
  // EXPOSE IN WEB UI
  ConfigItem(temp_f)
      ->set_title(name + " Temperature Curve (°F)")
      ->set_description("Map voltage to °F")
      ->set_sort_order(sort_order);
  if (temp_f->get_samples().empty()) {
    temp_f->clear_samples();
    temp_f->add_sample(sensesp::CurveInterpolator::Sample(0.5, 77.0));
    temp_f->add_sample(sensesp::CurveInterpolator::Sample(2.0, 140.0));
    temp_f->add_sample(sensesp::CurveInterpolator::Sample(3.5, 194.0));
  }
  voltage->connect_to(temp_f);
  if (enable_signalk_output) {
    auto* to_kelvin = new sensesp::LambdaTransform<float, double>(
        [](float f) { return (f - 32.0) * 5.0 / 9.0 + 273.15; }
    );
    char sk_path[80];
    snprintf(sk_path, sizeof(sk_path), "propulsion.%s.coolantTemperature", sk_id.c_str());
    auto* sk_out = new sensesp::SKOutputFloat(
        sk_path, "",
        new sensesp::SKMetadata("K", name + " Coolant Temp")
    );
    temp_f->connect_to(to_kelvin)->connect_to(sk_out);
  }
  return temp_f;
}
// ========================================================================
// PRESSURE SENDER (PASSIVE MODE)
// ========================================================================
sensesp::FloatProducer* ConnectPressureSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output
) {
  const uint read_delay = 500;
  auto* voltage = new sensesp::RepeatSensor<float>(
      read_delay,
      [ads1115, channel]() {
        int16_t raw = ads1115->readADC_SingleEnded(channel);
        float volts = ads1115->computeVolts(raw);
        return kVoltageDividerScale * volts;
      }
  );

  // === RAW VOLTAGE OUTPUT (PASSIVE) ===
  if (enable_signalk_output) {
    char voltage_sk_path[80];
    snprintf(voltage_sk_path, sizeof(voltage_sk_path), "vessels.self.local.sensors.oil.%s.voltage", sk_id.c_str());
    auto* voltage_sk_out = new sensesp::SKOutputFloat(
        voltage_sk_path, "/Pressure/" + name + "/Voltage SK Path",
        new sensesp::SKMetadata("V", name + " Sender Voltage")
    );
    ConfigItem(voltage_sk_out)
        ->set_title(name + " Voltage SK Path")
        ->set_sort_order(sort_order + 1);
    voltage->connect_to(voltage_sk_out);
  }

  char curve_path[80];
  snprintf(curve_path, sizeof(curve_path), "/Pressure/%s/PSI Curve", name.c_str());
  auto* pressure_psi = new sensesp::CurveInterpolator(nullptr, curve_path);
  pressure_psi->set_input_title("Voltage (V)");
  pressure_psi->set_output_title("Pressure (PSI)");
  // EXPOSE IN WEB UI
  ConfigItem(pressure_psi)
      ->set_title(name + " Oil Pressure Curve")
      ->set_description("Map voltage to PSI")
      ->set_sort_order(sort_order);
  if (pressure_psi->get_samples().empty()) {
    pressure_psi->clear_samples();
    pressure_psi->add_sample(sensesp::CurveInterpolator::Sample(0.5, 0.0));
    pressure_psi->add_sample(sensesp::CurveInterpolator::Sample(2.5, 50.0));
    pressure_psi->add_sample(sensesp::CurveInterpolator::Sample(4.5, 100.0));
  }
  voltage->connect_to(pressure_psi);
  if (enable_signalk_output) {
    auto* to_pascal = new sensesp::LambdaTransform<float, double>(
        [](float psi) { return psi * 6894.76; }
    );
    char sk_path[80];
    snprintf(sk_path, sizeof(sk_path), "propulsion.%s.oilPressure", sk_id.c_str());
    auto* sk_out = new sensesp::SKOutputFloat(
        sk_path, "",
        new sensesp::SKMetadata("Pa", name + " Oil Pressure")
    );
    pressure_psi->connect_to(to_pascal)->connect_to(sk_out);
  }
  return pressure_psi;
}
// ========================================================================
// RUDDER ANGLE SENDER (ACTIVE MODE)
// ========================================================================
sensesp::FloatProducer* ConnectRudderAngleSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output
) {
  const uint read_delay = 500;
  auto* resistance = new sensesp::RepeatSensor<float>(
      read_delay,
      [ads1115, channel]() {
        int16_t raw = ads1115->readADC_SingleEnded(channel);
        float volts = ads1115->computeVolts(raw);
        return kVoltageDividerScale * volts / kMeasurementCurrent;
      }
  );

  // === RAW RESISTANCE OUTPUT (ACTIVE) ===
  if (enable_signalk_output) {
    char resistance_sk_path[80];
    snprintf(resistance_sk_path, sizeof(resistance_sk_path), "vessels.self.local.sensors.rudder.%s.resistance", sk_id.c_str());
    auto* resistance_sk_out = new sensesp::SKOutputFloat(
        resistance_sk_path, "/Rudder/" + name + "/Resistance SK Path",
        new sensesp::SKMetadata("Ω", name + " Sender Resistance")
    );
    ConfigItem(resistance_sk_out)
        ->set_title(name + " Resistance SK Path")
        ->set_sort_order(sort_order + 1);
    resistance->connect_to(resistance_sk_out);
  }

  char curve_path[80];
  snprintf(curve_path, sizeof(curve_path), "/Rudder/%s/Angle Curve", name.c_str());
  auto* angle_deg = new sensesp::CurveInterpolator(nullptr, curve_path);
  angle_deg->set_input_title("Resistance (Ω)");
  angle_deg->set_output_title("Rudder Angle (°)");
  ConfigItem(angle_deg)
      ->set_title(name + " Rudder Angle Curve")
      ->set_description("Map resistance to angle")
      ->set_sort_order(sort_order);
  if (angle_deg->get_samples().empty()) {
    angle_deg->clear_samples();
    angle_deg->add_sample(sensesp::CurveInterpolator::Sample( 0.0, -45.0));
    angle_deg->add_sample(sensesp::CurveInterpolator::Sample( 95.0, 0.0));
    angle_deg->add_sample(sensesp::CurveInterpolator::Sample(190.0, 45.0));
  }
  resistance->connect_to(angle_deg);
  if (enable_signalk_output) {
    auto* to_rad = new sensesp::LambdaTransform<float, double>(
        [](float deg) { return deg * DEG_TO_RAD; }
    );
    auto* sk_out = new sensesp::SKOutputFloat(
        "steering.rudderAngle", "",
        new sensesp::SKMetadata("rad", "Rudder Angle")
    );
    angle_deg->connect_to(to_rad)->connect_to(sk_out);
  }
  return angle_deg;
}
// ========================================================================
// TRIM TAB ANGLE SENDER (ACTIVE MODE)
// ========================================================================
sensesp::FloatProducer* ConnectTrimAngleSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output
) {
  const uint read_delay = 500;
  auto* resistance = new sensesp::RepeatSensor<float>(
      read_delay,
      [ads1115, channel]() {
        int16_t raw = ads1115->readADC_SingleEnded(channel);
        float volts = ads1115->computeVolts(raw);
        return kVoltageDividerScale * volts / kMeasurementCurrent;
      }
  );

  // === RAW RESISTANCE OUTPUT (ACTIVE) ===
  if (enable_signalk_output) {
    char resistance_sk_path[80];
    snprintf(resistance_sk_path, sizeof(resistance_sk_path), "vessels.self.local.sensors.trim.%s.resistance", sk_id.c_str());
    auto* resistance_sk_out = new sensesp::SKOutputFloat(
        resistance_sk_path, "/Trim/" + name + "/Resistance SK Path",
        new sensesp::SKMetadata("Ω", name + " Sender Resistance")
    );
    ConfigItem(resistance_sk_out)
        ->set_title(name + " Resistance SK Path")
        ->set_sort_order(sort_order + 1);
    resistance->connect_to(resistance_sk_out);
  }

  char curve_path[80];
  snprintf(curve_path, sizeof(curve_path), "/Trim/%s/Angle Curve", name.c_str());
  auto* trim_deg = new sensesp::CurveInterpolator(nullptr, curve_path);
  trim_deg->set_input_title("Resistance (Ω)");
  trim_deg->set_output_title("Trim Angle (degrees)");
  ConfigItem(trim_deg)
      ->set_title(name + " Trim Angle Curve")
      ->set_description("Map resistance to trim angle")
      ->set_sort_order(sort_order);
  if (trim_deg->get_samples().empty()) {
    trim_deg->clear_samples();
    trim_deg->add_sample(sensesp::CurveInterpolator::Sample(0, -10));
    trim_deg->add_sample(sensesp::CurveInterpolator::Sample(2500, 0));
    trim_deg->add_sample(sensesp::CurveInterpolator::Sample(5000, 10));
  }
  resistance->connect_to(trim_deg);
  if (enable_signalk_output) {
    auto* port_out = new sensesp::SKOutputFloat(
        "steering.trimTab.port", "",
        new sensesp::SKMetadata("deg", name + " Port Trim Tab")
    );
    auto* stbd_out = new sensesp::SKOutputFloat(
        "steering.trimTab.starboard", "",
        new sensesp::SKMetadata("deg", name + " Starboard Trim Tab")
    );
    trim_deg->connect_to(port_out);
    trim_deg->connect_to(stbd_out);
  }
  return trim_deg;
}
// ========================================================================
// TRANSMISSION GEAR SENDER (ACTIVE MODE)
// ========================================================================
sensesp::FloatProducer* ConnectTransmissionGearSender(
    Adafruit_ADS1115* ads1115,
    int channel,
    const String& name,
    const String& sk_id,
    int sort_order,
    bool enable_signalk_output
) {
  const uint read_delay = 500;
  auto* resistance = new sensesp::RepeatSensor<float>(
      read_delay,
      [ads1115, channel]() {
        int16_t raw = ads1115->readADC_SingleEnded(channel);
        float volts = ads1115->computeVolts(raw);
        return kVoltageDividerScale * volts / kMeasurementCurrent;
      }
  );

  // === RAW RESISTANCE OUTPUT (ACTIVE) ===
  if (enable_signalk_output) {
    char resistance_sk_path[80];
    snprintf(resistance_sk_path, sizeof(resistance_sk_path), "vessels.self.local.sensors.gear.%s.resistance", sk_id.c_str());
    auto* resistance_sk_out = new sensesp::SKOutputFloat(
        resistance_sk_path, "/Transmission/" + name + "/Resistance SK Path",
        new sensesp::SKMetadata("Ω", name + " Sender Resistance")
    );
    ConfigItem(resistance_sk_out)
        ->set_title(name + " Resistance SK Path")
        ->set_sort_order(sort_order + 1);
    resistance->connect_to(resistance_sk_out);
  }

  char curve_path[80];
  snprintf(curve_path, sizeof(curve_path), "/Transmission/%s/Shifter Curve", name.c_str());
  auto* shifter_position = new sensesp::CurveInterpolator(nullptr, curve_path);
  shifter_position->set_input_title("Resistance (Ω)");
  shifter_position->set_output_title("Shifter Position (0-1)");
  ConfigItem(shifter_position)
      ->set_title(name + " Shifter Curve")
      ->set_description("Map resistance to shifter position")
      ->set_sort_order(sort_order);
  if (shifter_position->get_samples().empty()) {
    shifter_position->clear_samples();
    shifter_position->add_sample(sensesp::CurveInterpolator::Sample( 0.0, 0.0));
    shifter_position->add_sample(sensesp::CurveInterpolator::Sample(2500.0, 0.5));
    shifter_position->add_sample(sensesp::CurveInterpolator::Sample(5000.0, 1.0));
  }
  resistance->connect_to(shifter_position);
  if (enable_signalk_output) {
    char sk_path[80];
    snprintf(sk_path, sizeof(sk_path), "propulsion.%s.gear", sk_id.c_str());
    auto* sk_out = new sensesp::SKOutputFloat(
        sk_path, "",
        new sensesp::SKMetadata("ratio", name + " Shifter Position")
    );
    shifter_position->connect_to(sk_out);
  }
  return shifter_position;
}
} // namespace halmet