// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NMEA2000_esp32.h>

#include "n2k_senders.h"
#include "sensesp/net/discovery.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/system_status_led.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/system/observablevalue.h"
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"
#include "SPIFFS.h"
#include "config_json.h"

using namespace sensesp;
using namespace halmet;

/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.

tNMEA2000* nmea2000;
elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

TwoWire* i2c;
Adafruit_SSD1306* display;

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

// Set the ADS1115 GAIN to adjust the analog input voltage range.
// On HALMET, this refers to the voltage range of the ADS1115 input
// AFTER the 33.3/3.3 voltage divider.

// GAIN_TWOTHIRDS: 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// GAIN_ONE:       1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// GAIN_TWO:       2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// GAIN_FOUR:      4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// GAIN_EIGHT:     8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// GAIN_SIXTEEN:   16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

const adsGain_t kADS1115Gain = GAIN_ONE;

/////////////////////////////////////////////////////////////////////
// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.
const int kTestOutputFrequency = 380;
#endif

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);
  // esp_log_level_set("*", ESP_LOG_DEBUG);


  // These calls can be used for fine-grained control over the logging level.
  // esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);

  Serial.begin(115200);

  /////////////////////////////////////////////////////////////////////
  // Initialize the application framework

  // Construct the global SensESPApp() object
  BUILDER_CLASS builder;
  sensesp_app = (&builder)
                    // EDIT: Set a custom hostname for the app.
                    ->set_hostname("halmet")
                    // EDIT: Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    // EDIT: Enable OTA updates with a password.
                    ->enable_ota("thisisfine")
                    ->get_app();

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();

  ads1115->setGain(kADS1115Gain);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  // Set the LEDC peripheral to a 13-bit resolution
  ledcAttach(kTestOutputPin, kTestOutputFrequency, 13);
  // Set the duty cycle to 50%
  // Duty cycle value is calculated based on the resolution
  // For 13-bit resolution, max value is 8191, so 50% is 4096
  ledcWrite(0, 4096);
#endif

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20231229",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "HALMET",    // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    71  // Default N2k node address
  );
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  event_loop()->onRepeat(10, []() { nmea2000->ParseMessages(); });

  // Initialize the OLED display
  bool display_present = InitializeSSD1306(sensesp_app->get(), &display, i2c);

if (!global_sk_enable) {
  global_sk_enable = std::make_shared<sensesp::PersistingObservableValue<bool>>(true, "/System/Enable Signal K Output");
  ConfigItem(global_sk_enable)
      ->set_title("Enable Signal K Output")
      ->set_description("Global toggle for all analog Signal K paths")
      ->set_sort_order(0);
}

  // Ensure SPIFFS is mounted for config JSON operations
  if (!SPIFFS.begin(true)) {
    Serial.println("WARN: SPIFFS mount failed — config JSON features disabled");
  }

  // Prepare heap-allocated global import/export strings so they are in scope
  // for polling tasks defined later in setup().
  auto* global_config_export_def = new String(halmet_config::get_merged_config_string());
  if (global_config_export_def->length() == 0) *global_config_export_def = String("{}");
  auto* global_config_export_path = new String("/System/ConfigExport");
  auto* global_config_import_def = new String("");
  auto* global_config_import_path = new String("/System/ConfigImport");
  // Import error display
  auto* global_config_import_err_def = new String("");
  auto* global_config_import_err_path = new String("/System/ConfigImportError");

  // Register Export/Import fields under top-level ConfigItems so they are visible
  {
    auto* export_cfg = new sensesp::StringConfig(*global_config_export_def, *global_config_export_path);
    auto export_ci = sensesp::ConfigItem(export_cfg);
    export_ci->set_title("Export Merged Config (read-only)")->set_sort_order(10)->set_description(String("Copy this JSON to backup or to another device"));

    auto* import_cfg = new sensesp::StringConfig(*global_config_import_def, *global_config_import_path);
    auto import_ci = sensesp::ConfigItem(import_cfg);
    import_ci->set_title("Import Overrides JSON")->set_sort_order(11)->set_description(String("Paste your persisted_config.json here and Save to apply overrides. Use Reset to revert."));

    // Import error display (read-only field)
    auto* import_err_cfg = new sensesp::StringConfig(*global_config_import_err_def, *global_config_import_err_path);
    auto import_err_ci = sensesp::ConfigItem(import_err_cfg);
    import_err_ci->set_title("Import Error")->set_sort_order(12)->set_description(String("Last import parse/save error (empty when OK)"));
  }

  // Per-input calibration JSON editors (A01..A04)
  for (int i = 1; i <= 4; ++i) {
    String name = String("A") + (i < 10 ? String("0") + String(i) : String(i));
    char path_buf[80];
    snprintf(path_buf, sizeof(path_buf), "/Inputs/%s/CalibrationJSON", name.c_str());
    String path_str = String(path_buf);
    // Attempt to extract existing calibration from merged config
    String default_json = String("");
    String merged = halmet_config::get_merged_config_string();
    if (merged.length() > 0) {
      DynamicJsonDocument d(16 * 1024);
      if (!deserializeJson(d, merged)) {
        if (d.containsKey("inputs") && d["inputs"].containsKey(name.c_str())) {
          JsonObject in = d["inputs"][name.c_str()].as<JsonObject>();
          // Prefer raw calibration text (preserves comments) if present in
          // merged overrides; otherwise fall back to the structured object.
          if (in.containsKey("calibration_raw")) {
            const char* raw = in["calibration_raw"].as<const char*>();
            if (raw) default_json = String(raw);
          } else if (in.containsKey("calibration")) {
            String tmp;
            serializeJson(in["calibration"], tmp);
            default_json = tmp;
          }
        }
      }
    }
    auto* cal_def = new String(default_json);
    auto* cal_path = new String(path_str);
    auto* cfg = new sensesp::StringConfig(*cal_def, *cal_path);
    auto ci = sensesp::ConfigItem(cfg);
    ci->set_title(name + String(" Calibration (JSON)"))->set_sort_order(1200 + i)->set_description(String("Advanced calibration JSON for ") + name + String(". Save to persist override; empty to revert to defaults."));

    // Calibration error field per-input
    auto* cal_err_def = new String("");
    auto* cal_err_path = new String(String("/Inputs/") + name + String("/CalibrationError"));
    auto* cal_err_cfg = new sensesp::StringConfig(*cal_err_def, *cal_err_path);
    auto cal_err_ci = sensesp::ConfigItem(cal_err_cfg);
    cal_err_ci->set_title(name + String(" Calibration Error"))->set_sort_order(1210 + i)->set_description(String("Last calibration parse/save error (empty when OK)"));

    // Persist changes to this calibration JSON on change (poll every 2s)
    String input_id = name;
    auto* cfg_path_ptr = new String(path_str);
    auto* prev_val = new String(*cal_def);
    // capture cal_err_path pointer for error writes
    event_loop()->onRepeat(2000, [prev_val, cfg_path_ptr, input_id, cal_err_path]() {
      String def_empty = String("");
      sensesp::StringConfig watcher(def_empty, *cfg_path_ptr);
      String v = watcher.get_value();
      if (v != *prev_val) {
        *prev_val = v;
        // Validate JSON first
        if (v.length() == 0 || v == "null") {
          // Clear override
          String err;
          if (halmet_config::set_input_calibration(input_id, String(""), &err)) {
            Serial.printf("Cleared calibration override for %s\n", input_id.c_str());
            // clear error field
            String* err_def2 = new String("");
            sensesp::StringConfig err_cfg(*err_def2, *cal_err_path);
            err_cfg.save();
          } else {
            Serial.printf("Failed to clear calibration override for %s: %s\n", input_id.c_str(), err.c_str());
            String* err_def2 = new String(err);
            sensesp::StringConfig err_cfg(*err_def2, *cal_err_path);
            err_cfg.save();
          }
        } else {
          DynamicJsonDocument d(8 * 1024);
          DeserializationError derr = deserializeJson(d, v);
          if (derr) {
            String msg = String("JSON parse error: ") + derr.c_str();
            Serial.printf("Calibration parse error for %s: %s\n", input_id.c_str(), derr.c_str());
            String* err_def2 = new String(msg);
            sensesp::StringConfig err_cfg(*err_def2, *cal_err_path);
            err_cfg.save();
          } else {
            // JSON parsed; attempt to save structured calibration and also
            // persist the raw text so comments are round-trippable.
            String err_struct;
            String err_raw;
            bool ok_struct = halmet_config::set_input_calibration(input_id, v, &err_struct);
            bool ok_raw = halmet_config::set_input_calibration_raw(input_id, v, &err_raw);
            if (ok_struct && ok_raw) {
              Serial.printf("Saved calibration override for %s (structured + raw)\n", input_id.c_str());
              // clear error field
              String* err_def2 = new String("");
              sensesp::StringConfig err_cfg(*err_def2, *cal_err_path);
              err_cfg.save();
            } else {
              // Prefer structured error if present, otherwise raw error
              String msg = ok_struct ? err_raw : err_struct;
              Serial.printf("Failed to save calibration override for %s: %s\n", input_id.c_str(), msg.c_str());
              String* err_def2 = new String(msg);
              sensesp::StringConfig err_cfg(*err_def2, *cal_err_path);
              err_cfg.save();
            }
          }
        }
      }
    });
  }

  // Polling task to process Import field every 2s
  event_loop()->onRepeat(2000, [global_config_import_path]() {
    // Use heap-allocated lvalues for StringConfig constructor
    String* path = new String(*global_config_import_path);
    String* def = new String("");
    sensesp::StringConfig import_cfg(*def, *path);
    String v = import_cfg.get_value();
    if (!v.isEmpty() && v != "null") {
      String err;
      if (halmet_config::save_overrides_from_string(v, &err)) {
        Serial.println("Imported overrides saved to SPIFFS");
      } else {
        Serial.printf("Failed to save overrides: %s\n", err.c_str());
      }
      // clear the import field after attempt
      String* def2 = new String("");
      sensesp::StringConfig clear_cfg(*def2, *path);
      clear_cfg.save();
    }
  });

  // ------------------------------------------------------------------
  // SPIFFS Cleanup / Format helpers (INACTIVE by default)
  // ------------------------------------------------------------------
  // This block implements two guarded, one-shot maintenance actions that
  // you can enable temporarily when you need to remove old persisted
  // configuration files or fully reformat SPIFFS on a device.
  //
  // Available compile-time flags (do NOT enable these in normal builds):
  //
  //  - FORCE_SPIFFS_CLEANUP
  //      When defined, the firmware will perform a file-by-file cleanup
  //      at boot: it will iterate over SPIFFS and remove all files except
  //      the `/www` folder (which contains the web UI static assets). The
  //      code then writes `/persist_cleanup_done` so the cleanup does not
  //      repeat on subsequent boots.
  //
  //  - FORCE_SPIFFS_FORMAT
  //      When defined, the firmware will call `SPIFFS.format()` to erase
  //      the entire SPIFFS filesystem. This is the most destructive option
  //      and will remove *all* files, including `/www` unless you reflash
  //      the filesystem image later. It writes `/persist_format_done` so
  //      it does not repeat.
  //
  // Safe usage instructions:
  // 1) BACKUP: Export `/System/ConfigExport` from the web UI and save it
  //    externally. Also note WiFi/OTA settings if needed.
  // 2) ENABLE: In your local copy of `platformio.ini`, temporarily add
  //    `-D FORCE_SPIFFS_CLEANUP=1` or `-D FORCE_SPIFFS_FORMAT=1` under the
  //    `[env:halmet]` `build_flags` section. Example:
  //
  //      [env:halmet]
  //      build_flags =
  //          ${pioarduino.build_flags}
  //          ${esp32.build_flags}
  //          -D ARDUINOJSON_ENABLE_COMMENTS=1
  //          -D FORCE_SPIFFS_CLEANUP=1
  //
  // 3) BUILD & FLASH: Build and upload firmware to the device. Monitor the
  //    serial output — the firmware will print which files it removes and
  //    whether the format succeeded.
  // 4) VERIFY: Open the web UI and confirm the desired cleanup occurred.
  // 5) REVERT: Immediately remove the `-D FORCE_*` flag(s) from
  //    `platformio.ini` and rebuild, or revert the commit. Leaving the
  //    flags enabled will re-run the destructive action on every new
  //    flash/boot, which is dangerous.
  //
  // Notes:
  //  - The code is intentionally guarded with #if defined(...) so it is
  //    inactive by default. The repository currently does NOT enable any
  //    FORCE_SPIFFS_* flags in `platformio.ini`.
  //  - The file-by-file cleanup preserves `/www` to keep the web UI
  //    assets. The format option erases everything and should be used only
  //    when you plan to reflash the UI assets or do a full restore.
  // ------------------------------------------------------------------
#if defined(FORCE_SPIFFS_CLEANUP)
  Serial.println("FORCED: performing one-shot SPIFFS cleanup (FORCE_SPIFFS_CLEANUP enabled)");
  File root = SPIFFS.open("/");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      String name = String(file.name());
      if (name != "/www") {
        Serial.printf("Removing SPIFFS file: %s\n", name.c_str());
        SPIFFS.remove(name);
      }
      file = root.openNextFile();
    }
    root.close();
  }
  // write the marker so normal code path will not attempt cleanup again
  const char* cleanup_marker = "/persist_cleanup_done";
  File mf = SPIFFS.open(cleanup_marker, FILE_WRITE);
  if (mf) { mf.printf("done\n"); mf.close(); Serial.println("persist_cleanup_done marker written"); }
#else
  const char* cleanup_marker = "/persist_cleanup_done";
  if (!SPIFFS.exists(cleanup_marker)) {
    Serial.println("persist_cleanup_done marker not found — performing one-shot SPIFFS cleanup");
    File root = SPIFFS.open("/");
    if (root) {
      File file = root.openNextFile();
      while (file) {
        String name = String(file.name());
        if (name != "/www") {
          Serial.printf("Removing SPIFFS file: %s\n", name.c_str());
          SPIFFS.remove(name);
        }
        file = root.openNextFile();
      }
      root.close();
    }
    File mf = SPIFFS.open(cleanup_marker, FILE_WRITE);
    if (mf) { mf.printf("done\n"); mf.close(); Serial.println("persist_cleanup_done marker written"); }
  }
#endif

  // One-shot SPIFFS FORMAT (destructive) - formats the filesystem completely
  // and writes /persist_format_done so it does not repeat. Controlled via
  // the FORCE_SPIFFS_FORMAT compile-time flag. This is more aggressive than
  // the file-by-file cleanup above and will erase everything in SPIFFS.
#if defined(FORCE_SPIFFS_FORMAT)
  if (!SPIFFS.exists("/persist_format_done")) {
    Serial.println("FORCED: formatting SPIFFS (FORCE_SPIFFS_FORMAT enabled)");
    bool ok = SPIFFS.format();
    Serial.printf("SPIFFS.format() returned: %d\n", ok);
    File mf2 = SPIFFS.open("/persist_format_done", FILE_WRITE);
    if (mf2) { mf2.printf("done\n"); mf2.close(); Serial.println("persist_format_done marker written"); }
  } else {
    Serial.println("persist_format_done marker present — skipping format");
  }
#endif

  ///////////////////////////////////////////////////////////////////
  // Analog inputs
  auto a01 = ConnectAnalogInput(ads1115, 0, "A01", "a01", 2010);
  auto a02 = ConnectAnalogInput(ads1115, 1, "A02", "a02", 2020);
  auto a03 = ConnectAnalogInput(ads1115, 2, "A03", "a03", 2030);
  auto a04 = ConnectAnalogInput(ads1115, 3, "A04", "a04", 2040);

#ifdef ENABLE_NMEA2000_OUTPUT
  // Tank 1, instance 0. Capacity 200 liters. You can change the capacity
  // in the web UI as well.
  // EDIT: Make sure this matches your tank configuration above.
  N2kFluidLevelSender* tank_a1_sender = new N2kFluidLevelSender(
      "/Tanks/Fuel/NMEA 2000", 0, N2kft_Fuel, 200, nmea2000);

  ConfigItem(tank_a1_sender)
      ->set_title("Tank A1 NMEA 2000")
      ->set_description("NMEA 2000 tank sender for tank A1")
      ->set_sort_order(3005);

  tank_a1_volume->connect_to(&(tank_a1_sender->tank_level_));
#endif  // ENABLE_NMEA2000_OUTPUT
/*
  if (display_present) {
    // EDIT: Duplicate the lines below to make the display show all your tanks.
    tank_a1_volume->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 2, "Tank A1", 100 * value); }));
  }

  // Read the voltage level of analog input A2
  auto a2_voltage = new ADS1115VoltageInput(ads1115, 1, "/Voltage A2");

  ConfigItem(a2_voltage)
      ->set_title("Analog Voltage A2")
      ->set_description("Voltage level of analog input A2")
      ->set_sort_order(3000);

  a2_voltage->connect_to(new LambdaConsumer<float>(
      [](float value) { debugD("Voltage A2: %f", value); }));
      

  // If you want to output something else than the voltage value,
  // you can insert a suitable transform here.
  // For example, to convert the voltage to a distance with a conversion
  // factor of 0.17 m/V, you could use the following code:
  // auto a2_distance = new Linear(0.17, 0.0);
  // a2_voltage->connect_to(a2_distance);

  a2_voltage->connect_to(
      new SKOutputFloat("sensors.a2.voltage", "Analog Voltage A2",
                        new SKMetadata("V", "Analog Voltage A2")));
  */
  // Example of how to output the distance value to Signal K.
  a01->connect_to(
       new SKOutputFloat("sensors.a2.distance", "Analog nig A2",
                         new SKMetadata("m", "Analog nig A2")));

  ///////////////////////////////////////////////////////////////////
  // Digital alarm inputs

  // EDIT: More alarm inputs can be defined by duplicating the lines below.
  // Make sure to not define a pin for both a tacho and an alarm.
  auto alarm_d2_input = ConnectAlarmSender(kDigitalInputPin2, "D2");
  auto alarm_d3_input = ConnectAlarmSender(kDigitalInputPin3, "D3");
  // auto alarm_d4_input = ConnectAlarmSender(kDigitalInputPin4, "D4");

  // Update the alarm states based on the input value changes.
  // EDIT: If you added more alarm inputs, uncomment the respective lines below.
  alarm_d2_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  // In this example, alarm_d3_input is active low, so invert the value.
  auto alarm_d3_inverted = alarm_d3_input->connect_to(
      new LambdaTransform<bool, bool>([](bool value) { return !value; }));
  alarm_d3_inverted->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  // alarm_d4_input->connect_to(
  //     new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));

  // EDIT: This example connects the D2 alarm input to the low oil pressure
  // warning. Modify according to your needs.
  N2kEngineParameterDynamicSender* engine_dynamic_sender =
      new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0,
                                          nmea2000);

  ConfigItem(engine_dynamic_sender)
      ->set_title("Engine 1 Dynamic")
      ->set_description("NMEA 2000 dynamic engine parameters for engine 1")
      ->set_sort_order(3010);

  alarm_d2_input->connect_to(engine_dynamic_sender->low_oil_pressure_);

  // This is just an example -- normally temperature alarms would not be
  // active-low (inverted).
  alarm_d3_inverted->connect_to(engine_dynamic_sender->over_temperature_);

  // FIXME: Transmit the alarms over SK as well.

  ///////////////////////////////////////////////////////////////////
  // Digital tacho inputs

  // Connect the tacho senders. Engine name is "main".
  // EDIT: More tacho inputs can be defined by duplicating the line below.
  auto tacho_d1_frequency = ConnectTachoSender(kDigitalInputPin1, "main");

  // Connect outputs to the N2k senders.
  // EDIT: Make sure this matches your tacho configuration above.
  //       Duplicate the lines below to connect more tachos, but be sure to
  //       use different engine instances.
  N2kEngineParameterRapidSender* engine_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 1 Rapid Update", 0,
                                        nmea2000);  // Engine 1, instance 0

  ConfigItem(engine_rapid_sender)
      ->set_title("Engine 1 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 1")
      ->set_sort_order(3015);

  tacho_d1_frequency->connect_to(&(engine_rapid_sender->engine_speed_));

  if (display_present) {
    tacho_d1_frequency->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 3, "RPM D1", 60 * value); }));
  }

  ///////////////////////////////////////////////////////////////////
  // Display setup

  // Connect the outputs to the display
  if (display_present) {
    event_loop()->onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });

    // Create a poor man's "christmas tree" display for the alarms
    event_loop()->onRepeat(1000, []() {
      char state_string[5] = {};
      for (int i = 0; i < 4; i++) {
        state_string[i] = alarm_states[i] ? '*' : '_';
      }
      PrintValue(display, 4, "Alarm", state_string);
    });
  }

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
