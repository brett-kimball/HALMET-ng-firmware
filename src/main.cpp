#include <Adafruit_ADS1X15.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NMEA2000_esp32.h>
#include <map>
#include <string>

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
#include "sensesp/ui/status_page_item.h"
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "ais_gateway.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"
#include "SPIFFS.h"
#include "sensesp/system/observablevalue.h"

using namespace sensesp;
using namespace halmet;

// Simple boolean config class for calibration mode toggle
class BoolConfig : public sensesp::FileSystemSaveable {
 public:
  BoolConfig(String config_path, bool default_value = true)
      : sensesp::FileSystemSaveable{config_path}, value{default_value} {}

  bool value;

  // Deserialize from JSON (used for loading saved configuration)
  virtual bool from_json(const JsonObject& config) override {
    if (config["value"].is<bool>()) {
      value = config["value"];
    }
    return true;
  }

  // Serialize to JSON (used for saving configuration)
  virtual bool to_json(JsonObject& config) override {
    config["value"] = value;
    return true;
  }

  // JSON schema for web UI configuration form
  const String ConfigSchema(const BoolConfig& obj) {
    return R"###({
      "type": "object",
      "properties": {
        "value": {
          "title": "Enabled",
          "type": "boolean",
          "description": "Boolean value"
        }
      }
    })###";
  }
};

// Global ConfigSchema function for SensESP template system
namespace sensesp {

const String ConfigSchema(const BoolConfig& obj) {
  return R"###({
    "type": "object",
    "properties": {
      "value": {
        "title": "Enabled",
        "type": "boolean",
        "description": "Boolean value"
      }
    }
  })###";
}

}  // namespace sensesp

// ========================================================================
// GLOBAL VARIABLES
// ========================================================================
tNMEA2000* nmea2000;
elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

TwoWire* i2c;
Adafruit_SSD1306* display = nullptr;
Adafruit_BNO055* bno055 = nullptr;
Adafruit_ADS1115* ads1115_0 = nullptr;
Adafruit_ADS1115* ads1115_1 = nullptr;
bool ads1115_1_present = false;

// Store alarm states in an array for local display output
bool alarm_states[2] = {false, false};
bool ais_silent = false;
extern int ais_msg_count_a, ais_msg_count_b;

// Uptime tracking
elapsedMillis system_uptime_ms = false;

// Raw sensor values for calibration status
namespace halmet {
std::map<std::string, float> raw_sensor_values;

// Status page items for displaying raw sensor values on web UI
std::map<std::string, CalibrationStatusPageItem<float>*> raw_sensor_status_items;
}

// ========================================================================
// SPIFFS MAINTENANCE FUNCTIONS
// ========================================================================

void PerformSPIFFSCleanup() {
  // ------------------------------------------------------------------
  // SPIFFS Cleanup / Format helpers (INACTIVE by default)
  // ------------------------------------------------------------------
  // See docs/cleanup-and-session.md for detailed usage instructions.
  // This code implements compile-time guarded SPIFFS maintenance actions.
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
  if (SPIFFS.begin(true)) {
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
  } else {
    Serial.println("ERROR: SPIFFS.begin() failed for cleanup");
  }
#endif

  // One-shot SPIFFS FORMAT (destructive) - formats the filesystem completely
  // and writes /persist_format_done so it does not repeat. Controlled via
  // the FORCE_SPIFFS_FORMAT compile-time flag. This is more aggressive than
  // the file-by-file cleanup above and will erase everything in SPIFFS.
#if defined(FORCE_SPIFFS_FORMAT)
  if (SPIFFS.begin(true) && !SPIFFS.exists("/persist_format_done")) {
    Serial.println("FORCED: formatting SPIFFS (FORCE_SPIFFS_FORMAT enabled)");
    bool ok = SPIFFS.format();
    Serial.printf("SPIFFS.format() returned: %d\n", ok);
    File mf2 = SPIFFS.open("/persist_format_done", FILE_WRITE);
    if (mf2) { mf2.printf("done\n"); mf2.close(); Serial.println("persist_format_done marker written"); }
  } else {
    Serial.println("persist_format_done marker present — skipping format");
  }
#endif
}

// ========================================================================
// HARDWARE INITIALIZATION FUNCTIONS
// ========================================================================

void InitializeI2CBus() {
  i2c = &Wire;
  Wire.begin(kSDAPin, kSCLPin);
}

void InitializeADS1115s(Adafruit_ADS1115*& ads1115_0, Adafruit_ADS1115*& ads1115_1, bool& ads1115_1_present) {
  ads1115_0 = new Adafruit_ADS1115();
  ads1115_0->setGain(kADS1115Gain);
  ads1115_0->begin(kADS1115Address_0, i2c);
  debugD("ADS1115_0 (0x48) OK");

  ads1115_1_present = false;
  ads1115_1 = new Adafruit_ADS1115();
  ads1115_1->setGain(kADS1115Gain);
  if (ads1115_1->begin(kADS1115Address_1, i2c)) {
    debugD("ADS1115_1 (0x49) OK");
    ads1115_1_present = true;
  } else {
    debugE("ADS1115_1 NOT FOUND — oil/temp disabled");
    delete ads1115_1;
    ads1115_1 = nullptr;
  }
}

void InitializeCompass(Adafruit_BNO055*& bno055) {
  bno055 = new Adafruit_BNO055(55, kBNO055Address);
  if (bno055->begin()) {
    debugD("BNO055 (0x28) OK");
  } else {
    debugE("BNO055 NOT FOUND - heading disabled");
    delete bno055;
    bno055 = nullptr;
  }
}

// ========================================================================
// AIS GATEWAY FUNCTIONS
// ========================================================================

void InitializeAISGateway() {
  Serial2.begin(38400, SERIAL_8N1, kSerial2RxPin, kSerial2TxPin);
  debugD("Serial2 (AIS) initialized");

  // AIS Silent Mode Toggle
  auto ais_silent_config = new BoolConfig("/AIS Silent Mode", false);
  ConfigItem(ais_silent_config)
      ->set_title("AIS Silent Mode")
      ->set_description("Enable AIS silent mode (receive-only)")
      ->set_sort_order(1000);

  ais_silent_config->load();
  ais_silent = ais_silent_config->value;

  // Apply initial AIS silent mode setting
  const char* cmd = ais_silent ?
      "$PSRT,TRG,02,33" : "$PSRT,TRG,02,00";
  AISSendCommand(cmd);
  Serial.printf("AIS Silent Mode: %s\n", ais_silent ? "ON" : "OFF");

  // Handle AIS silent mode changes
  event_loop()->onRepeat(1000, [ais_silent_config]() {
    static bool last_mode = !ais_silent;
    if (last_mode != ais_silent) {
      const char* cmd = ais_silent ?
          "$PSRT,TRG,02,33" : "$PSRT,TRG,02,00";
      AISSendCommand(cmd);
      last_mode = ais_silent;
      Serial.printf("AIS Silent Mode: %s\n", ais_silent ? "ON" : "OFF");
    }
  });

  // AISResetFactory();               // factory reset
  // AISSetMMSI(123456789);           // set your MMSI
  // AISSendCommand("$PSRT,TRG,02,00"); // disable silent mode
}

// ========================================================================
// DISPLAY FUNCTIONS
// ========================================================================

void UpdateRPMDisplay();
void UpdateOilDisplay();
void UpdateTempDisplay();
void UpdateGearDisplay();
void UpdateRudTrimDisplay();
void UpdateHeadingDisplay();

void InitializeDisplay() {
  bool display_present = InitializeSSD1306(sensesp_app.get(), &display, i2c);
  if (display_present && display) {
    // WiFi Status / IP Address / AIS / Uptime rotation (every 5 seconds)
    static int display_state = 0;
    event_loop()->onRepeat(5000, []() {
      if (display_state == 0) {
        // WiFi status (SSID + signal if connected, or offline message)
        if (WiFi.status() == WL_CONNECTED) {
          String ssid = WiFi.SSID();
          ssid.replace("°", "o");
          String clean_ssid = "";
          for (char c : ssid) {
            if (c >= 32 && c <= 126) {
              clean_ssid += c;
            } else {
              clean_ssid += '?';
            }
          }
          String signal = String(WiFi.RSSI()) + "dBm";
          PrintValue(display, 0, clean_ssid, signal, "");
        } else {
          PrintValue(display, 0, "WiFi Offline", "", "");
        }
      } else if (display_state == 1) {
        // IP address
        if (WiFi.status() == WL_CONNECTED && WiFi.localIP() != IPAddress(0,0,0,0)) {
          String ip = WiFi.localIP().toString();
          PrintValue(display, 0, "IP", ip, "");
        } else {
          PrintValue(display, 0, "IP", "Not Connected", "");
        }
      } else if (display_state == 2) {
        // AIS counts
        extern int ais_msg_count_a, ais_msg_count_b;
        String ais_a = String(ais_msg_count_a) + "a";
        String ais_b = String(ais_msg_count_b) + "b";
        PrintValue(display, 0, "AIS", ais_a, ais_b);
      } else {
        // Uptime
        unsigned long uptime_seconds = system_uptime_ms / 1000;
        unsigned long days = uptime_seconds / 86400;
        unsigned long hours = (uptime_seconds % 86400) / 3600;
        unsigned long minutes = (uptime_seconds % 3600) / 60;
        
        String uptime_str;
        if (days > 0) {
          uptime_str = String(days) + "d " + String(hours) + "h";
        } else if (hours > 0) {
          uptime_str = String(hours) + "h " + String(minutes) + "m";
        } else {
          uptime_str = String(minutes) + "m " + String(uptime_seconds % 60) + "s";
        }
        
        PrintValue(display, 0, "Uptime", uptime_str, "");
      }
      display_state = (display_state + 1) % 4;
    });

    // Line 1: Alarm status with Port/Stbd headers
    event_loop()->onRepeat(1000, []() {
      bool any_alarm = alarm_states[0] || alarm_states[1];
      if (any_alarm) {
        PrintValue(display, 1, "<ALARM>", "Port", "Stbd");
      } else {
        PrintValue(display, 1, "", "Port", "Stbd");
      }
    });

    // Initialize display with default values
    UpdateRPMDisplay();
    UpdateOilDisplay();
    UpdateTempDisplay();
    UpdateGearDisplay();
    UpdateRudTrimDisplay();
    UpdateHeadingDisplay();
  }
}

// ========================================================================
// DISPLAY UPDATE FUNCTIONS
// ========================================================================

static String rpm_l = "----", rpm_r = "----";
void UpdateRPMDisplay() {
  if (display) PrintValue(display, 2, "RPM", rpm_l, rpm_r);
}

static String oil_l = "--", oil_r = "--";
void UpdateOilDisplay() {
  if (display) {
    String p = oil_l + "p";
    String s = oil_r + "p";
    PrintValue(display, 3, "Oil", p, s);
  }
}

static String temp_l = "---", temp_r = "---";
void UpdateTempDisplay() {
  if (display) {
    String p = temp_l + "f";
    String s = temp_r + "f";
    PrintValue(display, 4, "Temp", p, s);
  }
}

static String gear_l = "N", gear_r = "N";
void UpdateGearDisplay() {
  if (display) PrintValue(display, 5, "Gear", gear_l, gear_r);
}

static String rud = "---", trm = "---";
void UpdateRudTrimDisplay() {
  if (display) PrintValue(display, 6, "Rd/Tr", rud, trm);
}

static String heading_str = "---";
void UpdateHeadingDisplay() {
  if (display) PrintValue(display, 7, "Hdg", heading_str, "");
}

// ========================================================================
// SENSOR SETUP FUNCTIONS
// ========================================================================

void ConnectSensorsToNMEA2000(
    ValueProducer<float>* a01, ValueProducer<float>* a02, ValueProducer<float>* a03, ValueProducer<float>* a04,
    ValueProducer<float>* a11, ValueProducer<float>* a12, ValueProducer<float>* a13, ValueProducer<float>* a14,
    ValueProducer<float>* d01, ValueProducer<float>* d02, BoolProducer* d03, BoolProducer* d04
);

void SetupAllSensors(Adafruit_ADS1115* ads1115_0, Adafruit_ADS1115* ads1115_1, bool ads1115_1_present) {
  // Calibration mode configuration
  auto enable_calibration_config = new BoolConfig("/Enable Calibration", true);
  ConfigItem(enable_calibration_config)
      ->set_title("Enable Calibration Mode")
      ->set_description("Enable raw Signal K paths for calibration")
      ->set_sort_order(102);

  enable_calibration_config->load();
  bool enable_calibration = enable_calibration_config->value;
  halmet::g_enable_calibration = enable_calibration;

  debugI("Calibration mode setting: %s (value=%d)", enable_calibration ? "ENABLED" : "DISABLED", enable_calibration);

  // First ADS1115 sensors (active - resistive senders)
  // Rudder angle sensor
  auto* a01 = ConnectAnalogSender(
      ads1115_0, 0, RUDDER_ANGLE, "main", "a01", 3000, true, enable_calibration
  );
  a01->connect_to(new LambdaConsumer<float>([](float v) {
    rud = String((int)v);
    UpdateRudTrimDisplay();
  }));

  // Trim angle sensor
  auto* a02 = ConnectAnalogSender(
      ads1115_0, 1, TRIM_ANGLE, "port", "a02", 3005, true, enable_calibration
  );
  a02->connect_to(new LambdaConsumer<float>([](float v) {
    trm = String((int)v);
    UpdateRudTrimDisplay();
  }));

  // Transmission gear sensors
  auto* a03 = ConnectAnalogSender(
      ads1115_0, 2, TRANSMISSION_GEAR, "port", "a03", 3010, true, enable_calibration
  );
  a03->connect_to(new LambdaConsumer<float>([](float v) {
    gear_l = (v < 0.25f) ? "R" : ((v < 0.75f) ? "N" : "F");
    UpdateGearDisplay();
  }));

  auto* a04 = ConnectAnalogSender(
      ads1115_0, 3, TRANSMISSION_GEAR, "stbd", "a04", 3015, true, enable_calibration
  );
  a04->connect_to(new LambdaConsumer<float>([](float v) {
    gear_r = (v < 0.25f) ? "R" : ((v < 0.75f) ? "N" : "F");
    UpdateGearDisplay();
  }));

  // Second ADS1115 sensors (passive - voltage output)
  ValueProducer<float>* a11 = nullptr;
  ValueProducer<float>* a12 = nullptr;
  ValueProducer<float>* a13 = nullptr;
  ValueProducer<float>* a14 = nullptr;

  if (ads1115_1_present) {
    // Port engine oil pressure
    a11 = ConnectAnalogSender(
        ads1115_1, 0, PRESSURE, "port", "a11", 3020, true, enable_calibration
    );
    a11->connect_to(new LambdaConsumer<float>([](float v) {
      oil_l = String((int)v);
      UpdateOilDisplay();
    }));

    // Port engine coolant temperature
    a12 = ConnectAnalogSender(
        ads1115_1, 1, TEMPERATURE, "port", "a12", 3025, true, enable_calibration
    );
    a12->connect_to(new LambdaConsumer<float>([](float v) {
      temp_l = String((int)v);
      UpdateTempDisplay();
    }));

    // Starboard engine oil pressure
    a13 = ConnectAnalogSender(
        ads1115_1, 2, PRESSURE, "stbd", "a13", 3030, true, enable_calibration
    );
    a13->connect_to(new LambdaConsumer<float>([](float v) {
      oil_r = String((int)v);
      UpdateOilDisplay();
    }));

    // Starboard engine coolant temperature
    a14 = ConnectAnalogSender(
        ads1115_1, 3, TEMPERATURE, "stbd", "a14", 3035, true, enable_calibration
    );
    a14->connect_to(new LambdaConsumer<float>([](float v) {
      temp_r = String((int)v);
      UpdateTempDisplay();
    }));
  }

  // Digital sensors
  // Port engine low oil pressure alarm
  auto d03 = ConnectAlarmSender(kDigitalInputPin3, "D3");
  d03->connect_to(new LambdaConsumer<bool>([](bool value) { 
    alarm_states[0] = value; 
  }));

  // Starboard engine low oil pressure alarm
  auto d04 = ConnectAlarmSender(kDigitalInputPin4, "D4");
  d04->connect_to(new LambdaConsumer<bool>([](bool value) { 
    alarm_states[1] = value; 
  }));

  // Port engine RPM
  auto d01 = ConnectTachoSender(kDigitalInputPin1, "port");
  d01->connect_to(new LambdaConsumer<float>([](float v) {
    rpm_l = String((int)(60 * v));
    UpdateRPMDisplay();
  }));

  // Starboard engine RPM
  auto d02 = ConnectTachoSender(kDigitalInputPin2, "stbd");
  d02->connect_to(new LambdaConsumer<float>([](float v) {
    rpm_r = String((int)(60 * v));
    UpdateRPMDisplay();
  }));

  // Connect sensors to NMEA 2000
  ConnectSensorsToNMEA2000(a01, a02, a03, a04, a11, a12, a13, a14, d01, d02, d03, d04);

  // Heading sensor (if compass is available)
  if (bno055) {
    ValueProducer<float>* heading_sensor = new sensesp::RepeatSensor<float>(
        100,  // 100ms interval
        []() {
          sensors_event_t event;
          bno055->getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
          return event.orientation.x;  // Heading in degrees
        }
    );
    heading_sensor->connect_to(new LambdaConsumer<float>([](float v) {
      heading_str = String((int)v);
      UpdateHeadingDisplay();
    }));
  } else {
    UpdateHeadingDisplay(); // Shows "---"
  }
}

void ConnectSensorsToNMEA2000(
    ValueProducer<float>* a01, ValueProducer<float>* a02, ValueProducer<float>* a03, ValueProducer<float>* a04,
    ValueProducer<float>* a11, ValueProducer<float>* a12, ValueProducer<float>* a13, ValueProducer<float>* a14,
    ValueProducer<float>* d01, ValueProducer<float>* d02, BoolProducer* d03, BoolProducer* d04
) {
  // Rudder angle sender
  N2kRudderSender* rudder_sender = new N2kRudderSender("/NMEA 2000/Rudder", 0, nmea2000);
  a01->connect_to(new sensesp::LambdaTransform<float, double>([](float deg) { return deg * PI / 180.0; }))
      ->connect_to(rudder_sender->rudder_angle_deg_);

  // Transmission senders
  N2kTransmissionSender* transmission_1_sender =
      new N2kTransmissionSender("/NMEA 2000/Port Transmission", 0, nmea2000);

  N2kTransmissionSender* transmission_2_sender =
      new N2kTransmissionSender("/NMEA 2000/Stbd Transmission", 1, nmea2000);

  // Connect transmission sensors
  a03->connect_to(
    new sensesp::LambdaTransform<float, int>([](float gear_pos) {
      if (gear_pos < 0.25f) return 0;  // Reverse
      else if (gear_pos < 0.75f) return 1;  // Neutral
      else return 2;  // Forward
    })
  )->connect_to(transmission_1_sender->gear_);

  a04->connect_to(
    new sensesp::LambdaTransform<float, int>([](float gear_pos) {
      if (gear_pos < 0.25f) return 0;  // Reverse
      else if (gear_pos < 0.75f) return 1;  // Neutral
      else return 2;  // Forward
    })
  )->connect_to(transmission_2_sender->gear_);

  // Engine parameter senders (if second ADS1115 is present)
  if (a11 && a13) {
    N2kEngineParameterDynamicSender* engine_1_dynamic_sender =
        new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0, nmea2000);
    ConfigItem(engine_1_dynamic_sender)
        ->set_title("Engine 1 Dynamic Parameters")
        ->set_description("NMEA 2000 dynamic engine parameters for engine 1")
        ->set_sort_order(2000);

    N2kEngineParameterDynamicSender* engine_2_dynamic_sender =
        new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 2 Dynamic", 1, nmea2000);
    ConfigItem(engine_2_dynamic_sender)
        ->set_title("Engine 2 Dynamic Parameters")
        ->set_description("NMEA 2000 dynamic engine parameters for engine 2")
        ->set_sort_order(2005);

    // Connect oil pressure sensors
    a11->connect_to(
      new sensesp::LambdaTransform<float, double>([](float psi) { return psi * 6894.76; })
    )->connect_to(engine_1_dynamic_sender->oil_pressure_);

    a13->connect_to(
      new sensesp::LambdaTransform<float, double>([](float psi) { return psi * 6894.76; })
    )->connect_to(engine_2_dynamic_sender->oil_pressure_);

    // Connect temperature sensors (coolant temperature)
    a12->connect_to(engine_1_dynamic_sender->temperature_);
    a14->connect_to(engine_2_dynamic_sender->temperature_);

    // Connect alarm sensors
    d03->connect_to(engine_1_dynamic_sender->low_oil_pressure_);
    d04->connect_to(engine_2_dynamic_sender->low_oil_pressure_);
  }

  // RPM senders (rapid update)
  N2kEngineParameterRapidSender* engine_1_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Port Engine Rapid Update", 0, nmea2000);

  N2kEngineParameterRapidSender* engine_2_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Stbd Engine Rapid Update", 1, nmea2000);

  // Connect RPM sensors
  d01->connect_to(&(engine_1_rapid_sender->engine_speed_));
  d02->connect_to(&(engine_2_rapid_sender->engine_speed_));
}

// ========================================================================
// NMEA 2000 FUNCTIONS
// ========================================================================

void InitializeNMEA2000() {
  // Initialize NMEA 2000 interface
  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);
  nmea2000->SetProductInformation("20231229", 104, "HALMET", "1.0.0", "1.0.0");
  nmea2000->SetDeviceInformation(1, 140, 50, 2046);
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 71);
  nmea2000->EnableForward(false);
  nmea2000->Open();
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });
  debugD("NMEA 2000 initialized");
}

// ========================================================================
// MAIN SETUP FUNCTION
// ========================================================================

void setup() {
  SetupLogging(ESP_LOG_DEBUG);
  Serial.begin(115200);

  // Initialize the application framework
  BUILDER_CLASS builder;
  sensesp_app = (&builder)
                    ->set_hostname("halmet")
                    ->enable_ota("thisisfine")
                    ->get_app();

  // SPIFFS maintenance (compile-time controlled)
  PerformSPIFFSCleanup();

  // Hardware initialization
  InitializeI2CBus();
  InitializeADS1115s(ads1115_0, ads1115_1, ads1115_1_present);
  InitializeCompass(bno055);

  // Communication systems
  InitializeAISGateway();
  InitializeNMEA2000();

  // User interface
  InitializeDisplay();

  // Sensors and NMEA 2000 connections
  SetupAllSensors(ads1115_0, ads1115_1, ads1115_1_present);

  // NMEA 2000 data transmission is handled within SetupAllSensors

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

// ========================================================================
// MAIN LOOP
// ========================================================================
void loop() {
  event_loop()->tick();
  AISGatewayLoop();               // <-- processes Serial2
}
