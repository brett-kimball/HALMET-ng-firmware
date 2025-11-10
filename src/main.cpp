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

using namespace sensesp;
using namespace halmet;

// Simple boolean config class
class BoolConfig : public sensesp::FileSystemSaveable {
 public:
  BoolConfig(String config_path, bool default_value = true)
      : sensesp::FileSystemSaveable{config_path}, value{default_value} {}

  bool value;

  virtual bool from_json(const JsonObject& config) override {
    if (config["value"].is<bool>()) {
      value = config["value"];
    }
    return true;
  }

  virtual bool to_json(JsonObject& config) override {
    config["value"] = value;
    return true;
  }

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

} // namespace sensesp

// ========================================================================
// GLOBAL VARIABLES
// ========================================================================
tNMEA2000* nmea2000;
elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

TwoWire* i2c;
Adafruit_SSD1306* display = nullptr;
Adafruit_BNO055* bno055 = nullptr;

// Store alarm states in an array for local display output
bool alarm_states[2] = {false, false};
bool ais_silent = false;
extern int ais_msg_count;

// Raw sensor values for calibration status
std::map<std::string, float> raw_sensor_values;

// --------------------------------------------------------------------
// ADS1115 GAIN CONFIGURATION
// --------------------------------------------------------------------
const adsGain_t kADS1115Gain = GAIN_ONE;

// ========================================================================
// TEST OUTPUT PIN CONFIGURATION
// ========================================================================
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
const int kTestOutputFrequency = 380;
#endif

// ========================================================================
// SETUP FUNCTION
// ========================================================================
void setup() {
  SetupLogging(ESP_LOG_DEBUG);
  Serial.begin(115200);

  // --------------------------------------------------------------------
  // 1. Initialize the application framework
  // --------------------------------------------------------------------
  BUILDER_CLASS builder;
  sensesp_app = (&builder)
                    ->set_hostname("halmet")
                    ->enable_ota("thisisfine")
                    ->get_app();

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

  // --------------------------------------------------------------------
  // 2. Initialize the I2C bus
  // --------------------------------------------------------------------
  i2c = &Wire;
  Wire.begin(kSDAPin, kSCLPin);

  // --------------------------------------------------------------------
  // 3. Initialize ADS1115s — AUTO-DETECT SECOND CHIP
  // --------------------------------------------------------------------
  auto ads1115_0 = new Adafruit_ADS1115();
  ads1115_0->setGain(kADS1115Gain);
  ads1115_0->begin(kADS1115Address_0, i2c);
  debugD("ADS1115_0 (0x48) OK");

  bool ads1115_1_present = false;
  auto ads1115_1 = new Adafruit_ADS1115();
  ads1115_1->setGain(kADS1115Gain);
  if (ads1115_1->begin(kADS1115Address_1, i2c)) {
    debugD("ADS1115_1 (0x49) OK");
    ads1115_1_present = true;
  } else {
    debugE("ADS1115_1 NOT FOUND — oil/temp disabled");
    delete ads1115_1;
    ads1115_1 = nullptr;
  }

  // --------------------------------------------------------------------
  // 3b. Initialize BNO055 Compass
  // --------------------------------------------------------------------
  bno055 = new Adafruit_BNO055(55, 0x28);  // Placeholder address 0x28
  if (bno055->begin()) {
    debugD("BNO055 (0x28) OK");
  } else {
    debugE("BNO055 NOT FOUND - heading disabled");
    delete bno055;
    bno055 = nullptr;
  }

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  ledcAttach(kTestOutputPin, kTestOutputFrequency, 13);
  ledcWrite(0, 4096);  // 50% duty
#endif

  // --------------------------------------------------------------------
  // 4. Initialize NMEA 2000 & AIS functionality
  // --------------------------------------------------------------------
  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);
  nmea2000->SetProductInformation("20231229", 104, "HALMET", "1.0.0", "1.0.0");
  nmea2000->SetDeviceInformation(GetBoardSerialNumber(), 140, 50, 2046);
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, 71);
  nmea2000->EnableForward(false);
  nmea2000->Open();
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  AISGatewayInit();

  // Send command when changed
  event_loop()->onRepeat(1000, []() {
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

  // --------------------------------------------------------------------
  // 5. Initialize the OLED display
  // --------------------------------------------------------------------
  bool display_present = InitializeSSD1306(sensesp_app.get(), &display, i2c);  

  // --------------------------------------------------------------------
  // 6. Analog inputs
  // --------------------------------------------------------------------
  auto enable_calibration_config = new BoolConfig("/Enable Calibration", true);
  ConfigItem(enable_calibration_config)
      ->set_title("Enable Calibration Mode")
      ->set_description("Enable raw Signal K paths for calibration")
      ->set_sort_order(102);

  bool enable_calibration = enable_calibration_config->value;

  halmet::g_enable_calibration = enable_calibration;

  // RUDDER ANGLE — ACTIVE MODE
  auto* a01 = ConnectAnalogSender(
      ads1115_0, 0, RUDDER_ANGLE, "main", "a01", 3000, true, enable_calibration
  );

  // CRUISE CONTROL ANGLE — ACTIVE MODE
  auto* a02 = ConnectAnalogSender(
      ads1115_0, 1, TRIM_ANGLE, "port", "a02", 3005, true, enable_calibration
  );

  // TRANSMISSION GEARS - ACTIVE MODE
  auto* a03 = ConnectAnalogSender(
      ads1115_0, 2, TRANSMISSION_GEAR, "port", "a03", 3010, true, enable_calibration
  );
  auto* a04 = ConnectAnalogSender(
      ads1115_0, 3, TRANSMISSION_GEAR, "stbd", "a04", 3015, true, enable_calibration
  );

  // Optional (ADS1115 #1) — only created if chip exists
  ValueProducer<float>* a11 = nullptr;
  ValueProducer<float>* a12 = nullptr;
  ValueProducer<float>* a13 = nullptr;
  ValueProducer<float>* a14 = nullptr;

  if (ads1115_1_present) {
    a11 = ConnectAnalogSender(
        ads1115_1, 0, PRESSURE, "port", "a11", 3020, true, enable_calibration
    );
    a12 = ConnectAnalogSender(
        ads1115_1, 1, TEMPERATURE, "port", "a12", 3025, true, enable_calibration
    );
    a13 = ConnectAnalogSender(
        ads1115_1, 2, PRESSURE, "stbd", "a13", 3030, true, enable_calibration
    );
    a14 = ConnectAnalogSender(
        ads1115_1, 3, TEMPERATURE, "stbd", "a14", 3035, true, enable_calibration
    );
  }

  // --------------------------------------------------------------------
  // 7. Digital alarm inputs
  // --------------------------------------------------------------------
  auto d03 = ConnectAlarmSender(kDigitalInputPin3, "D3");
  auto d04 = ConnectAlarmSender(kDigitalInputPin4, "D4");

  d03->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[0] = value; })
  );
  d04->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; })
  );

  // --------------------------------------------------------------------
  // 7b. Compass (BNO055)
  // --------------------------------------------------------------------
  ValueProducer<float>* heading_sensor = nullptr;
  ValueProducer<float>* pitch_sensor = nullptr;
  ValueProducer<float>* roll_sensor = nullptr;
  if (bno055) {
    heading_sensor = new sensesp::RepeatSensor<float>(
        100,  // 100ms interval
        []() {
          sensors_event_t event;
          bno055->getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
          return event.orientation.x;  // Heading in degrees
        }
    );
    pitch_sensor = new sensesp::RepeatSensor<float>(
        100,  // 100ms interval
        []() {
          sensors_event_t event;
          bno055->getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
          return event.orientation.y;  // Pitch in degrees
        }
    );
    roll_sensor = new sensesp::RepeatSensor<float>(
        100,  // 100ms interval
        []() {
          sensors_event_t event;
          bno055->getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
          return event.orientation.z;  // Roll in degrees
        }
    );
  }

  // --------------------------------------------------------------------
  // 8. NMEA 2000 Engine Dynamic Senders
  // --------------------------------------------------------------------
  N2kEngineParameterDynamicSender* engine_1_dynamic_sender =
      new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0, nmea2000);
  ConfigItem(engine_1_dynamic_sender)
      ->set_title("Engine 1 Dynamic")
      ->set_description("NMEA 2000 dynamic engine parameters for engine 1")
      ->set_sort_order(2000);

  N2kEngineParameterDynamicSender* engine_2_dynamic_sender =
      new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 2 Dynamic", 1, nmea2000);
  ConfigItem(engine_2_dynamic_sender)
      ->set_title("Engine 2 Dynamic")
      ->set_description("NMEA 2000 dynamic engine parameters for engine 2")
      ->set_sort_order(2100);

  if (a12) {
    // Port Coolant Temperature
    a12->connect_to(
      new sensesp::LambdaTransform<float, double>([](float f) {
        return (f - 32.0) * 5.0 / 9.0 + 273.15;
      })
    )->connect_to(engine_1_dynamic_sender->temperature_);
  }

  if (a14) {
    // Stbd Coolant Temperature
    a14->connect_to(
      new sensesp::LambdaTransform<float, double>([](float f) {
        return (f - 32.0) * 5.0 / 9.0 + 273.15;
      })
    )->connect_to(engine_2_dynamic_sender->temperature_);
  }

  if (a11) {
    // Port Oil Pressure
    a11->connect_to(
      new sensesp::LambdaTransform<float, double>([](float psi) { return psi * 6894.76; })
    )->connect_to(engine_1_dynamic_sender->oil_pressure_);
  }

  if (a13) {
    // Stbd Oil Pressure
    a13->connect_to(
      new sensesp::LambdaTransform<float, double>([](float psi) { return psi * 6894.76; })
    )->connect_to(engine_2_dynamic_sender->oil_pressure_);
  }

  // Port Low Oil Pressure Alarm
  d03->connect_to(engine_1_dynamic_sender->low_oil_pressure_);
  // Stbd Low Oil Pressure Alarm
  d04->connect_to(engine_2_dynamic_sender->low_oil_pressure_);

  // --------------------------------------------------------------------
  // 9. Digital tacho inputs
  // --------------------------------------------------------------------
  auto d01 = ConnectTachoSender(kDigitalInputPin1, "port");
  auto d02 = ConnectTachoSender(kDigitalInputPin2, "stbd");

  N2kEngineParameterRapidSender* engine_1_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 1 Rapid Update", 0, nmea2000);
  ConfigItem(engine_1_rapid_sender)
      ->set_title("Engine 1 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 1")
      ->set_sort_order(2500);
  // Port Engine RPM
  d01->connect_to(&(engine_1_rapid_sender->engine_speed_));

  N2kEngineParameterRapidSender* engine_2_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 2 Rapid Update", 1, nmea2000);
  ConfigItem(engine_2_rapid_sender)
      ->set_title("Engine 2 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 2")
      ->set_sort_order(2505);
  // Stbd Engine RPM
  d02->connect_to(&(engine_2_rapid_sender->engine_speed_));

  // --------------------------------------------------------------------
  // 10. NMEA 2000 Rudder Sender
  // --------------------------------------------------------------------
  N2kRudderSender* rudder_sender = new N2kRudderSender("/NMEA 2000/Rudder", 0, nmea2000);
  // Rudder Angle
  a01->connect_to(&rudder_sender->rudder_angle_deg_);

  // --------------------------------------------------------------------
  // 11. NMEA 2000 Trim Tabs
  // --------------------------------------------------------------------
  N2kTrimTabSender* trim_tab_sender = new N2kTrimTabSender("/NMEA 2000/Trim Tabs", nmea2000);
  ConfigItem(trim_tab_sender)
      ->set_title("Trim Tabs NMEA 2000")
      ->set_sort_order(3006);
  a02->connect_to(&trim_tab_sender->trim_deg_port_);
  a02->connect_to(&trim_tab_sender->trim_deg_stbd_);

  // ========================================================================
  // 13. OLED — WORKS WITH OR WITHOUT OLED, !!STATIC OR BUST!!
  // ========================================================================
if (display_present && display) {
// === ROW 0: WiFi Status / IP Address / AIS (rotates every 5 seconds) ===
static int display_state = 0;
event_loop()->onRepeat(5000, []() {
  if (WiFi.status() == WL_CONNECTED) {
    if (display_state == 0) {
      // SSID + signal
      String ssid = WiFi.SSID();
      ssid.replace("°", ".");
      String signal = String(WiFi.RSSI()) + "dBm";
      PrintValue(display, 0, ssid, signal, "");
    } else if (display_state == 1) {
      // IP address
      String ip = WiFi.localIP().toString();
      PrintValue(display, 0, ip, "", "");
    } else {
      // AIS counts
      extern int ais_msg_count_a, ais_msg_count_b;
      String ais_a = String(ais_msg_count_a) + "a";
      String ais_b = String(ais_msg_count_b) + "b";
      PrintValue(display, 0, "AIS", ais_a, ais_b);
    }
    display_state = (display_state + 1) % 3;
  } else {
    PrintValue(display, 0, "WiFi Offline", "", "");
    display_state = 0;  // reset when offline
  }
});

// === ROW 1: ALARM or IP + HEADERS ===
event_loop()->onRepeat(1000, []() {
  bool any = false;
  for (int i = 0; i < 2; i++) if (alarm_states[i]) any = true;

  if (any) {
    PrintValue(display, 1, "<ALARM>", "Port", "Stbd");
  } else {
    PrintValue(display, 1, "", "Port", "Stbd");
  }
});

  // === ROW 2: RPM ===
  static String rpm_l = "----", rpm_r = "----";
  auto update_rpm = [&]() {
    PrintValue(display, 2, "RPM", rpm_l, rpm_r);
  };
  // Port Engine RPM
  d01->connect_to(new LambdaConsumer<float>([&](float v) {
    rpm_l = String((int)(60 * v));
    update_rpm();
  }));
  // Stbd Engine RPM
  d02->connect_to(new LambdaConsumer<float>([&](float v) {
    rpm_r = String((int)(60 * v));
    update_rpm();
  }));

  // === ROW 3: Oil ===
  static String oil_l = "--", oil_r = "--";
  auto update_oil = [&]() {
    String p = oil_l + "p";
    String s = oil_r + "p";
    PrintValue(display, 3, "Oil", p, s);
  };
  if (a11) {
    // Port Oil Pressure
    a11->connect_to(new LambdaConsumer<float>([&](float v){
      oil_l = String((int)v);
      update_oil();
    }));
  }
  if (a13) {
    // Stbd Oil Pressure
    a13->connect_to(new LambdaConsumer<float>([&](float v){
      oil_r = String((int)v);
      update_oil();
    }));
  }
  if (!a11 && !a13) {
    PrintValue(display, 3, "Oil", "--p", "--p");
  }

  // === ROW 4: Temp ===
  static String temp_l = "---", temp_r = "---";
  auto update_temp = [&]() {
    String p = temp_l + "f";
    String s = temp_r + "f";
    PrintValue(display, 4, "Temp", p, s);
  };
  if (a12) {
    // Port Coolant Temperature
    a12->connect_to(new LambdaConsumer<float>([&](float v){
      temp_l = String((int)v);
      update_temp();
    }));
  }
  if (a14) {
    // Stbd Coolant Temperature
    a14->connect_to(new LambdaConsumer<float>([&](float v){
      temp_r = String((int)v);
      update_temp();
    }));
  }
  if (!a12 && !a14) {
    PrintValue(display, 4, "Temp", "---f", "---f");
  }

  // === ROW 5: Gear ===
  static String gear_l = "N", gear_r = "N";
  auto update_gear = [&]() {
    PrintValue(display, 5, "Gear", gear_l, gear_r);
  };
  // Port Transmission Gear
  a03->connect_to(new LambdaConsumer<float>([&](float v) {
    gear_l = (v < 0.25f) ? "R" : ((v < 0.75f) ? "N" : "F");
    update_gear();
  }));
  // Stbd Transmission Gear
  a04->connect_to(new LambdaConsumer<float>([&](float v) {
    gear_r = (v < 0.25f) ? "R" : ((v < 0.75f) ? "N" : "F");
    update_gear();
  }));

  // === ROW 6: Rd/Tr ===
  static String rud = "---", trm = "---";
  auto update_rudtrm = [&]() {
    PrintValue(display, 6, "Rd/Tr", rud, trm);
  };
  // Rudder Angle
  a01->connect_to(new LambdaConsumer<float>([&](float v) {
    rud = String((int)v);
    update_rudtrm();
  }));
  // Trim Angle
  a02->connect_to(new LambdaConsumer<float>([&](float v) {
    trm = String((int)v);
    update_rudtrm();
  }));

  // === ROW 7: Heading ===
  if (heading_sensor) {
    static String heading_str = "---";
    auto update_heading = [&]() {
      PrintValue(display, 7, "HDG", heading_str, "");
    };
    heading_sensor->connect_to(new LambdaConsumer<float>([&](float v) {
      heading_str = String((int)v);
      update_heading();
    }));
  } else {
    PrintValue(display, 7, "HDG", "---", "");
  }
}

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
