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

using namespace sensesp;
using namespace halmet;

// ========================================================================
// GLOBAL VARIABLES
// ========================================================================
tNMEA2000* nmea2000;
elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

TwoWire* i2c;
Adafruit_SSD1306* display = nullptr;

// Store alarm states in an array for local display output
bool alarm_states[2] = {false, false};
bool ais_silent = false;
extern int ais_msg_count;

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
                    ->set_hostname("halmet1")
                    ->enable_ota("thisisfine")
                    ->get_app();

  // --------------------------------------------------------------------
  // 2. Initialize the I2C bus
  // --------------------------------------------------------------------
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

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
  bool enable_signalk_output = true;

  // RUDDER ANGLE — ACTIVE MODE
  auto* rudder_deg = ConnectRudderAngleSender(
      ads1115_0, 0, "Rudder", "main", 3000, true
  );

  // CRUISE CONTROL ANGLE — ACTIVE MODE
  auto* trim_deg = ConnectTrimAngleSender(
      ads1115_0, 1, "Cruise Control", "main", 3005, true
  );

  // Port Transmission
  auto* port_gear_position = ConnectTransmissionGearSender(
      ads1115_0, 2, "Port Transmission", "port", 3010, true
  );

  // Stbd Transmission
  auto* stbd_gear_position = ConnectTransmissionGearSender(
      ads1115_0, 3, "Stbd Transmission", "stbd", 3015, true
  );

  // Optional (ADS1115 #1) — only created if chip exists
  ValueProducer<float>* port_oil_psi = nullptr;
  ValueProducer<float>* port_temp_f  = nullptr;
  ValueProducer<float>* stbd_oil_psi = nullptr;
  ValueProducer<float>* stbd_temp_f  = nullptr;

  if (ads1115_1_present) {
    port_oil_psi = ConnectPressureSender(
        ads1115_1, 0, "Port Oil", "port", 3020, true
    );
    port_temp_f  = ConnectTemperatureSender(
        ads1115_1, 1, "Port Coolant", "port", 3025, true
    );
    stbd_oil_psi = ConnectPressureSender(
        ads1115_1, 2, "Stbd Oil", "stbd", 3030, true
    );
    stbd_temp_f  = ConnectTemperatureSender(
        ads1115_1, 3, "Stbd Coolant", "stbd", 3035, true
    );
  }

  // --------------------------------------------------------------------
  // 7. Digital alarm inputs
  // --------------------------------------------------------------------
  auto alarm_d3_input = ConnectAlarmSender(kDigitalInputPin3, "D3");
  auto alarm_d4_input = ConnectAlarmSender(kDigitalInputPin4, "D4");

  alarm_d3_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[0] = value; })
  );
  alarm_d4_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; })
  );

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

  if (port_temp_f) {
    port_temp_f->connect_to(
      new sensesp::LambdaTransform<float, double>([](float f) {
        return (f - 32.0) * 5.0 / 9.0 + 273.15;
      })
    )->connect_to(engine_1_dynamic_sender->temperature_);
  }

  if (stbd_temp_f) {
    stbd_temp_f->connect_to(
      new sensesp::LambdaTransform<float, double>([](float f) {
        return (f - 32.0) * 5.0 / 9.0 + 273.15;
      })
    )->connect_to(engine_2_dynamic_sender->temperature_);
  }

  if (port_oil_psi) {
    port_oil_psi->connect_to(
      new sensesp::LambdaTransform<float, double>([](float psi) { return psi * 6894.76; })
    )->connect_to(engine_1_dynamic_sender->oil_pressure_);
  }

  if (stbd_oil_psi) {
    stbd_oil_psi->connect_to(
      new sensesp::LambdaTransform<float, double>([](float psi) { return psi * 6894.76; })
    )->connect_to(engine_2_dynamic_sender->oil_pressure_);
  }

  alarm_d3_input->connect_to(engine_1_dynamic_sender->low_oil_pressure_);
  alarm_d4_input->connect_to(engine_2_dynamic_sender->low_oil_pressure_);

  // --------------------------------------------------------------------
  // 9. Digital tacho inputs
  // --------------------------------------------------------------------
  auto tacho_d1_frequency = ConnectTachoSender(kDigitalInputPin1, "port");
  auto tacho_d2_frequency = ConnectTachoSender(kDigitalInputPin2, "stbd");

  N2kEngineParameterRapidSender* engine_1_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 1 Rapid Update", 0, nmea2000);
  ConfigItem(engine_1_rapid_sender)
      ->set_title("Engine 1 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 1")
      ->set_sort_order(2500);
  tacho_d1_frequency->connect_to(&(engine_1_rapid_sender->engine_speed_));

  N2kEngineParameterRapidSender* engine_2_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 2 Rapid Update", 1, nmea2000);
  ConfigItem(engine_2_rapid_sender)
      ->set_title("Engine 2 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 2")
      ->set_sort_order(2505);
  tacho_d2_frequency->connect_to(&(engine_2_rapid_sender->engine_speed_));

  // --------------------------------------------------------------------
  // 10. NMEA 2000 Rudder Sender
  // --------------------------------------------------------------------
  N2kRudderSender* rudder_sender = new N2kRudderSender("/NMEA 2000/Rudder", 0, nmea2000);
  rudder_deg->connect_to(&rudder_sender->rudder_angle_deg_);

  // --------------------------------------------------------------------
  // 11. NMEA 2000 Trim Tabs (both port & starboard = same value)
  // --------------------------------------------------------------------
  N2kTrimTabSender* trim_tab_sender = new N2kTrimTabSender("/NMEA 2000/Trim Tabs", nmea2000);
  ConfigItem(trim_tab_sender)
      ->set_title("Trim Tabs NMEA 2000")
      ->set_sort_order(3006);

  trim_deg->connect_to(&trim_tab_sender->trim_deg_);

  // --------------------------------------------------------------------
  // 12. NMEA 2000 Transmission Senders
  // --------------------------------------------------------------------
  N2kTransmissionSender* port_transmission_sender = new N2kTransmissionSender(
      "/NMEA 2000/Port Transmission", 0, nmea2000
  );
  ConfigItem(port_transmission_sender)
      ->set_title("Port Transmission NMEA 2000")
      ->set_sort_order(3011);

  port_gear_position->connect_to(
    new sensesp::LambdaTransform<float, int>([](float ratio) -> int {
      if (ratio < 0.25f) return 0;  // Reverse
      if (ratio < 0.75f) return 1;  // Neutral
      return 2;                     // Forward
    })
  )->connect_to(&port_transmission_sender->gear_);

  N2kTransmissionSender* stbd_transmission_sender = new N2kTransmissionSender(
      "/NMEA 2000/Stbd Transmission", 1, nmea2000
  );
  ConfigItem(stbd_transmission_sender)
      ->set_title("Stbd Transmission NMEA 2000")
      ->set_sort_order(3016);

  stbd_gear_position->connect_to(
    new sensesp::LambdaTransform<float, int>([](float ratio) -> int {
      if (ratio < 0.25f) return 0;  // Reverse
      if (ratio < 0.75f) return 1;  // Neutral
      return 2;                     // Forward
    })
  )->connect_to(&stbd_transmission_sender->gear_);

  // ========================================================================
  // 13. OLED — WORKS WITH OR WITHOUT OLED, !!STATIC OR BUST!!
  // ========================================================================
if (display_present && display) {
// === ROW 0: WiFi Status (alternates between SSID + signal and IP address) ===
static bool show_ip = false;
event_loop()->onRepeat(5000, []() {
  if (WiFi.status() == WL_CONNECTED) {
    if (show_ip) {
      String ip = WiFi.localIP().toString();
      PrintValue(display, 0, ip, "", "");
    } else {
      String ssid = WiFi.SSID();
      String signal = String(WiFi.RSSI()) + "dBm";
      PrintValue(display, 0, ssid, signal, "");
    }
    show_ip = !show_ip;  // toggle for next cycle
  } else {
    PrintValue(display, 0, "WiFi Offline", "", "");
    show_ip = false;  // reset when offline
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
  tacho_d1_frequency->connect_to(new LambdaConsumer<float>([&](float v) {
    rpm_l = String((int)(60 * v));
    update_rpm();
  }));
  tacho_d2_frequency->connect_to(new LambdaConsumer<float>([&](float v) {
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
  if (port_oil_psi) port_oil_psi->connect_to(new LambdaConsumer<float>([&](float v){
    oil_l = String((int)v);
    update_oil();
  }));
  if (stbd_oil_psi) stbd_oil_psi->connect_to(new LambdaConsumer<float>([&](float v){
    oil_r = String((int)v);
    update_oil();
  }));
  if (!port_oil_psi && !stbd_oil_psi) {
    PrintValue(display, 3, "Oil", "--p", "--p");
  }

  // === ROW 4: Temp ===
  static String temp_l = "---", temp_r = "---";
  auto update_temp = [&]() {
    String p = temp_l + "f";
    String s = temp_r + "f";
    PrintValue(display, 4, "Temp", p, s);
  };
  if (port_temp_f) port_temp_f->connect_to(new LambdaConsumer<float>([&](float v){
    temp_l = String((int)v);
    update_temp();
  }));
  if (stbd_temp_f) stbd_temp_f->connect_to(new LambdaConsumer<float>([&](float v){
    temp_r = String((int)v);
    update_temp();
  }));
  if (!port_temp_f && !stbd_temp_f) {
    PrintValue(display, 4, "Temp", "---f", "---f");
  }

  // === ROW 5: Gear ===
  static String gear_l = "N", gear_r = "N";
  auto update_gear = [&]() {
    PrintValue(display, 5, "Gear", gear_l, gear_r);
  };
  port_gear_position->connect_to(new LambdaConsumer<float>([&](float v) {
    gear_l = (v < 0.25f) ? "R" : ((v < 0.75f) ? "N" : "F");
    update_gear();
  }));
  stbd_gear_position->connect_to(new LambdaConsumer<float>([&](float v) {
    gear_r = (v < 0.25f) ? "R" : ((v < 0.75f) ? "N" : "F");
    update_gear();
  }));

  // === ROW 6: Rd/Tr ===
  static String rud = "---", trm = "---";
  auto update_rudtrm = [&]() {
    PrintValue(display, 6, "Rd/Tr", rud, trm);
  };
  rudder_deg->connect_to(new LambdaConsumer<float>([&](float v) {
    rud = String((int)v);
    update_rudtrm();
  }));
  trim_deg->connect_to(new LambdaConsumer<float>([&](float v) {
    trm = String((int)v);
    update_rudtrm();
  }));

  // === ROW 7: AIS ===
  static String ais_a = "0a", ais_b = "0b";
  event_loop()->onRepeat(1000, []() {
    extern int ais_msg_count_a, ais_msg_count_b;
    ais_a = String(ais_msg_count_a) + "a";
    ais_b = String(ais_msg_count_b) + "b";
    PrintValue(display, 7, "AIS", ais_a, ais_b);
  });
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
