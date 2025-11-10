// src/ais_gateway.cpp
#include <Arduino.h>
#include <NMEA2000.h>
#include <N2kMsg.h>
#include <N2kMessages.h>
#include <elapsedMillis.h>
#include "ais_gateway.h"

// EXTERNAL GLOBALS FROM main.cpp
extern tNMEA2000* nmea2000;
extern bool ais_silent;

// LOCAL GLOBALS
uint8_t ais_led = 0;
float ais_vsupply = 0.0, ais_v3v3 = 0.0, ais_v6v = 0.0;
float ais_tx_fwd = 0.0, ais_tx_rev = 0.0;
float ais_rssi1 = 0.0, ais_rssi2 = 0.0;
int ais_msg_count = 0;

// === NEW: EXPOSE CLASS A & B COUNTERS ===
int ais_msg_count_a = 0;
int ais_msg_count_b = 0;

static elapsedMillis ais_poll_timer = 0;

// ----------------------------------------------------------------
// PGN 129038 – Class A Position Report
static void SendPGN129038(uint8_t msgType, uint32_t mmsi,
                          double lat, double lon,
                          float sog, float cog, float hdg) {
  if (!nmea2000) return;
  tN2kMsg N2kMsg;
  SetN2kPGN129038(N2kMsg, msgType, N2kaisr_Initial, mmsi,
                  lat, lon, true, false, 0,
                  cog, sog, N2kaischannel_A_VDL_reception,
                  hdg, 0, N2kaisns_Moored);
  nmea2000->SendMsg(N2kMsg);
}

// ----------------------------------------------------------------
// PGN 129794 – Class A Static (Type 5)
static void SendPGN129794(uint32_t mmsi, uint32_t imo,
                          const char* callsign, const char* name,
                          uint8_t shipType,
                          uint16_t toBow, uint16_t toStern,
                          uint8_t toPort, uint8_t toStarboard) {
  if (!nmea2000) return;
  tN2kMsg N2kMsg;
  SetN2kPGN129794(N2kMsg,
    5, N2kaisr_Initial, mmsi, imo,
    callsign, name, shipType,
    toBow + toStern, toPort + toStarboard,
    toStarboard, toBow,
    0, 0, 0.0, "",
    N2kaisv_ITU_R_M_1371_1,
    N2kGNSSt_GPS,
    N2kaisdte_Ready,
    N2kaischannel_A_VDL_reception,
    0);
  nmea2000->SendMsg(N2kMsg);
}

// ----------------------------------------------------------------
// PGN 129039 – Class B Position (Type 18)
static void SendPGN129039(uint32_t mmsi,
                          double lat, double lon,
                          float sog, float cog, float hdg) {
  if (!nmea2000) return;
  tN2kMsg N2kMsg;
  SetN2kPGN129039(N2kMsg, 18, N2kaisr_Initial, mmsi,
                  lat, lon, true, false, 0,
                  cog, sog, N2kaischannel_A_VDL_reception,
                  hdg, N2kaisunit_ClassB_CS,
                  true, true, true, true,
                  N2kaismode_Autonomous, true);
  nmea2000->SendMsg(N2kMsg);
}

// ----------------------------------------------------------------
// PGN 129029 – GNSS Position Data
static void SendPGN129029(double lat, double lon, double alt,
                          uint8_t sats, uint8_t quality, double hdop) {
  if (!nmea2000) return;
  tN2kMsg N2kMsg;
  SetN2kPGN129029(N2kMsg, 0, 0, 0, lat, lon, alt,
                  N2kGNSSt_GPS, N2kGNSSm_GNSSfix,
                  sats, hdop);
  nmea2000->SendMsg(N2kMsg);
}

// ----------------------------------------------------------------
// Parse NMEA 0183
static void ParseNMEA(const char* line) {
  if (!line[0]) return;

    // VDM / VDO
    if (strncmp(line, "!AI", 3) == 0 && (line[3] == 'V' || line[3] == 'D')) {
      char type = line[5];
      char* p = (char*)line;
      for (int i = 0; i < 5; ++i) p = strchr(p, ',') + 1;
      uint32_t mmsi = atol(p); p = strchr(p, ',') + 1;
      ais_msg_count++;

      // === INCREMENT CLASS A OR B COUNTER ===
      if (type >= '1' && type <= '3') {
        ais_msg_count_a++;
        // Position reports (types 1-3)
        p = strchr(p, ',') + 1; float sog = atof(p) / 10.0; p = strchr(p, ',') + 1;
        float cog = atof(p) / 10.0; p = strchr(p, ',') + 1;
        double lat = atof(p) / 600000.0; p = strchr(p, ',') + 1;
        double lon = atof(p) / 600000.0; p = strchr(p, ',') + 1;
        p = strchr(p, ',') + 1; float hdg = atof(p);
        SendPGN129038(type - '0', mmsi, lat, lon, sog, cog, hdg);
      }
      else if (type == '5') {
        ais_msg_count_a++;
        // Static data (type 5)
        p = strchr(p, ',') + 1; uint32_t imo = atol(p); p = strchr(p, ',') + 1;
        char callsign[8] = {0}; strncpy(callsign, p, 7); p = strchr(p, ',') + 1;
        char name[21] = {0}; strncpy(name, p, 20); p = strchr(p, ',') + 1;
        uint8_t shipType = atoi(p); p = strchr(p, ',') + 1;
        uint16_t toBow = atoi(p); p = strchr(p, ',') + 1;
        uint16_t toStern = atoi(p); p = strchr(p, ',') + 1;
        uint8_t toPort = atoi(p); p = strchr(p, ',') + 1;
        uint8_t toStarboard = atoi(p);
        SendPGN129794(mmsi, imo, callsign, name, shipType, toBow, toStern, toPort, toStarboard);
      }
      else if (type == 'B') { // 18
        ais_msg_count_b++;
        // Class B position (type 18)
        p = strchr(p, ',') + 1; p = strchr(p, ',') + 1;
        float sog = atof(p) / 10.0; p = strchr(p, ',') + 1;
        float cog = atof(p) / 10.0; p = strchr(p, ',') + 1;
        double lat = atof(p) / 600000.0; p = strchr(p, ',') + 1;
        double lon = atof(p) / 600000.0; p = strchr(p, ',') + 1;
        p = strchr(p, ',') + 1; float hdg = atof(p);
        SendPGN129039(mmsi, lat, lon, sog, cog, hdg);
      }
      return;
    }

  // GPS: GGA
  if (strncmp(line, "$GPGGA", 6) == 0) {
    char* p = (char*)line;
    for (int i = 0; i < 2; ++i) p = strchr(p, ',') + 1;
    double lat = atof(p) / 100.0; p = strchr(p, ',') + 1;
    char ns = *p; p = strchr(p, ',') + 1;
    double lon = atof(p) / 100.0; p = strchr(p, ',') + 1;
    char ew = *p; p = strchr(p, ',') + 1;
    uint8_t qual = atoi(p); p = strchr(p, ',') + 1;
    uint8_t sats = atoi(p); p = strchr(p, ',') + 1;
    double hdop = atof(p); p = strchr(p, ',') + 1;
    double alt = atof(p);
    if (ns == 'S') lat = -lat;
    if (ew == 'W') lon = -lon;
    SendPGN129029(lat, lon, alt, sats, qual, hdop);
  }
}

// ----------------------------------------------------------------
// Serial reader
static char buf[256];
static int pos = 0;

void AISGatewayInit() {
  // Serial2 already started in main.cpp
}

static void PollAISStatus() {
  // Poll AIS status every 30 seconds instead of 10 to reduce serial traffic
  if (ais_poll_timer > 30000) {
    AISSendCommand("$DUAIQ,LED");
    AISSendCommand("$DUAIQ,ADC");
    AISSendCommand("$DUAIQ,SRM");
    ais_poll_timer = 0;
  }
}

// Send command with checksum
void AISSendCommand(const char* cmd) {
  char buf[128];
  uint8_t cksum = 0;
  const char* p = cmd;
  while (*p && *p != '*') cksum ^= *p++;
  snprintf(buf, sizeof(buf), "%s*%02X\r\n", cmd, cksum);
  Serial2.print(buf);
}

void AISGatewayLoop() {
  PollAISStatus();
  
  // Process up to 64 characters per loop iteration to prevent blocking
  const int MAX_CHARS_PER_LOOP = 64;
  int chars_processed = 0;
  
  while (Serial2.available() && chars_processed < MAX_CHARS_PER_LOOP) {
    char c = Serial2.read();
    chars_processed++;
    
    if (c == '\r' || c == '\n') {
      if (pos) { buf[pos] = 0; ParseNMEA(buf); pos = 0; }
    } else if (pos < 255) {
      buf[pos++] = c;
    }
    
    // Process status messages inline during parsing
    if (pos >= 10) {  // Minimum length for status messages
      if (strncmp(buf, "$PSRT,LED,", 10) == 0) {
        char* p = strchr(buf, ',') + 1;
        ais_led = atoi(p);
      }
      else if (strncmp(buf, "$PSRT,ADC,", 10) == 0) {
        char* p = strchr(buf, ',') + 1;
        ais_tx_fwd = atof(p); p = strchr(p, ',') + 1;
        ais_tx_rev = atof(p); p = strchr(p, ',') + 1;
        ais_rssi1 = atof(p); p = strchr(p, ',') + 1;
        ais_rssi2 = atof(p); p = strchr(p, ',') + 1;
        ais_v3v3 = atof(p); p = strchr(p, ',') + 1;
        ais_v6v = atof(p); p = strchr(p, ',') + 1;
        ais_vsupply = atof(p);
      }
      else if (strncmp(buf, "$PSRT,SRM,", 10) == 0) {
        char* p = strchr(buf, ',') + 1;
        p = strchr(p, ',') + 1;
        ais_silent = (atoi(p) & 0x08) != 0;
      }
    }
  }
  
  // Send AIS status PGN if polling interval has elapsed
  if (nmea2000 && ais_poll_timer > 30000) {
    tN2kMsg Msg;
    Msg.SetPGN(130001L);
    Msg.Priority = 6;
    Msg.AddByte(ais_led);
    Msg.Add2ByteDouble(ais_vsupply, 0.1);
    Msg.Add2ByteDouble(ais_v3v3, 0.01);
    Msg.Add2ByteDouble(ais_v6v, 0.01);
    Msg.Add2ByteDouble(ais_tx_fwd, 0.1);
    Msg.Add2ByteDouble(ais_tx_rev, 0.1);
    Msg.Add2ByteDouble(ais_rssi1, 0.1);
    Msg.Add2ByteDouble(ais_rssi2, 0.1);
    nmea2000->SendMsg(Msg);
  }
}