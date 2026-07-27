// src/ais_gateway.cpp
//
// AIS → NMEA 2000 gateway.
//
// Receives standard NMEA 0183 VDM/VDO sentences from an AIS transponder on
// Serial2, decodes the 6-bit armored payload, and re-transmits the data as
// the appropriate N2K PGNs so chartplotters and other N2K devices see traffic.
//
// PGN mapping:
//   AIS Type  1/2/3  (Class A position)       → PGN 129038
//   AIS Type  5      (Class A static/voyage)   → PGN 129794  (2-sentence)
//   AIS Type  18     (Class B CS position)     → PGN 129039
//   AIS Type  21     (Aid to Navigation)       → PGN 129041
//   GPRMC / GPGGA    (own-vessel GPS)          → PGN 129026 / 129029
//
// Types 4 (base station), 9 (SAR), 14, 24, etc. are logged but not
// converted — PGN 129793 is not in the bundled NMEA2000 library.

#include <Arduino.h>
#include <NMEA2000.h>
#include <N2kMsg.h>
#include <N2kMessages.h>
#include <elapsedMillis.h>
#include "ais_gateway.h"

extern tNMEA2000* nmea2000;
extern bool ais_silent;

int ais_msg_count_a = 0;
int ais_msg_count_b = 0;

static elapsedMillis ais_poll_timer = 0;

// AIS hardware status values (populated from $PSRT responses)
static uint8_t ais_led      = 0;
static float ais_vsupply    = 0.0f;
static float ais_v3v3       = 0.0f;
static float ais_v6v        = 0.0f;
static float ais_tx_fwd     = 0.0f;
static float ais_tx_rev     = 0.0f;
static float ais_rssi1      = 0.0f;
static float ais_rssi2      = 0.0f;

// ----------------------------------------------------------------
// 6-BIT VDM PAYLOAD BIT EXTRACTION
//
// AIS VDM sentences encode binary data as 6-bit ASCII-armored characters.
// Each character encodes 6 bits: value = ASCII - 48, then subtract 8 more
// if the result exceeds 40. Bits are packed MSB-first within each character.
// ----------------------------------------------------------------

static uint32_t vdm_uint(const char* p, int startBit, int numBits) {
  uint32_t v = 0;
  for (int i = startBit; i < startBit + numBits; i++) {
    int c = (uint8_t)p[i / 6] - 48;
    if (c > 40) c -= 8;
    v = (v << 1) | ((c >> (5 - i % 6)) & 1);
  }
  return v;
}

static int32_t vdm_int(const char* p, int startBit, int numBits) {
  uint32_t v = vdm_uint(p, startBit, numBits);
  if (v & (1u << (numBits - 1)))
    v |= ~((1u << numBits) - 1u);
  return (int32_t)v;
}

// Extract a 6-bit ASCII string from the payload. '@' (ASCII 64) is the AIS
// padding/space character; trailing '@' and spaces are trimmed.
static void vdm_str(const char* p, int startBit, int nChars,
                    char* out, int outLen) {
  int n = (nChars < outLen - 1) ? nChars : outLen - 1;
  for (int i = 0; i < n; i++) {
    int c = (int)vdm_uint(p, startBit + i * 6, 6);
    if (c < 32) c += 64;
    out[i] = (char)c;
  }
  while (n > 0 && (out[n - 1] == '@' || out[n - 1] == ' ')) n--;
  out[n] = '\0';
}

// Convert NMEA DDMM.MMMM format to decimal degrees.
static double nmea_to_deg(const char* s) {
  double raw = atof(s);
  int deg    = (int)(raw / 100);
  return deg + (raw - deg * 100.0) / 60.0;
}

// ----------------------------------------------------------------
// NMEA 2000 SENDERS
// ----------------------------------------------------------------

// PGN 129038 — AIS Class A Position Report (Types 1, 2, 3)
static void SendPGN129038(int msgType, uint32_t mmsi,
                          double lat, double lon,
                          float sog_kn, float cog_deg, float hdg_deg,
                          uint8_t navStatus) {
  if (!nmea2000) return;
  double sog_ms  = (sog_kn  >= 102.3f) ? N2kDoubleNA : (double)sog_kn * 0.514444;
  double cog_rad = (cog_deg >= 360.0f)  ? N2kDoubleNA : (double)cog_deg * DEG_TO_RAD;
  double hdg_rad = (hdg_deg >= 360.0f)  ? N2kDoubleNA : (double)hdg_deg * DEG_TO_RAD;
  tN2kMsg msg;
  SetN2kPGN129038(msg, (uint8_t)msgType, N2kaisr_Initial, mmsi,
                  lat, lon,
                  false, false, 0xFF,
                  cog_rad, sog_ms,
                  N2kaischannel_A_VDL_reception,
                  hdg_rad,
                  N2kDoubleNA,
                  (tN2kAISNavStatus)navStatus);
  nmea2000->SendMsg(msg);
}

// PGN 129039 — AIS Class B CS Position Report (Type 18)
static void SendPGN129039(uint32_t mmsi,
                          double lat, double lon,
                          float sog_kn, float cog_deg, float hdg_deg) {
  if (!nmea2000) return;
  double sog_ms  = (sog_kn  >= 102.3f) ? N2kDoubleNA : (double)sog_kn * 0.514444;
  double cog_rad = (cog_deg >= 360.0f)  ? N2kDoubleNA : (double)cog_deg * DEG_TO_RAD;
  double hdg_rad = (hdg_deg >= 360.0f)  ? N2kDoubleNA : (double)hdg_deg * DEG_TO_RAD;
  tN2kMsg msg;
  SetN2kPGN129039(msg, 18, N2kaisr_Initial, mmsi,
                  lat, lon,
                  false, false, 0xFF,
                  cog_rad, sog_ms,
                  N2kaischannel_A_VDL_reception,
                  hdg_rad,
                  N2kaisunit_ClassB_CS,
                  true, true, true, true,
                  N2kaismode_Autonomous, true);
  nmea2000->SendMsg(msg);
}

// PGN 129794 — AIS Class A Static and Voyage Data (Type 5, 2-sentence)
static void SendPGN129794(const char* payload) {
  if (!nmea2000) return;
  uint32_t mmsi    = vdm_uint(payload, 8, 30);
  uint32_t imo     = vdm_uint(payload, 40, 30);
  char callsign[8]; vdm_str(payload, 70, 7, callsign, sizeof(callsign));
  char name[21];    vdm_str(payload, 112, 20, name, sizeof(name));
  uint8_t shipType = (uint8_t)vdm_uint(payload, 232, 8);
  double toBow     = (double)vdm_uint(payload, 240, 9);
  double toStern   = (double)vdm_uint(payload, 249, 9);
  double toPort    = (double)vdm_uint(payload, 258, 4);
  double toStbd    = (double)vdm_uint(payload, 262, 4);
  tN2kMsg msg;
  SetN2kPGN129794(msg,
                  5, N2kaisr_Initial, mmsi, imo,
                  callsign, name, shipType,
                  toBow + toStern, toPort + toStbd,
                  toStbd, toBow,
                  0, 0.0, 0.0, "",
                  N2kaisv_ITU_R_M_1371_1, N2kGNSSt_GPS, N2kaisdte_Ready,
                  N2kaischannel_A_VDL_reception);
  nmea2000->SendMsg(msg);
}

// PGN 129041 — AIS Aid-to-Navigation Report (Type 21)
static void SendPGN129041(const char* payload) {
  if (!nmea2000) return;
  tN2kAISAtoNReportData d;
  d.MessageID = 21;
  d.Repeat    = N2kaisr_Initial;
  d.UserID    = vdm_uint(payload, 8, 30);
  d.AtoNType  = (tN2kAISAtoNType)vdm_uint(payload, 38, 5);
  char name[21]; vdm_str(payload, 43, 20, name, sizeof(name));
  d.SetAtoNName(name);
  d.Accuracy         = (bool)vdm_uint(payload, 163, 1);
  int32_t lon_raw    = vdm_int(payload, 164, 28);
  int32_t lat_raw    = vdm_int(payload, 192, 27);
  d.Longitude        = (lon_raw == 0x6791AC0) ? N2kDoubleNA : lon_raw / 600000.0;
  d.Latitude         = (lat_raw == 0x3412140) ? N2kDoubleNA : lat_raw / 600000.0;
  double toBow       = (double)vdm_uint(payload, 219, 9);
  double toStern     = (double)vdm_uint(payload, 228, 9);
  double toPort      = (double)vdm_uint(payload, 237, 4);
  double toStbd      = (double)vdm_uint(payload, 241, 4);
  d.Length                    = toBow + toStern;
  d.Beam                      = toPort + toStbd;
  d.PositionReferenceTrueNorth = toBow;
  d.PositionReferenceStarboard = toStbd;
  d.GNSSType                  = (tN2kGNSStype)vdm_uint(payload, 245, 4);
  d.Seconds                   = (uint8_t)vdm_uint(payload, 249, 6);
  d.OffPositionIndicator       = (bool)vdm_uint(payload, 255, 1);
  d.RAIM                      = (bool)vdm_uint(payload, 264, 1);
  d.VirtualAtoNFlag            = (bool)vdm_uint(payload, 265, 1);
  d.AssignedModeFlag           = (bool)vdm_uint(payload, 266, 1);
  d.AISTransceiverInformation  = N2kaischannel_A_VDL_reception;
  d.AtoNStatus                = 0;
  tN2kMsg msg;
  SetN2kPGN129041(msg, d);
  nmea2000->SendMsg(msg);
}

// ----------------------------------------------------------------
// MULTI-SENTENCE ASSEMBLY
//
// AIS Type 5 (Class A static) spans 2 VDM sentences; we buffer the first
// and combine with the second before parsing. Buffer sized for 71 payload
// chars + NUL.
// ----------------------------------------------------------------

static char  s_type5[80] = {0};
static bool  s_type5_got = false;

// ----------------------------------------------------------------
// SENTENCE PARSER
// ----------------------------------------------------------------

static void ParseNMEA(const char* line) {
  if (!line || !line[0]) return;

  // VDM / VDO — standard AIS armored sentences
  if (strncmp(line, "!AIVDM", 6) == 0 || strncmp(line, "!AIVDO", 6) == 0) {
    // Sentence format: !AIVDx,<total>,<sentNum>,<seqId>,<channel>,<payload>,<fill>*<cksum>

    // Field 1: total sentence count for this message
    const char* f = strchr(line, ',');
    if (!f) return;
    int total   = atoi(f + 1);

    // Field 2: sentence number within this message
    f = strchr(f + 1, ',');
    if (!f) return;
    int sentNum = atoi(f + 1);

    // Advance to field 5 (the payload)
    const char* fp = line;
    for (int i = 0; i < 5; i++) {
      fp = strchr(fp, ',');
      if (!fp) return;
      fp++;
    }
    const char* payloadEnd = strchr(fp, ',');
    if (!payloadEnd) return;
    int plen = (int)(payloadEnd - fp);
    if (plen < 1 || plen > 56) return;

    // Decode message type from the first payload character
    int cv = (uint8_t)fp[0] - 48;
    if (cv > 40) cv -= 8;
    int msgType = cv;

    // Multi-sentence messages (Type 5 is always 2 sentences)
    if (total == 2) {
      if (sentNum == 1) {
        memcpy(s_type5, fp, plen);
        s_type5[plen] = '\0';
        s_type5_got   = true;
      } else if (sentNum == 2 && s_type5_got) {
        int firstLen = (int)strlen(s_type5);
        if (firstLen + plen < (int)sizeof(s_type5) - 1) {
          memcpy(s_type5 + firstLen, fp, plen);
          s_type5[firstLen + plen] = '\0';
        }
        s_type5_got = false;
        if (msgType == 5) {
          ais_msg_count_a++;
          SendPGN129794(s_type5);
        }
      }
      return;
    }
    s_type5_got = false;

    // Single-sentence: copy payload into a local null-terminated buffer
    char payload[57];
    memcpy(payload, fp, plen);
    payload[plen] = '\0';

    uint32_t mmsi = vdm_uint(payload, 8, 30);

    switch (msgType) {
      case 1: case 2: case 3: {  // Class A position
        ais_msg_count_a++;
        float    sog   = (float)vdm_uint(payload, 50, 10) / 10.0f;
        int32_t  lon_r = vdm_int(payload, 61, 28);
        int32_t  lat_r = vdm_int(payload, 89, 27);
        float    cog   = (float)vdm_uint(payload, 116, 12) / 10.0f;
        uint32_t hdg_r = vdm_uint(payload, 128, 9);
        float    hdg   = (hdg_r == 511) ? 511.0f : (float)hdg_r;
        uint8_t  nav   = (uint8_t)vdm_uint(payload, 38, 4);
        SendPGN129038(msgType, mmsi,
                      lat_r / 600000.0, lon_r / 600000.0,
                      sog, cog, hdg, nav);
        break;
      }
      case 18: {  // Class B CS position
        ais_msg_count_b++;
        float   sog   = (float)vdm_uint(payload, 46, 10) / 10.0f;
        int32_t lon_r = vdm_int(payload, 57, 28);
        int32_t lat_r = vdm_int(payload, 85, 27);
        float   cog   = (float)vdm_uint(payload, 112, 12) / 10.0f;
        uint32_t hdg_r = vdm_uint(payload, 124, 9);
        float   hdg   = (hdg_r == 511) ? 511.0f : (float)hdg_r;
        SendPGN129039(mmsi,
                      lat_r / 600000.0, lon_r / 600000.0,
                      sog, cog, hdg);
        break;
      }
      case 21:  // Aid to Navigation
        SendPGN129041(payload);
        break;
      default:
        // Types 4 (base station), 9, 14, 24, etc. — not converted.
        // PGN 129793 (base station) is absent from the bundled N2K library.
        (void)mmsi;
        break;
    }
    return;
  }

  // GPRMC — most common GPS sentence from this AIS hardware.
  // Sends PGN 129026 (COG/SOG Rapid Update).
  if (strncmp(line, "$GPRMC", 6) == 0) {
    const char* p = strchr(line, ',');
    if (!p) return;
    p = strchr(p + 1, ',');    // skip time field
    if (!p) return;
    if (*(p + 1) != 'A') return;  // V = invalid
    p = strchr(p + 1, ',') + 1;   // lat
    double lat = nmea_to_deg(p);
    p = strchr(p, ',') + 1;       // N/S
    if (*p == 'S') lat = -lat;
    p = strchr(p, ',') + 1;       // lon
    double lon = nmea_to_deg(p);
    p = strchr(p, ',') + 1;       // E/W
    if (*p == 'W') lon = -lon;
    p = strchr(p, ',') + 1;       // SOG in knots
    double sog_ms = atof(p) * 0.514444;
    p = strchr(p, ',') + 1;       // COG in degrees true
    double cog_rad = atof(p) * DEG_TO_RAD;
    (void)lat; (void)lon;          // position available but 129026 doesn't carry it
    if (nmea2000) {
      tN2kMsg msg;
      SetN2kPGN129026(msg, 0xFF, N2khr_true, cog_rad, sog_ms);
      nmea2000->SendMsg(msg);
    }
    return;
  }

  // GPGGA — full position fix with altitude and quality.
  // Sends PGN 129029 (GNSS Position Data).
  if (strncmp(line, "$GPGGA", 6) == 0) {
    const char* p = line;
    for (int i = 0; i < 2; ++i) { p = strchr(p, ','); if (!p) return; p++; }
    double lat = nmea_to_deg(p);  p = strchr(p, ',') + 1;
    if (*p == 'S') lat = -lat;    p = strchr(p, ',') + 1;
    double lon = nmea_to_deg(p);  p = strchr(p, ',') + 1;
    if (*p == 'W') lon = -lon;    p = strchr(p, ',') + 1;
    uint8_t qual = (uint8_t)atoi(p);
    if (!qual) return;            p = strchr(p, ',') + 1;
    uint8_t sats = (uint8_t)atoi(p); p = strchr(p, ',') + 1;
    double hdop = atof(p);        p = strchr(p, ',') + 1;
    double alt  = atof(p);
    if (nmea2000) {
      tN2kMsg msg;
      SetN2kPGN129029(msg, 0, 0, 0, lat, lon, alt,
                      N2kGNSSt_GPS, N2kGNSSm_GNSSfix, sats, hdop);
      nmea2000->SendMsg(msg);
    }
    return;
  }

  // $PSRT responses from the AIS transponder hardware
  if (strncmp(line, "$PSRT,", 6) == 0) {
    const char* sub = line + 6;
    if (strncmp(sub, "LED,", 4) == 0) {
      ais_led = (uint8_t)atoi(sub + 4);
    } else if (strncmp(sub, "ADC,", 4) == 0) {
      const char* p = sub + 4;
      ais_tx_fwd  = atof(p); p = strchr(p, ',') + 1;
      ais_tx_rev  = atof(p); p = strchr(p, ',') + 1;
      ais_rssi1   = atof(p); p = strchr(p, ',') + 1;
      ais_rssi2   = atof(p); p = strchr(p, ',') + 1;
      ais_v3v3    = atof(p); p = strchr(p, ',') + 1;
      ais_v6v     = atof(p); p = strchr(p, ',') + 1;
      ais_vsupply = atof(p);
    } else if (strncmp(sub, "SRM,", 4) == 0) {
      const char* p = sub + 4;
      p = strchr(p, ',') + 1;
      ais_silent = (atoi(p) & 0x08) != 0;
    }
    return;
  }
}

// ----------------------------------------------------------------
// SERIAL READER
// ----------------------------------------------------------------

static char s_buf[256];
static int  s_pos = 0;

void AISGatewayInit() {
  // Serial2 already started in main.cpp; nothing to do here.
}

// Periodic AIS hardware status poll + N2K broadcast.
// Called from AISGatewayLoop; resets ais_poll_timer so both actions
// share one timer (avoids the original bug where the N2K broadcast
// fired after the timer was already reset by the poll).
static void PeriodicTasks() {
  if (ais_poll_timer < 30000) return;
  ais_poll_timer = 0;

  AISSendCommand("$DUAIQ,LED");
  AISSendCommand("$DUAIQ,ADC");
  AISSendCommand("$DUAIQ,SRM");

  if (!nmea2000) return;
  tN2kMsg msg;
  msg.SetPGN(130001L);
  msg.Priority = 6;
  msg.AddByte(ais_led);
  msg.Add2ByteDouble(ais_vsupply, 0.1);
  msg.Add2ByteDouble(ais_v3v3,    0.01);
  msg.Add2ByteDouble(ais_v6v,     0.01);
  msg.Add2ByteDouble(ais_tx_fwd,  0.1);
  msg.Add2ByteDouble(ais_tx_rev,  0.1);
  msg.Add2ByteDouble(ais_rssi1,   0.1);
  msg.Add2ByteDouble(ais_rssi2,   0.1);
  nmea2000->SendMsg(msg);
}

void AISSendCommand(const char* cmd) {
  // Compute NMEA checksum: XOR of characters after leading '$'/'!' up to '*'
  uint8_t cksum = 0;
  const char* p = cmd + 1;  // skip the '$' or '!' prefix
  while (*p && *p != '*') cksum ^= (uint8_t)*p++;
  char buf[128];
  snprintf(buf, sizeof(buf), "%s*%02X\r\n", cmd, cksum);
  Serial2.print(buf);
}

void AISGatewayLoop() {
  PeriodicTasks();

  // Process up to 64 bytes per call to avoid blocking the event loop
  int processed = 0;
  while (Serial2.available() && processed < 64) {
    char c = (char)Serial2.read();
    processed++;
    if (c == '\r' || c == '\n') {
      if (s_pos > 0) {
        s_buf[s_pos] = '\0';
        ParseNMEA(s_buf);
        s_pos = 0;
      }
    } else if (s_pos < 255) {
      s_buf[s_pos++] = c;
    } else {
      s_pos = 0;  // overflow — discard and resync
    }
  }
}
