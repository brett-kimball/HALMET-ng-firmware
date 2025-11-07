// halmet_display.cpp
#include "halmet_display.h"
#include "sensesp/system/local_debug.h"

namespace halmet {

// INITIALIZE OLED
bool InitializeSSD1306(
    const sensesp::SensESPBaseApp* sensesp_app,
    Adafruit_SSD1306** display_ptr,
    TwoWire* i2c) {

  Adafruit_SSD1306* display = new Adafruit_SSD1306(kScreenWidth, kScreenHeight, i2c, -1);

  if (!display) {
    debugD("SSD1306 allocation failed");
    return false;
  }

  if (!display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    debugD("SSD1306 begin failed (address 0x3C)");
    delete display;
    return false;
  }

  delay(100);
  display->setRotation(0);  // 8 lines, 21 chars
  display->clearDisplay();
  display->setTextSize(1);
  display->setTextColor(SSD1306_WHITE);

  *display_ptr = display;
  return true;
}

// CLEAR ROW
void ClearRow(Adafruit_SSD1306* display, int row) {
  if (!display) return;
  display->fillRect(0, row * 8, kScreenWidth, 8, SSD1306_BLACK);
}

// Helper: safe space padding
String spaces(int count) {
  String s = "";
  for (int i = 0; i < count && i < 21; i++) s += ' ';
  return s;
}

void PrintValue(Adafruit_SSD1306* display,
                int row,
                const String& title,
                const String& value1,
                const String& value2) {
  if (!display) return;
  ClearRow(display, row);
  display->setCursor(0, row * 8);

  String line = "";
  int title_len = title.length();
  int v1_len = value1.length();
  int v2_len = value2.length();

  // CASE 1: Title only
  if (title_len > 0 && v1_len == 0 && v2_len == 0) {
    line = title;
    if (line.length() > 21) line = line.substring(0, 21);
    line += spaces(21 - line.length());
  }
  // CASE 2: Title + value1 only
  else if (title_len > 0 && v1_len > 0 && v2_len == 0) {
    line = title + ": ";
    int used = line.length() + v1_len;
    int pad = 21 - used;
    if (pad < 0) pad = 0;
    line += spaces(pad) + value1.substring(0, 21 - line.length());
  }
  // CASE 3: Title + value1 + value2
  else if (title_len > 0 && v1_len > 0 && v2_len > 0) {
    line = title + ": ";
    int v1_space = 15 - line.length();
    int v1_pad = v1_space - v1_len;
    if (v1_pad < 0) v1_pad = 0;
    int v2_pad = 6 - v2_len;
    if (v2_pad < 0) v2_pad = 0;
    line += spaces(v1_pad) + value1.substring(0, v1_space) + 
            spaces(v2_pad) + value2.substring(0, 6);
  }
  // CASE 4: No title, value1 only
  else if (title_len == 0 && v1_len > 0 && v2_len == 0) {
    int pad = 21 - v1_len;
    line = spaces(pad) + value1.substring(0, 21);
  }
  // CASE 5: No title, value1 + value2
  else if (title_len == 0 && v1_len > 0 && v2_len > 0) {
    int v1_pad = 15 - v1_len;
    int v2_pad = 6 - v2_len;
    if (v1_pad < 0) v1_pad = 0;
    if (v2_pad < 0) v2_pad = 0;
    line = spaces(v1_pad) + value1.substring(0, 15) + 
           spaces(v2_pad) + value2.substring(0, 6);
  }

  if (line.length() > 21) line = line.substring(0, 21);
  display->printf("%s", line.c_str());
  display->display();
}

}  // namespace halmet