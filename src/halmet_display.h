// halmet_display.h
#pragma once

#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Arduino.h>
#include <String.h>
#include "sensesp_base_app.h"

namespace halmet {

const int kScreenWidth = 128;
const int kScreenHeight = 64;

bool InitializeSSD1306(
    const sensesp::SensESPBaseApp* sensesp_app,
    Adafruit_SSD1306** display,
    TwoWire* i2c
);

void ClearRow(Adafruit_SSD1306* display, int row);

void PrintValue(Adafruit_SSD1306* display,
                int row,
                const String& title = "",
                const String& value1 = "",
                const String& value2 = "");

}  // namespace halmet