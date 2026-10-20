#pragma once
#include <Arduino.h>
struct Adafruit_MCP4725 {
    uint16_t lastCounts = 0;
    bool begin(uint8_t) { return true; }
    bool setVoltage(uint16_t counts, bool) { lastCounts = counts; return true; }
};
