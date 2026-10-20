#pragma once
#include <Arduino.h>
struct MockWire {
    void begin() {}
    void setClock(uint32_t) {}
};
static MockWire Wire;
