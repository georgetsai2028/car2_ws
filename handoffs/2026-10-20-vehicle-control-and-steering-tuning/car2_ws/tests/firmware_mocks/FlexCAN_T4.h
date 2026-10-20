#pragma once
#include <Arduino.h>
#include <vector>
enum { CAN1, RX_SIZE_256, TX_SIZE_16 };
enum FLEXCAN_PINS { DEF, ALT };
static uint32_t mock_ecr = 0, mock_esr = 0;
#define FLEXCANb_ECR(bus) mock_ecr
#define FLEXCANb_ESR1(bus) mock_esr
struct CAN_message_t {
    uint32_t id = 0;
    uint8_t len = 0;
    uint8_t buf[8] = {};
    struct { bool extended = false, remote = false; } flags;
};
template<int, int, int> struct FlexCAN_T4 {
    std::deque<CAN_message_t> incoming;
    std::vector<CAN_message_t> outgoing;
    uint32_t advanceOnNextReadMs = 0;
    void begin() {}
    void setTX(FLEXCAN_PINS) {}
    void setRX(FLEXCAN_PINS) {}
    void setBaudRate(uint32_t) {}
    bool read(CAN_message_t& msg) {
        if (incoming.empty()) return false;
        mock_now += advanceOnNextReadMs;
        advanceOnNextReadMs = 0;
        msg = incoming.front(); incoming.pop_front(); return true;
    }
    bool write(const CAN_message_t& msg) { outgoing.push_back(msg); return true; }
};
