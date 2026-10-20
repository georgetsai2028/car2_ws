// Standalone CAN1 wiring test. No actuator libraries or actuator pin writes.
// Temporary diagnostic firmware: restore the appropriate vehicle image afterward.
// IDs 0x6A0/0x6A1 do not overlap the vehicle command/status protocol.
#include <Arduino.h>
#include <FlexCAN_T4.h>

#ifndef CAN_LINK_SENDER
#define CAN_LINK_SENDER 0
#endif

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
static constexpr uint32_t REQUEST_ID = 0x6A0;
static constexpr uint32_t REPLY_ID = 0x6A1;
static uint32_t sequence = 0, latest_sent = 0;
static uint32_t tx_accepted = 0, tx_busy = 0, rx_total = 0, matched = 0;
static bool sent = false, reply_seen = false;

static uint32_t readSequence(const CAN_message_t& msg) {
    return (uint32_t)msg.buf[4] | ((uint32_t)msg.buf[5] << 8) |
           ((uint32_t)msg.buf[6] << 16) | ((uint32_t)msg.buf[7] << 24);
}

static void submit(const CAN_message_t& msg) {
    // Accepted/queued is not proof of acknowledgment or delivery.
    if (can.write(msg) == 1) ++tx_accepted;
    else ++tx_busy;
}

void setup() {
    Serial.begin(115200);
    can.begin();
    can.setTX(DEF); // Teensy 4.1 CAN1 TX = pin 22
    can.setRX(DEF); // Teensy 4.1 CAN1 RX = pin 23
    can.setBaudRate(250000);
}

void loop() {
    CAN_message_t msg;
    while (can.read(msg)) {
        ++rx_total;
        if (msg.flags.extended || msg.flags.remote || msg.len != 8) continue;
        if (!CAN_LINK_SENDER && msg.id == REQUEST_ID &&
            memcmp(msg.buf, "PING", 4) == 0) {
            ++matched;
            msg.id = REPLY_ID;
            memcpy(msg.buf, "PONG", 4);
            submit(msg);
        } else if (CAN_LINK_SENDER && sent && msg.id == REPLY_ID &&
                   memcmp(msg.buf, "PONG", 4) == 0 &&
                   readSequence(msg) == latest_sent && !reply_seen) {
            reply_seen = true;
            ++matched;
        }
    }

    static uint32_t last = 0;
    const uint32_t now = millis();
    if (now - last < 1000) return;
    last = now;

    if (CAN_LINK_SENDER) {
        CAN_message_t request;
        request.id = REQUEST_ID;
        request.len = 8;
        memcpy(request.buf, "PING", 4);
        latest_sent = ++sequence;
        for (uint8_t i = 0; i < 4; ++i) request.buf[4 + i] = (uint8_t)(latest_sent >> (8 * i));
        sent = true;
        reply_seen = false;
        submit(request);
    }

    // Read actual hardware counters, not application command echoes.
    const uint32_t ecr = FLEXCANb_ECR(CAN1);
    const uint32_t esr = FLEXCANb_ESR1(CAN1);
    Serial.printf("CAN1 TX=22 RX=23 bitrate=250000 role=%s tx_accepted=%lu tx_busy=%lu "
                  "rx_total=%lu %s=%lu TEC=%lu REC=%lu fault=%lu ESR1=0x%08lX\n",
        CAN_LINK_SENDER ? "SENDER" : "RESPONDER",
        (unsigned long)tx_accepted, (unsigned long)tx_busy,
        (unsigned long)rx_total, CAN_LINK_SENDER ? "replies" : "requests",
        (unsigned long)matched, (unsigned long)(ecr & 0xFF),
        (unsigned long)((ecr >> 8) & 0xFF),
        (unsigned long)((esr >> 4) & 3), (unsigned long)esr);
}
