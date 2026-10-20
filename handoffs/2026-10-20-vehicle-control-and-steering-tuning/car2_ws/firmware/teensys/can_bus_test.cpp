// Temporary four-node CAN diagnostic. No actuator libraries or actuator writes.
// Read the master USB output to identify each responding node and its CAN errors.
// CAN_BUS_TEST_NODE: 0=master, 1=throttle, 2=steering, 3=brake.
// Restore the appropriate operational image after testing.
#include <Arduino.h>
#include <FlexCAN_T4.h>

#ifndef CAN_BUS_TEST_NODE
#define CAN_BUS_TEST_NODE 0
#endif
static_assert(CAN_BUS_TEST_NODE >= 0 && CAN_BUS_TEST_NODE <= 3,
              "Select master=0, throttle=1, steering=2, or brake=3");

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
static constexpr uint32_t REQUEST_BASE = 0x6B0;
static constexpr uint32_t REPLY_BASE = 0x6C0;
static constexpr uint32_t PROBE_PERIOD_MS = 1000;
static constexpr uint32_t REPLY_TIMEOUT_MS = 2500;
static constexpr uint8_t REPLY_MAGIC = 0xDB;
static constexpr uint8_t VERSION_BITS = 0x10;
static constexpr uint32_t SYNCH_MASK = 1UL << 18;

struct Peer {
    uint32_t expected_sequence = 0;
    uint32_t replies = 0;
    uint32_t last_reply_ms = 0;
    bool sent = false;
    bool reply_seen = false;
    bool ever_seen = false;
    uint8_t tec = 0, rec = 0, fault = 0;
    bool synchronized = false;
};
static Peer peers[4];
static uint32_t sequence = 0, rx_total = 0, tx_accepted = 0, tx_busy = 0;
static uint32_t requests = 0;

static const char* nodeName(uint8_t node) {
    switch (node) {
        case 0: return "MASTER";
        case 1: return "THROTTLE";
        case 2: return "STEERING";
        case 3: return "BRAKE";
        default: return "UNKNOWN";
    }
}

static uint32_t readU32(const uint8_t* data) {
    return uint32_t(data[0]) | (uint32_t(data[1]) << 8) |
           (uint32_t(data[2]) << 16) | (uint32_t(data[3]) << 24);
}

static void writeU32(uint8_t* data, uint32_t value) {
    for (uint8_t i = 0; i < 4; ++i) data[i] = uint8_t(value >> (8 * i));
}

static void submit(const CAN_message_t& msg) {
    // A successful write submission is not proof of delivery. Replies prove it.
    if (can.write(msg) == 1) ++tx_accepted;
    else ++tx_busy;
}

static const char* peerState(const Peer& peer, uint32_t now) {
    if (!peer.ever_seen) return "NO_REPLY";
    if (uint32_t(now - peer.last_reply_ms) > REPLY_TIMEOUT_MS) return "STALE";
    if (!peer.synchronized || peer.tec || peer.rec || peer.fault) return "CAN_ERROR";
    return "OK";
}

static void handleFrame(const CAN_message_t& msg) {
    ++rx_total;
    if (msg.flags.extended || msg.flags.remote || msg.len != 8) return;

    if (CAN_BUS_TEST_NODE == 0) {
        if (msg.id <= REPLY_BASE || msg.id > REPLY_BASE + 3 ||
            msg.buf[0] != REPLY_MAGIC || (msg.buf[1] & 0xF8) != VERSION_BITS) return;
        const uint8_t node = uint8_t(msg.id - REPLY_BASE);
        Peer& peer = peers[node];
        if (!peer.sent || peer.reply_seen ||
            readU32(msg.buf + 2) != peer.expected_sequence) return;
        peer.reply_seen = true;
        peer.ever_seen = true;
        ++peer.replies;
        peer.last_reply_ms = millis();
        peer.fault = msg.buf[1] & 3;
        peer.synchronized = (msg.buf[1] & 4) != 0;
        peer.tec = msg.buf[6];
        peer.rec = msg.buf[7];
    } else {
        if (msg.id != REQUEST_BASE + CAN_BUS_TEST_NODE ||
            memcmp(msg.buf, "PING", 4) != 0) return;
        ++requests;
        const uint32_t ecr = FLEXCANb_ECR(CAN1);
        const uint32_t esr = FLEXCANb_ESR1(CAN1);
        CAN_message_t reply;
        reply.id = REPLY_BASE + CAN_BUS_TEST_NODE;
        reply.len = 8;
        reply.buf[0] = REPLY_MAGIC;
        reply.buf[1] = VERSION_BITS | uint8_t((esr >> 4) & 3) |
                       ((esr & SYNCH_MASK) ? 4 : 0);
        writeU32(reply.buf + 2, readU32(msg.buf + 4));
        reply.buf[6] = uint8_t(ecr & 0xFF);
        reply.buf[7] = uint8_t((ecr >> 8) & 0xFF);
        submit(reply);
    }
}

static void probeAll() {
    ++sequence;
    for (uint8_t node = 1; node <= 3; ++node) {
        CAN_message_t request;
        request.id = REQUEST_BASE + node;
        request.len = 8;
        memcpy(request.buf, "PING", 4);
        writeU32(request.buf + 4, sequence);
        peers[node].expected_sequence = sequence;
        peers[node].sent = true;
        peers[node].reply_seen = false;
        submit(request);
    }
}

static void printStatus(uint32_t now) {
    const uint32_t ecr = FLEXCANb_ECR(CAN1);
    const uint32_t esr = FLEXCANb_ESR1(CAN1);
    Serial.printf("CAN_TEST_V1 role=%s CAN1 TX=22 RX=23 bitrate=250000 "
                  "sync=%u TEC=%lu REC=%lu fault=%lu rx=%lu tx=%lu busy=%lu",
        nodeName(CAN_BUS_TEST_NODE), unsigned((esr & SYNCH_MASK) != 0),
        (unsigned long)(ecr & 0xFF), (unsigned long)((ecr >> 8) & 0xFF),
        (unsigned long)((esr >> 4) & 3), (unsigned long)rx_total,
        (unsigned long)tx_accepted, (unsigned long)tx_busy);
    if (CAN_BUS_TEST_NODE == 0) {
        for (uint8_t node = 1; node <= 3; ++node) {
            const Peer& peer = peers[node];
            Serial.printf(" | %s=%s replies=%lu", nodeName(node),
                          peerState(peer, now), (unsigned long)peer.replies);
            // Absent measurements remain unknown, rather than printing false zeros.
            if (peer.ever_seen) {
                Serial.printf(" age_ms=%lu TEC=%u REC=%u fault=%u sync=%u",
                    (unsigned long)(now - peer.last_reply_ms), unsigned(peer.tec),
                    unsigned(peer.rec), unsigned(peer.fault), unsigned(peer.synchronized));
            }
        }
    } else {
        Serial.printf(" requests=%lu", (unsigned long)requests);
    }
    Serial.printf("\n");
}

void setup() {
    Serial.begin(115200);
    can.begin();
    can.setTX(DEF);
    can.setRX(DEF);
    can.setBaudRate(250000);
}

void loop() {
    CAN_message_t msg;
    while (can.read(msg)) handleFrame(msg);
    static uint32_t last_report = 0;
    const uint32_t now = millis();
    if (uint32_t(now - last_report) < PROBE_PERIOD_MS) return;
    last_report = now;
    if (CAN_BUS_TEST_NODE == 0) probeAll();
    printStatus(now);
}
