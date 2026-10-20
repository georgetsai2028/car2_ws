// Four actual firmware roles, connected through a mocked CAN wire.
#include <cassert>
#include <Arduino.h>
#include <FlexCAN_T4.h>
namespace master {
#define CAN_BUS_TEST_NODE 0
#include "../firmware/teensys/can_bus_test.cpp"
#undef CAN_BUS_TEST_NODE
}
namespace throttle {
#define CAN_BUS_TEST_NODE 1
#include "../firmware/teensys/can_bus_test.cpp"
#undef CAN_BUS_TEST_NODE
}
namespace steering {
#define CAN_BUS_TEST_NODE 2
#include "../firmware/teensys/can_bus_test.cpp"
#undef CAN_BUS_TEST_NODE
}
namespace brake {
#define CAN_BUS_TEST_NODE 3
#include "../firmware/teensys/can_bus_test.cpp"
#undef CAN_BUS_TEST_NODE
}

template<class A, class B> void transfer(A& from, B& to) {
    for (const auto& msg : from.outgoing) to.incoming.push_back(msg);
}

static void exchange(bool include_brake) {
    mock_now += 1000;
    master::loop();
    assert(master::can.outgoing.size() == 3);
    // Every responder sees the entire bus, rather than its own request only.
    transfer(master::can, throttle::can);
    transfer(master::can, steering::can);
    transfer(master::can, brake::can);
    master::can.outgoing.clear();
    throttle::loop(); steering::loop();
    if (include_brake) brake::loop();
    else brake::can.incoming.clear();
    assert(throttle::can.outgoing.size() == 1);
    assert(steering::can.outgoing.size() == 1);
    assert(brake::can.outgoing.size() == unsigned(include_brake));
    transfer(throttle::can, master::can);
    transfer(steering::can, master::can);
    transfer(brake::can, master::can);
    throttle::can.outgoing.clear(); steering::can.outgoing.clear();
    brake::can.outgoing.clear();
    master::loop();
}

int main() {
    mock_esr = master::SYNCH_MASK;
    master::setup(); throttle::setup(); steering::setup(); brake::setup();
    assert(master::can.outgoing.empty() && throttle::can.outgoing.empty() &&
           steering::can.outgoing.empty() && brake::can.outgoing.empty());
    assert(strcmp(master::peerState(master::peers[3], mock_now), "NO_REPLY") == 0);
    master::sequence = UINT32_MAX - 5;
    for (unsigned i = 1; i <= 100; ++i) {
        exchange(true);
        for (unsigned node = 1; node <= 3; ++node) {
            assert(master::peers[node].replies == i);
            assert(strcmp(master::peerState(master::peers[node], mock_now), "OK") == 0);
        }
        assert(throttle::requests == i && steering::requests == i && brake::requests == i);
    }

    // A brake reply cannot impersonate steering or satisfy a newer request.
    CAN_message_t reply;
    reply.id = master::REPLY_BASE + 3; reply.len = 8;
    reply.buf[0] = master::REPLY_MAGIC; reply.buf[1] = master::VERSION_BITS | 4;
    master::writeU32(reply.buf + 2, master::peers[3].expected_sequence);
    master::handleFrame(reply); // duplicate of the latest accepted sequence
    assert(master::peers[3].replies == 100 && master::peers[2].replies == 100);
    master::peers[3].reply_seen = false;
    reply.flags.extended = true; master::handleFrame(reply);
    reply.flags.extended = false; reply.flags.remote = true; master::handleFrame(reply);
    reply.flags.remote = false; reply.len = 7; master::handleFrame(reply);
    reply.len = 8; reply.buf[1] |= 8; master::handleFrame(reply); // invalid protocol flags
    reply.buf[1] &= ~8; reply.buf[4] ^= 1; master::handleFrame(reply); // stale sequence
    assert(master::peers[3].replies == 100);
    reply.buf[4] ^= 1;
    master::peers[3].reply_seen = true;

    // Missing brake must not prevent independently identifying the other nodes.
    exchange(false); exchange(false); exchange(false);
    assert(master::peers[1].replies == 103 && master::peers[2].replies == 103);
    assert(master::peers[3].replies == 100);
    assert(strcmp(master::peerState(master::peers[3], mock_now), "STALE") == 0);
    exchange(true);
    assert(master::peers[3].replies == 101);
    assert(strcmp(master::peerState(master::peers[3], mock_now), "OK") == 0);

    // Propagate real responder counter/status fields to the master display.
    master::probeAll();
    transfer(master::can, brake::can); master::can.outgoing.clear();
    mock_ecr = 150 | (9 << 8); mock_esr = master::SYNCH_MASK | (1 << 4);
    brake::loop();
    transfer(brake::can, master::can); brake::can.outgoing.clear();
    master::loop();
    assert(master::peers[3].tec == 150 && master::peers[3].rec == 9);
    assert(master::peers[3].fault == 1 && master::peers[3].synchronized);
    assert(strcmp(master::peerState(master::peers[3], mock_now), "CAN_ERROR") == 0);
    Serial.output.clear(); master::printStatus(mock_now);
    assert(Serial.output.find("BRAKE=CAN_ERROR") != std::string::npos);
    assert(Serial.output.find("TEC=150 REC=9 fault=1") != std::string::npos);

    // Freshness uses unsigned elapsed time across millis() wrap.
    master::peers[3].last_reply_ms = UINT32_MAX - 10;
    master::peers[3].tec = master::peers[3].rec = master::peers[3].fault = 0;
    assert(strcmp(master::peerState(master::peers[3], 20), "OK") == 0);
    assert(strcmp(master::peerState(master::peers[3], 3000), "STALE") == 0);
    puts("PASS four-node CAN test: unique roles, 100 exchanges, missing-node isolation, error reporting, sequence/time wrap");
}
