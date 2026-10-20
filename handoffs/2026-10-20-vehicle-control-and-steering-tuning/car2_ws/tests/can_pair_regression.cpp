// Run the actual sender and responder together. Only the CAN wire is mocked.
#include <cassert>
#include <Arduino.h>
#include <FlexCAN_T4.h>
namespace sender {
#define CAN_LINK_SENDER 1
#include "../firmware/teensys/can_link_test.cpp"
#undef CAN_LINK_SENDER
}
namespace responder {
#define CAN_LINK_SENDER 0
#include "../firmware/teensys/can_link_test.cpp"
#undef CAN_LINK_SENDER
}
template<class A, class B> void transfer(A& from, B& to) {
    for (const auto& msg : from.outgoing) to.incoming.push_back(msg);
    from.outgoing.clear();
}
int main() {
    sender::setup(); responder::setup();
    // Cross the 32-bit sequence wrap as well as ordinary exchanges.
    sender::sequence = UINT32_MAX - 5;
    for (unsigned i = 1; i <= 100; ++i) {
        mock_now += 1000;
        sender::loop();
        assert(sender::can.outgoing.size() == 1);
        assert(sender::can.outgoing[0].id == responder::REQUEST_ID);
        transfer(sender::can, responder::can);
        responder::loop();
        assert(responder::can.outgoing.size() == 1);
        assert(responder::can.outgoing[0].id == sender::REPLY_ID);
        transfer(responder::can, sender::can);
        sender::loop();
        assert(sender::matched == i && responder::matched == i);
    }
    assert(sender::tx_busy == 0 && responder::tx_busy == 0);
    puts("PASS actual CAN diagnostic pair: 100 matching PING/PONG exchanges, including sequence wrap");
}
