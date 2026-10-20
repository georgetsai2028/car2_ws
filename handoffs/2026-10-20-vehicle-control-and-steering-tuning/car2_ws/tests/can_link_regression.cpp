#include <cassert>
#include "../firmware/teensys/can_link_test.cpp"
int main() {
    setup(); loop();
    CAN_message_t msg;
    msg.len = 8;
    if (CAN_LINK_SENDER) {
        assert(can.outgoing.size() == 1 && can.outgoing.back().id == REQUEST_ID);
        assert(memcmp(can.outgoing.back().buf, "PING", 4) == 0);
        msg = can.outgoing.back(); msg.id = REPLY_ID;
        memcpy(msg.buf, "PONG", 4);
        msg.buf[4] ^= 1;
        can.incoming.push_back(msg); loop(); assert(matched == 0); // wrong sequence
        msg.buf[4] ^= 1;
        can.incoming.push_back(msg); loop(); assert(matched == 1);
        can.incoming.push_back(msg); loop(); assert(matched == 1); // duplicate reply
    } else {
        assert(can.outgoing.empty()); // responder never commands spontaneously
        msg.id = REQUEST_ID; memcpy(msg.buf, "PING", 4); msg.buf[4] = 42;
        msg.flags.extended = true;
        can.incoming.push_back(msg); loop(); assert(can.outgoing.empty());
        msg.flags.extended = false; msg.len = 2;
        can.incoming.push_back(msg); loop(); assert(can.outgoing.empty());
        msg.len = 8;
        can.incoming.push_back(msg); loop();
        assert(matched == 1 && can.outgoing.size() == 1);
        assert(can.outgoing.back().id == REPLY_ID);
        assert(memcmp(can.outgoing.back().buf, "PONG", 4) == 0);
        assert(readSequence(can.outgoing.back()) == 42);
    }
    mock_ecr = 7 | (3 << 8); mock_esr = 2 << 4;
    mock_now += 1000; loop();
    assert(Serial.output.find("TEC=7 REC=3 fault=2") != std::string::npos);
    puts(CAN_LINK_SENDER ? "PASS CAN link sender: matching reply required" :
                          "PASS CAN link responder: validated request echo");
}
