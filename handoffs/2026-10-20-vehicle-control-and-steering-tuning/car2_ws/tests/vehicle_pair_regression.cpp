// Test real master packing and real steering decoding/interlocks together.
#include <cassert>
#include <cctype>
#include <Arduino.h>
#include <AccelStepper.h>
#include "../firmware/teensys/dbw_can.h"
namespace master_fw {
#include "../firmware/teensys/master.cpp"
}
namespace steer_fw {
#include "../firmware/teensys/steer.cpp"
}
static void deliverCommands() {
    master_fw::sendCommands();
    for (const auto& msg : master_fw::can.outgoing) steer_fw::can.incoming.push_back(msg);
    master_fw::can.outgoing.clear();
    steer_fw::loop();
}
static void deliverSteeringStatus() {
    for (const auto& msg : steer_fw::can.outgoing) master_fw::can.incoming.push_back(msg);
    steer_fw::can.outgoing.clear();
    master_fw::handleStatus();
}
static void otherNodeStatus() {
    CAN_message_t throttle;
    throttle.id = DBW_ID_THROTTLE_STAT; throttle.len = 8;
    throttle.buf[4] = 80;
    master_fw::can.incoming.push_back(throttle);
    CAN_message_t brake;
    brake.id = DBW_ID_BRAKE_STAT; brake.len = 8; brake.buf[5] = 8;
    master_fw::can.incoming.push_back(brake);
    master_fw::handleStatus();
}
int main() {
    master_fw::setup(); steer_fw::setup();
    deliverCommands(); deliverSteeringStatus(); otherNodeStatus();
    assert(steer_fw::cmd_estop && !steer_fw::homed && steer_fw::stepper.pulses == 0);
    char reference[] = "Z";
    master_fw::parseLine(reference);
    mock_now += 10; deliverCommands();
    assert(steer_fw::homed && steer_fw::cmd_estop && steer_fw::stepper.pulses == 0);
    mock_now += 100; steer_fw::loop(); deliverSteeringStatus();
    assert(master_fw::fb_steer_homed == 1);

    char active[] = "A E=0 T=0 M=N B=1 S=0.1";
    master_fw::parseLine(active); deliverCommands();
    assert(!master_fw::g_estop && !steer_fw::cmd_estop);
    assert(steer_fw::stepper.pulses > 0 && steer_fw::setpoint > 0);
    // An operational switch hit returns to the existing Z and must not make
    // master latch E-stop through a lost steering reference.
    steer_fw::stepper.setCurrentPosition(5);
    mock_pins[18] = HIGH;
    mock_now += 100; deliverCommands(); deliverSteeringStatus();
    assert(steer_fw::return_to_center && steer_fw::homed);
    assert(steer_fw::stepper.currentPosition() == 4);
    assert(master_fw::fb_steer_homed == 1);
    mock_pins[18] = LOW;
    for (int i = 0; i < 6; ++i) {
        mock_now += 100;
        deliverCommands(); deliverSteeringStatus();
        assert(!master_fw::g_estop && master_fw::fb_steer_homed == 1);
    }
    assert(steer_fw::stepper.currentPosition() == 0);
    assert(steer_fw::return_to_center); // outward command is still held
    unsigned pulses = steer_fw::stepper.pulses;
    char stop[] = "A E=1 T=0 M=N B=1 S=-0.4";
    master_fw::parseLine(stop); deliverCommands();
    assert(steer_fw::cmd_estop && steer_fw::stepper.pulses == pulses);
    puts("PASS actual repaired master/steering pair: command CRC/ID/layout, reference, active steering, E-stop");
}
