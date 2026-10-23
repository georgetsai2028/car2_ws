#include <cassert>
#include "../firmware/teensys/steer.cpp"
static uint8_t seq = 0;
static void command(bool estop, uint8_t flags, int8_t position) {
    CAN_message_t msg;
    uint8_t payload[3] = {(uint8_t)estop, flags, (uint8_t)position};
    dbw_pack_cmd(msg, DBW_ID_STEER_CMD_V2, seq, payload, 3);
    can.incoming.push_back(msg);
    mock_now += 10;
    loop();
}
int main() {
    setup();
    assert(setpointToSteps(-1.0f) == -8250 && setpointToSteps(1.0f) == 10000);
    assert(setpointToSteps(-5.0f) == -8250 && setpointToSteps(5.0f) == 10000);
    assert(setpointToSteps(0.0f) == 0);
    assert(stepper.minimumPulseUs == 25);
    for (int i = 0; i < 20; ++i) { mock_now += 10; loop(); }
    assert(stale && !homed && stepper.pulses == 0);

    // The old two-byte steering protocol cannot enable a v2 node.
    CAN_message_t legacy;
    uint8_t old_payload[2] = {0, 127};
    uint8_t old_seq = 0;
    dbw_pack_cmd(legacy, DBW_ID_STEER_CMD, old_seq, old_payload, 2);
    can.incoming.push_back(legacy); loop();
    assert(!rx.inited && stepper.pulses == 0);

    CAN_message_t corrupt;
    uint8_t corrupt_seq = 0;
    uint8_t active_payload[3] = {0, 0, 127};
    dbw_pack_cmd(corrupt, DBW_ID_STEER_CMD_V2, corrupt_seq, active_payload, 3);
    corrupt.buf[4] ^= 1; // bad CRC must not refresh or enable the node
    can.incoming.push_back(corrupt); loop();
    assert(!rx.inited && stepper.pulses == 0);

    command(true, 0, -51); // original startup regression: left trim under E-stop
    assert(cmd_estop && stepper.pulses == 0);
    uint8_t duplicate_seq = rx.last_seq;
    CAN_message_t duplicate;
    dbw_pack_cmd(duplicate, DBW_ID_STEER_CMD_V2, duplicate_seq, active_payload, 3);
    can.incoming.push_back(duplicate); loop();
    assert(cmd_estop && stepper.pulses == 0); // duplicate counter cannot clear E-stop
    command(false, 0, 127); // active commands cannot invent a position reference
    assert(!homed && stepper.pulses == 0);
    command(false, DBW_STEER_REFERENCE, 0);
    assert(!homed);

    mock_pins[cfg::PIN_LIM_L] = HIGH;
    command(true, DBW_STEER_REFERENCE, 0);
    assert(!homed); // cannot zero at an end stop
    mock_pins[cfg::PIN_LIM_L] = LOW;
    command(true, DBW_STEER_REFERENCE, 0);
    assert(homed && stepper.currentPosition() == 0 && stepper.pulses == 0);

    // Travel is opposite the mounting location: rightward linkage approaches
    // the physically left switch, GPIO18. The opposite stop is GPIO17.
    mock_pins[17] = HIGH;
    assert(!limitR() && limitL());
    mock_pins[17] = LOW;
    mock_pins[18] = HIGH;
    assert(limitR() && !limitL());
    mock_pins[18] = LOW;

    // Either end switch returns to the existing Z without losing reference.
    // Held outward input stays overridden until the input returns to zero.
    for (int pin : {17, 18}) {
        const int direction = pin == 18 ? 1 : -1;
        command(true, DBW_STEER_REFERENCE, 0);
        stepper.setCurrentPosition(direction * 5);
        const unsigned pulses_before_limit = stepper.pulses;
        mock_pins[pin] = HIGH;
        command(false, 0, (int8_t)(direction * 127));
        assert(homed && energized && return_to_center);
        assert(stepper.pulses == pulses_before_limit + 1);
        assert(stepper.currentPosition() == direction * 4);
        // E-stop must interrupt the automatic return without inventing a new Z.
        const unsigned before_estop = stepper.pulses;
        command(true, 0, 0);
        assert(homed && return_to_center && stepper.pulses == before_estop);
        mock_pins[pin] = LOW;
        for (int i = 0; i < 8; ++i) command(false, 0, (int8_t)(direction * 127));
        assert(homed && return_to_center && stepper.currentPosition() == 0);
        assert(stepper.pulses == pulses_before_limit + 5);
        command(false, 0, 0); // input released; resume normal steering
        assert(!return_to_center && homed && stepper.currentPosition() == 0);
        command(false, 0, (int8_t)(direction * 127));
        assert(stepper.currentPosition() == direction);

        // A switch on the wrong side of the recorded Z cannot choose a safe
        // return direction. A switch still pressed at Z also stops.
        command(true, DBW_STEER_REFERENCE, 0);
        stepper.setCurrentPosition(-direction * 5);
        const unsigned before_invalid = stepper.pulses;
        mock_pins[pin] = HIGH;
        command(false, 0, (int8_t)(direction * 127));
        assert(!homed && !energized && stepper.pulses == before_invalid);
        command(true, DBW_STEER_REFERENCE, 0);
        assert(!homed); // cannot re-reference at a pressed switch
        mock_pins[pin] = LOW;
        command(true, DBW_STEER_REFERENCE, 0);
        mock_pins[pin] = HIGH;
        command(false, 0, 0);
        assert(!homed && !energized && stepper.currentPosition() == 0);
        mock_pins[pin] = LOW;
    }
    command(true, DBW_STEER_REFERENCE, 0);
    stepper.setCurrentPosition(5);
    mock_pins[18] = HIGH;
    command(false, 0, 127);
    assert(return_to_center && homed);
    const unsigned before_return_timeout = stepper.pulses;
    mock_now += cfg::WATCHDOG_MS + 1; loop();
    assert(stale && !homed && !return_to_center && stepper.pulses == before_return_timeout);
    mock_pins[18] = LOW;
    command(false, 0, 0);
    assert(!homed && stepper.pulses == before_return_timeout);
    command(true, DBW_STEER_REFERENCE, 0);
    const unsigned pulses_after_limit_tests = stepper.pulses;

    const unsigned long delay_before_pulse = mock_microsecond_delays;
    command(false, 0, 64);
    assert(stepper.pulses == pulses_after_limit_tests + 1 && stepper.currentPosition() > 0);
    assert(mock_microsecond_delays - delay_before_pulse == 35);
    assert(mock_pins[16] == LOW && mock_pins[15] == LOW);
    unsigned before = stepper.pulses;
    long held = stepper.currentPosition();
    command(true, DBW_STEER_CENTER, -127);
    assert(stepper.pulses == before && stepper.currentPosition() == held);
    for (int i = 0; i < 10; ++i) command(true, 0, -127);
    assert(stepper.pulses == before);

    command(false, 0, -64);
    assert(stepper.pulses == before + 1);
    assert(mock_pins[16] == HIGH && mock_pins[15] == LOW);
    before = stepper.pulses;
    mock_now += cfg::WATCHDOG_MS + 1; loop();
    assert(stale && !homed && stepper.pulses == before);
    command(false, 0, 127);
    assert(stepper.pulses == before); // reconnection needs explicit reference

    command(true, DBW_STEER_REFERENCE, 0);
    assert(homed);
    mock_pins[cfg::PIN_LIM_L] = mock_pins[cfg::PIN_LIM_R] = HIGH;
    command(false, 0, 127);
    assert(!homed && stepper.pulses == before);
    puts("PASS steering: end-switch Z return, held-input override, invalid switch stop, E-stop, reference and watchdog");
}
