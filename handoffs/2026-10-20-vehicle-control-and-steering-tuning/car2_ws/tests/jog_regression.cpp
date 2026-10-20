#include <cassert>
#include "../firmware/teensys/steer_usb_jog.cpp"
static void command(const char* text) {
    while (*text) Serial.input.push_back(*text++);
    Serial.input.push_back('\n'); loop();
}
int main() {
    setup(); loop();
    command("J 25"); assert(!jogMoving && jogStepper.pulses == 0);
    command("ARM"); assert(!jogArmed);
    mock_now += 1001; loop(); command("ARM"); assert(jogArmed);
    command("J 25");
    for (int i = 0; i < 40; ++i) { mock_now += 1; loop(); }
    assert(jogStepper.pulses == 25 && !jogMoving);
    command("J 26"); assert(!jogArmed && jogStepper.pulses == 25);

    command("ARM"); command("J -25");
    unsigned before = jogStepper.pulses;
    mock_pins[jog_cfg::LEFT_LIMIT] = HIGH; loop();
    assert(jogStepper.pulses == before && !jogMoving);
    command("J 25"); assert(jogMoving); // may move away from active left limit
    command("E"); assert(!jogArmed && !jogMoving);

    mock_pins[jog_cfg::LEFT_LIMIT] = LOW; loop();
    mock_now += 1001; loop(); command("ARM"); command("J 25");
    before = jogStepper.pulses;
    Serial.connected = false; loop();
    assert(jogStepper.pulses == before && !jogArmed && !jogMoving);
    Serial.connected = true;
    command("ARM"); command("J 25"); before = jogStepper.pulses;
    mock_now += 501; loop();
    assert(!jogMoving && !jogArmed && jogStepper.pulses == before);

    mock_pins[jog_cfg::LEFT_LIMIT] = mock_pins[jog_cfg::RIGHT_LIMIT] = HIGH;
    loop(); command("ARM"); assert(jogFault && !jogArmed);
    command("CLEAR"); assert(jogFault);
    mock_pins[jog_cfg::LEFT_LIMIT] = mock_pins[jog_cfg::RIGHT_LIMIT] = LOW;
    loop(); mock_now += 1001; loop(); command("CLEAR");
    assert(!jogFault && !jogArmed);
    command("ARM"); command("J 10garbage"); assert(!jogArmed && !jogMoving);
    puts("PASS USB jog: finite bursts, arm gate, limits, USB loss, timeout, faults, malformed commands");
}
