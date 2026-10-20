#include <cassert>
#include "../firmware/teensys/steer_limit_test.cpp"
static void send(const char* value) { command(value, millis()); }
static void advance(uint32_t ms) { mock_now += ms; loop(); }
int main() {
    setup(); advance(600); assert(!armed && !moving && limitStepper.pulses == 0);
    assert(limitStepper.minimumPulseUs == 100);
    send("J 500"); assert(!moving && !armed);
    send("ARM"); send("J 5000"); assert(moving && limitStepper.distanceToGo() == 5000);
    assert(mock_pins[16] == LOW);
    unsigned before = limitStepper.pulses;
    mock_pins[18] = HIGH; advance(1);
    assert(!moving && !armed && haveRight && !fault && limitStepper.pulses == before);
    advance(600); send("ARM"); send("J 100"); assert(!moving); // Cannot push into right switch.
    send("ARM"); send("J -500"); assert(moving); // Escape right switch is allowed.
    assert(mock_pins[16] == HIGH);
    advance(1); assert(limitStepper.pulses > before);
    send("STOP"); mock_pins[18] = LOW; advance(1); advance(600);
    send("ZERO"); assert(manualZero && limitStepper.currentPosition() == 0);
    send("ARM"); send("J -500"); before = limitStepper.pulses;
    mock_pins[17] = HIGH; advance(1);
    assert(!moving && haveLeft && !fault && limitStepper.pulses == before);
    advance(600); send("ARM"); send("J 500"); assert(moving);
    before = limitStepper.pulses; advance(251);
    assert(!moving && !armed && limitStepper.pulses == before);
    mock_pins[17] = LOW; advance(1); advance(600); send("ARM"); send("J 5001"); assert(!moving);
    send("ARM"); send("J -5001"); assert(!moving);
    send("ARM"); send("J 10bad"); assert(!moving);
    send("ARM"); send("J 10"); before = limitStepper.pulses;
    Serial.connected = false; advance(1); assert(!moving && limitStepper.pulses == before);
    Serial.connected = true; mock_pins[17] = mock_pins[18] = HIGH;
    advance(1); send("ARM"); assert(fault && !armed);
    send("CLEAR"); assert(fault);
    mock_pins[17] = mock_pins[18] = LOW; advance(1); advance(600); send("CLEAR"); assert(!fault);
    // Keepalives do not extend the independent finite move timeout.
    send("ARM"); send("J 1000"); before = limitStepper.pulses;
    mock_now += 2501; heartbeat = mock_now; tick(mock_now);
    assert(!moving && !armed && limitStepper.pulses == before);
    // A small normal burst completes and disarms without repeat/queued motion.
    send("ARM"); send("J 10");
    for (int i = 0; i < 20; ++i) { mock_now++; send("K"); loop(); }
    assert(!moving && !armed && limitStepper.currentPosition() == 10);
    // A new switch hit also stops when it contradicts the expected travel map.
    for (int direction : {1, -1}) {
        send("ARM"); send(direction > 0 ? "J 500" : "J -500");
        before = limitStepper.pulses;
        const int unexpected = direction > 0 ? 17 : 18;
        mock_pins[unexpected] = HIGH; advance(1);
        assert(fault && !moving && !armed && limitStepper.pulses == before);
        assert(!strcmp(event, "UNEXPECTED_LIMIT_STOP"));
        mock_pins[unexpected] = LOW; advance(1); advance(600); send("CLEAR"); assert(!fault);
    }
    puts("PASS steering limit test: right travel stops GPIO18, left travel GPIO17, unexpected switch stops either way, retreat, finite bursts and timeouts");
}
