// Temporary USB-only steering diagnostic. No CAN or encoder dependency.
// Caleb car2 pins/polarity; no automatic sweep or homing.
// Commands: P=status, CLEAR=clear fault, ARM=enable bounded jogs,
// J -25..25=relative step burst, E=stop/disarm. Starts disarmed.
#include <Arduino.h>
#include <AccelStepper.h>

namespace jog_cfg {
    constexpr uint8_t STEP = 15, DIR = 16, RIGHT_LIMIT = 18, LEFT_LIMIT = 17;
    constexpr bool DIR_INVERT = true;
    constexpr long MAX_BURST = 25;
    constexpr uint32_t MAX_MOVE_MS = 500;
    constexpr uint32_t RELEASE_STABLE_MS = 1000;
}
AccelStepper jogStepper(AccelStepper::DRIVER, jog_cfg::STEP, jog_cfg::DIR);
static bool jogArmed = false, jogFault = false, jogMoving = false;
static bool jogRight = false, jogLeft = false, jogReleased = false;
static uint32_t jogReleasedAt = 0, jogStartedAt = 0;
static const char* jogEvent = "BOOT_DISARMED";

static void jogStop(const char* reason, bool disarm = true) {
    jogStepper.setCurrentPosition(jogStepper.currentPosition());
    jogMoving = false;
    if (disarm) jogArmed = false;
    jogEvent = reason;
}

static bool jogReleaseStable(uint32_t now) {
    return jogReleased && now - jogReleasedAt >= jog_cfg::RELEASE_STABLE_MS;
}

static void jogStatus() {
    Serial.printf("USB_JOG STEP=15 DIR=16 R18=%s L17=%s fault=%u armed=%u "
                  "moving=%u relative_steps=%ld event=%s\n",
                  jogRight ? "PRESSED" : "open", jogLeft ? "PRESSED" : "open",
                  (unsigned)jogFault, (unsigned)jogArmed, (unsigned)jogMoving,
                  jogStepper.currentPosition(), jogEvent);
}

static void jogCommand(char* line, uint32_t now) {
    if (!strcmp(line, "P")) { jogStatus(); return; }
    if (!strcmp(line, "E")) { jogStop("USB_STOP"); return; }
    if (!strcmp(line, "CLEAR")) {
        if (!jogMoving && jogReleaseStable(now)) {
            jogFault = false;
            jogStop("FAULT_CLEARED_DISARMED");
        }
        return;
    }
    if (!strcmp(line, "ARM")) {
        if (!jogMoving && !jogFault && jogReleaseStable(now) && Serial) {
            jogArmed = true;
            jogEvent = "ARMED";
        } else jogStop("ARM_REJECTED");
        return;
    }
    if (line[0] == 'J' && line[1] == ' ') {
        char* end = nullptr;
        long steps = strtol(line + 2, &end, 10);
        if (end == line + 2 || *end || !steps || steps < -jog_cfg::MAX_BURST ||
            steps > jog_cfg::MAX_BURST || !jogArmed || jogFault || jogMoving ||
            (steps > 0 && jogRight) || (steps < 0 && jogLeft)) {
            jogStop("JOG_REJECTED"); return;
        }
        jogStepper.moveTo(jogStepper.currentPosition() + steps);
        jogStartedAt = now;
        jogMoving = true;
        jogEvent = "JOGGING";
        return;
    }
    jogStop("INVALID_COMMAND");
}

void setup() {
    Serial.begin(115200);
    pinMode(jog_cfg::RIGHT_LIMIT, INPUT_PULLUP);
    pinMode(jog_cfg::LEFT_LIMIT, INPUT_PULLUP);
    jogStepper.setPinsInverted(jog_cfg::DIR_INVERT, false, false);
    jogStepper.setMaxSpeed(250.0f);
    jogStepper.setAcceleration(1500.0f);
    jogStepper.setCurrentPosition(0);
}

void loop() {
    const uint32_t now = millis();
    jogRight = digitalRead(jog_cfg::RIGHT_LIMIT) == HIGH;
    jogLeft = digitalRead(jog_cfg::LEFT_LIMIT) == HIGH;
    if (!jogRight && !jogLeft) {
        if (!jogReleased) jogReleasedAt = now;
        jogReleased = true;
    } else jogReleased = false;
    if (jogRight && jogLeft) { jogFault = true; jogStop("BOTH_LIMITS_FAULT"); }
    if (!Serial) jogStop("USB_DISCONNECTED");

    static char input[32];
    static uint8_t used = 0;
    static bool overflow = false;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r' || c == '\n') {
            if (used && !overflow) { input[used] = 0; jogCommand(input, now); }
            used = 0; overflow = false;
        } else if (used < sizeof(input) - 1 && !overflow) input[used++] = c;
        else { overflow = true; jogStop("INPUT_OVERFLOW"); }
    }
    if (jogMoving) {
        long delta = jogStepper.distanceToGo();
        if (!jogArmed || jogFault || !Serial) jogStop("INHIBITED");
        else if (now - jogStartedAt >= jog_cfg::MAX_MOVE_MS) jogStop("MOVE_TIMEOUT");
        else if ((delta > 0 && jogRight) || (delta < 0 && jogLeft)) jogStop("LIMIT_STOP", false);
        else {
            jogStepper.run();
            if (!jogStepper.distanceToGo()) {
                jogMoving = false;
                jogEvent = "BURST_DONE";
            }
        }
    }
    static uint32_t lastStatus = 0;
    if (Serial && now - lastStatus >= 100) { lastStatus = now; jogStatus(); }
}
