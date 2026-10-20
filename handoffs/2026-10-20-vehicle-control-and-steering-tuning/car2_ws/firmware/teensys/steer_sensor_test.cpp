// USB-only analog sensor retest within operator-selected travel bounds.
// No CAN, autonomous sweep, EEPROM writes or movement before explicit ZERO/ARM.
#include <Arduino.h>
#include <AccelStepper.h>
#include <cstring>
#include <cstdlib>

namespace limit_test_cfg {
// Rightward linkage travel approaches the physically LEFT switch (GPIO18).
// RIGHT/LEFT below name travel endpoints, not switch mounting locations.
constexpr uint8_t STEP = 15, DIR = 16, RIGHT = 18, LEFT = 17;
constexpr long MAX_BURST = 5000;
constexpr long SOFT_LEFT = -8250, SOFT_RIGHT = 10000;
constexpr unsigned int PULSE_US = 100;
constexpr uint32_t MOVE_TIMEOUT = 2500, LINK_TIMEOUT = 250, STABLE_MS = 500;
}
AccelStepper limitStepper(AccelStepper::DRIVER, limit_test_cfg::STEP, limit_test_cfg::DIR);
static bool armed = false, moving = false, fault = false;
static bool rightPressed = false, leftPressed = false, initializedInputs = false;
static bool haveRight = false, haveLeft = false, manualZero = false;
static long rightSteps = 0, leftSteps = 0;
static uint8_t escapePin = 0;
static int sensorRaw = 0;
static uint32_t stableSince = 0, moveStarted = 0, heartbeat = 0;
static const char* event = "BOOT_DISARMED";

static void stop(const char* why) {
    limitStepper.setCurrentPosition(limitStepper.currentPosition());
    moving = false; armed = false; escapePin = 0; event = why;
}
static void readInputs(uint32_t now) {
    sensorRaw = analogRead(A0);
    const bool r = digitalRead(limit_test_cfg::RIGHT) == HIGH;
    const bool l = digitalRead(limit_test_cfg::LEFT) == HIGH;
    if (!initializedInputs || r != rightPressed || l != leftPressed) stableSince = now;
    initializedInputs = true; rightPressed = r; leftPressed = l;
    if (r && l) { fault = true; stop("BOTH_LIMITS_FAULT"); }
    if (sensorRaw < 8 || sensorRaw > 4087) { fault = true; stop("SENSOR_RAIL_FAULT"); }
}
static void status() {
    Serial.printf("{\"fw\":\"STEER_SENSOR_TEST_V1\",\"pulseUs\":%u,\"maxBurst\":%ld,"
                  "\"leftBound\":-8250,\"rightBound\":10000,\"ms\":%lu,"
                  "\"rightStopPin\":18,\"leftStopPin\":17,\"R17\":%d,\"L18\":%d,"
                  "\"rightTravelLimit\":%d,\"leftTravelLimit\":%d,"
                  "\"armed\":%d,\"moving\":%d,\"fault\":%d,\"pos\":%ld,"
                  "\"target\":%ld,\"adc12\":%d,\"adcUsedForControl\":false,"
                  "\"rightHit\":%d,\"leftHit\":%d,\"rightSteps\":%ld,\"leftSteps\":%ld,"
                  "\"manualZero\":%d,\"event\":\"%s\"}\n",
                  limit_test_cfg::PULSE_US, limit_test_cfg::MAX_BURST,
                  (unsigned long)millis(),
                  leftPressed, rightPressed, rightPressed, leftPressed,
                  armed, moving, fault,
                  limitStepper.currentPosition(), limitStepper.currentPosition() + limitStepper.distanceToGo(),
                  sensorRaw, haveRight, haveLeft, rightSteps, leftSteps, manualZero, event);
}
static void command(const char* line, uint32_t now) {
    if (!strcmp(line, "P")) { status(); return; }
    if (!strcmp(line, "STOP")) { stop("USB_STOP"); return; }
    if (!strcmp(line, "K")) { if (moving) heartbeat = now; return; }
    if (!strcmp(line, "CLEAR")) {
        if (!moving && !rightPressed && !leftPressed && sensorRaw >= 8 && sensorRaw <= 4087 &&
            now - stableSince >= limit_test_cfg::STABLE_MS) {
            fault = false; stop("CLEARED_DISARMED");
        }
        return;
    }
    if (!strcmp(line, "ZERO")) {
        if (!moving && !armed && !fault && !rightPressed && !leftPressed &&
            now - stableSince >= limit_test_cfg::STABLE_MS) {
            // Operator must have confirmed physically straight wheels. No pulses.
            const long offset = limitStepper.currentPosition();
            if (haveRight) rightSteps -= offset;
            if (haveLeft) leftSteps -= offset;
            limitStepper.setCurrentPosition(0); manualZero = true; event = "MANUAL_ZERO";
        } else stop("ZERO_REFUSED");
        return;
    }
    if (!strcmp(line, "ARM")) {
        // At one switch, allow arming so a command can retreat from that switch.
        if (!moving && manualZero && !fault && Serial && now - stableSince >= limit_test_cfg::STABLE_MS) {
            armed = true; event = "ARMED";
        } else stop("ARM_REFUSED");
        return;
    }
    if (line[0] == 'J' && line[1] == ' ') {
        char* tail = nullptr;
        const long amount = strtol(line + 2, &tail, 10);
        if (tail == line + 2 || *tail || !amount || amount < -limit_test_cfg::MAX_BURST ||
            amount > limit_test_cfg::MAX_BURST || !manualZero || !armed || moving || fault || !Serial ||
            limitStepper.currentPosition() + amount < limit_test_cfg::SOFT_LEFT ||
            limitStepper.currentPosition() + amount > limit_test_cfg::SOFT_RIGHT ||
            (amount > 0 && rightPressed) || (amount < 0 && leftPressed)) {
            stop("JOG_REFUSED"); return;
        }
        // DIR is inverted as in the operational steering firmware. Establish
        // direction before the first pulse rather than relying on step1's
        // near-simultaneous DIR/STEP writes.
        digitalWrite(limit_test_cfg::DIR, amount > 0 ? LOW : HIGH);
        delayMicroseconds(10);
        escapePin = rightPressed ? limit_test_cfg::RIGHT :
                    leftPressed ? limit_test_cfg::LEFT : 0;
        limitStepper.moveTo(limitStepper.currentPosition() + amount);
        moveStarted = heartbeat = now; moving = true; event = "JOGGING";
        return;
    }
    stop("INVALID_COMMAND");
}
static void tick(uint32_t now) {
    if (!Serial) { stop("USB_DISCONNECTED"); return; }
    if (!moving) return;
    const long remaining = limitStepper.distanceToGo();
    if (fault || !armed) { stop("INHIBITED"); return; }
    if ((remaining > 0 && rightPressed) || (remaining < 0 && leftPressed)) {
        if (remaining > 0) { haveRight = true; rightSteps = limitStepper.currentPosition(); }
        else { haveLeft = true; leftSteps = limitStepper.currentPosition(); }
        stop(remaining > 0 ? "RIGHT_LIMIT_STOP" : "LEFT_LIMIT_STOP"); return;
    }
    // Ignore only an already-pressed switch while retreating away from it.
    // Once it releases, ANY newly hit switch must stop this calibration jog.
    if (escapePin && digitalRead(escapePin) == LOW) escapePin = 0;
    if ((rightPressed || leftPressed) && !escapePin) {
        fault = true; stop("UNEXPECTED_LIMIT_STOP"); return;
    }
    if (now - heartbeat > limit_test_cfg::LINK_TIMEOUT) { stop("KEEPALIVE_TIMEOUT"); return; }
    if (now - moveStarted > limit_test_cfg::MOVE_TIMEOUT) { stop("MOVE_TIMEOUT"); return; }
    limitStepper.run();
    if (!limitStepper.distanceToGo()) stop("BURST_DONE");
}
void setup() {
    Serial.begin(115200);
    pinMode(limit_test_cfg::RIGHT, INPUT_PULLUP); pinMode(limit_test_cfg::LEFT, INPUT_PULLUP);
    analogReadResolution(12);
    limitStepper.setPinsInverted(true, false, false);
    // Match the reference Steer_Test.cpp's 100 us HIGH pulse. The driver's
    // physical microstep setting is independent of this pulse duration.
    limitStepper.setMinPulseWidth(limit_test_cfg::PULSE_US);
    limitStepper.setMaxSpeed(5000); limitStepper.setAcceleration(20000);
    limitStepper.setCurrentPosition(0);
    readInputs(millis());
}
void loop() {
    uint32_t now = millis(); readInputs(now); tick(now);
    static char input[40]; static uint8_t used = 0; static bool overflow = false;
    while (Serial.available()) {
        const char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (used && !overflow) { input[used] = 0; command(input, millis()); }
            used = 0; overflow = false;
        } else if (!overflow && used < sizeof(input) - 1) input[used++] = c;
        else { overflow = true; stop("INPUT_OVERFLOW"); }
    }
    now = millis(); readInputs(now); tick(now);
    static uint32_t last = 0;
    if (Serial && now - last >= 20) { last = now; status(); }
}
