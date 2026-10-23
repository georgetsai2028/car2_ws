// steer.cpp — car2 drive-by-wire STEERING node (Teensy 4.1)
//
// Drives a NEMA-23 closed-loop stepper (StepperOnline CL57T driver) through a
// 20:1 right-angle gearbox (RYG23-G20-D8). Command on CAN 0x210, status 0x201.
//
// Steering v2: no steps until explicitly referenced and E-stop is cleared.
// Z on the master records the physically centered position while stopped.
// A single end switch returns steering to Z; E-stop, missing commands, and
// invalid limit states hold the current position.
//
// Why this rewrite (see DESIGN.md §3):
//   - the CL57T is closed-loop: it ramps current to its set max when it can't reach
//     the commanded position (a stall). So the #1 job is to NEVER command past a
//     mechanical stop. Travel below was selected on this car during supervised
//     USB commissioning and confirmed against the operator's straight center.
//   - limit switches were reading inverted (both asserted). Polarity is now a config
//     flag, and "both asserted" is a FAULT -> stop (no ping-pong).
//   - optional ENA output can de-energize the driver on fault/stale (old fw could not).
//
// Installed switch locations: physically right=17, physically left=18.
// Rightward steering approaches the physically LEFT switch, so the rightward
// travel guard is GPIO18, and the opposite travel guard is GPIO17. Either
// physical switch overrides steering input with a return to the recorded Z.
// encoder A0 (reported for calibration, not used as an unverified center).

#include <Arduino.h>
#include <AccelStepper.h>
#include "dbw_can.h"

// ----------------------------- Configuration -----------------------------
namespace cfg {
    constexpr uint8_t  PIN_STEP  = 15;
    constexpr uint8_t  PIN_DIR   = 16;
    constexpr uint8_t  PIN_LIM_R = 18;   // rightward travel: physically left switch
    constexpr uint8_t  PIN_LIM_L = 17;   // leftward travel: physically right switch
    constexpr uint8_t  PIN_ENC   = A0;   // center-homing encoder

    // ---- SAFETY GATES (bring-up) ----
    constexpr bool     MOTION_ENABLED   = true;   // normal operation
    constexpr bool     LIMIT_ACTIVE_LOW = false;  // NC switches: released LOW, pressed/open circuit HIGH

    // Optional driver-enable output wired to CL57T ENA+/ENA- (lets us de-energize).
    constexpr bool     USE_ENABLE_PIN   = false;  // set true only after you wire ENA
    constexpr uint8_t  PIN_ENA          = 20;     // pick a truly unused Teensy pin
    constexpr bool     ENA_ACTIVE_HIGH  = true;   // verify against your wiring
    constexpr uint32_t ENA_SETTLE_MS    = 250;    // CL57T: ENA must lead DIR ~200ms

    // Operator-selected boundaries before switch contact, from verified center.
    // V3 test coordinates: left -14000, straight -5750, right +4250.
    // These are commanded motor pulses, not measured wheel angles.
    constexpr long     STEPS_LEFT  = -8250;
    constexpr long     STEPS_RIGHT = 10000;
    constexpr long     STEP_BAND   =  10;
    constexpr float    CENTER_TRIM = 0.00f;   // correction is owned by teleop

    // Operational steering pulse-rate ceiling.
    constexpr float    MAX_SPEED  = 24000.0f; // 1.6x the previous 15000 pulses/s ceiling
    constexpr float    ACCEL      = 51200.0f; // 1.6 squared times the 20000 pulses/s^2 baseline
    constexpr unsigned MIN_PULSE_US = 25;     // leaves low time at 24000 pulses/s
    constexpr bool     DIR_INVERT     = true;   // USB jog verified positive steps turn wheels right

    // One-time travel calibration: set true, flash, watch serial for the measured
    // numbers, then paste them into STEPS_LEFT/RIGHT above and set this false.
    constexpr bool     CALIBRATE  = false;   // normal operation (not calibrating)
    constexpr float    CAL_SPEED  = 1500.0f; // steps/s during the sweep
    constexpr long     CAL_BACKOFF = 3000;   // steps to drive off the near limit (past switch bounce) before seeking the far one

    // Motor self-test: gentle limit-guarded oscillation using ONLY this board +
    // motor power (no CAN/teleop). Isolates the Teensy->driver->motor path.
    constexpr bool     STEP_TEST  = false;
    constexpr long     STEP_TEST_AMP        = 2000;  // steps each way
    constexpr uint32_t STEP_TEST_PERIOD_MS  = 2500;  // flip direction every 2.5 s

    constexpr uint32_t STATUS_PERIOD_MS = 100;
    constexpr uint32_t WATCHDOG_MS      = 150;
}

// Status flag bits (CAN 0x201 buf[5]) -- also relayed to PC telemetry as "sf".
enum : uint8_t {
    SF_LIM_R     = 0x01,
    SF_LIM_L     = 0x02,
    SF_NOTHOMED  = 0x04,
    SF_STALE     = 0x08,
    SF_FAULT     = 0x10,  // both limits asserted / implausible
    SF_MOVING    = 0x20,  // motion allowed this cycle
    SF_ENERGIZED = 0x40,  // driver enabled
    SF_ESTOP     = 0x80,
};

// ------------------------------- Globals ---------------------------------
// Respect DIR setup on each pulse, including reversals while decelerating.
// Use AccelStepper's actual direction rather than the target sign.
class SteeringStepper : public AccelStepper {
public:
    SteeringStepper() : AccelStepper(DRIVER, cfg::PIN_STEP, cfg::PIN_DIR) {}
protected:
    void step1(long) override {
        digitalWrite(cfg::PIN_DIR, (_direction != cfg::DIR_INVERT) ? HIGH : LOW);
        delayMicroseconds(10);
        digitalWrite(cfg::PIN_STEP, HIGH);
        delayMicroseconds(cfg::MIN_PULSE_US);
        digitalWrite(cfg::PIN_STEP, LOW);
    }
};
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
SteeringStepper stepper;
DbwRxState rx;

static float    setpoint  = 0.0f;
static bool     homed     = false;
static bool     cmd_estop = true;
static bool     return_to_center = false;

static uint32_t last_valid_cmd_ms = 0;
static bool     stale     = true;
static bool     energized = false;
static uint32_t energized_ms = 0;

// ------------------------------- Helpers ---------------------------------
static inline float clamp11(float v) { return (v < -1.0f) ? -1.0f : (v > 1.0f ? 1.0f : v); }

static inline bool limitR() {
    return digitalRead(cfg::PIN_LIM_R) == (cfg::LIMIT_ACTIVE_LOW ? LOW : HIGH);
}
static inline bool limitL() {
    return digitalRead(cfg::PIN_LIM_L) == (cfg::LIMIT_ACTIVE_LOW ? LOW : HIGH);
}

static void setEnergized(bool en) {
    if (en == energized) return;
    energized = en;
    if (en) energized_ms = millis();
    if (cfg::USE_ENABLE_PIN) {
        const bool level = (en == cfg::ENA_ACTIVE_HIGH);
        digitalWrite(cfg::PIN_ENA, level ? HIGH : LOW);
    }
}

static long setpointToSteps(float sp) {
    const float trimmed = clamp11(sp + cfg::CENTER_TRIM);
    long target = (trimmed < 0.0f)
                    ? (long)(trimmed * (float)(-cfg::STEPS_LEFT))
                    : (long)(trimmed * (float)cfg::STEPS_RIGHT);
    if (target < cfg::STEPS_LEFT)  target = cfg::STEPS_LEFT;
    if (target > cfg::STEPS_RIGHT) target = cfg::STEPS_RIGHT;
    return target;
}

// Diagnostic step-test guard: stop at either switch. Neither switch position nor the old travel constants
// establishes a measured endpoint on this car. A trip needs a new stationary
// center reference; it must never silently reset position to an old endpoint.
static void runMotionGuarded() {
    if (limitR() || limitL()) {
        stepper.setCurrentPosition(stepper.currentPosition());
        homed = false;
        setEnergized(false);
    } else {
        stepper.run();
    }
}

// Operational end-switch recovery preserves the operator's existing Z.
// At a pressed switch only motion toward Z and away from that end is allowed.
static void runOperationalMotion(bool lr, bool ll) {
    const long current = stepper.currentPosition();
    if (lr || ll) {
        const bool can_return = (lr && !ll && current > 0) ||
                                (ll && !lr && current < 0);
        if (!can_return) {
            stepper.setCurrentPosition(current);
            homed = false;
            return_to_center = false;
            setEnergized(false);
            return;
        }
        if (!return_to_center) {
            // Cancel any outward velocity before starting the inward return.
            stepper.setCurrentPosition(current);
            return_to_center = true;
        }
    }

    // A held steering key must not send the wheels straight back to the switch.
    if (return_to_center && current == 0 && !lr && !ll && setpoint == 0.0f) {
        return_to_center = false;
    }
    stepper.moveTo(return_to_center ? 0 : setpointToSteps(setpoint));
    stepper.run();
}

// --------------------------- Calibration ---------------------------------
// Slowly drives to one limit (=one end), backs off, drives to the other limit,
// measures the travel, recenters to the midpoint, then holds at center and
// prints the numbers to paste into STEPS_LEFT/RIGHT.
enum class Cal : uint8_t { INIT, SEEK_FIRST, BACKOFF, SEEK_SECOND, FINISH, IDLE };
static Cal  cal = Cal::INIT;
static int  cal_dir = 1;          // direction of the first seek (away from a parked limit)
static bool cal_cleared = false;  // have we moved off the starting limit yet
static long cal_range = 0;

static void printCalResult() {
    Serial.println("==== STEER CALIBRATION RESULT ====");
    Serial.print("range (limit-to-limit) = "); Serial.print(cal_range); Serial.println(" steps");
    Serial.print("STEPS_RIGHT = "); Serial.println(cal_range / 2);
    Serial.print("STEPS_LEFT  = "); Serial.println(-(cal_range / 2));
    Serial.println("(paste these to me; we'll set the steer direction by testing)");
}

static void runCalibration(uint32_t now) {
    setEnergized(true);
    if (cfg::USE_ENABLE_PIN && (now - energized_ms < cfg::ENA_SETTLE_MS)) return;

    const bool lr = limitR();
    const bool ll = limitL();

    switch (cal) {
        case Cal::INIT:
            // First move AWAY from a parked limit so we never drive into a stop.
            // At the left limit -> go right (-1); otherwise go left (+1).
            cal_dir = ll ? -1 : +1;
            cal_cleared = false;
            cal = Cal::SEEK_FIRST;
            break;

        case Cal::SEEK_FIRST:                   // drive cal_dir to the first end
            stepper.setSpeed((float)cal_dir * cfg::CAL_SPEED);
            if (!cal_cleared) {
                if (!lr && !ll) cal_cleared = true;   // moved clear of the start limit
                stepper.runSpeed();
            } else if (lr || ll) {
                stepper.setCurrentPosition(0);
                cal_cleared = false;
                cal = Cal::BACKOFF;
            } else {
                stepper.runSpeed();
            }
            break;

        case Cal::BACKOFF:                       // drive back a fixed distance off that limit
            stepper.setSpeed((float)(-cal_dir) * cfg::CAL_SPEED);
            if (labs(stepper.currentPosition()) >= cfg::CAL_BACKOFF) {
                cal = Cal::SEEK_SECOND;
            } else {
                stepper.runSpeed();
            }
            break;

        case Cal::SEEK_SECOND:                   // continue to the far end
            stepper.setSpeed((float)(-cal_dir) * cfg::CAL_SPEED);
            if (lr || ll) {
                const long cur = stepper.currentPosition();  // = -cal_dir * range
                cal_range = labs(cur);
                stepper.setCurrentPosition(cur - cur / 2);   // midpoint -> 0
                stepper.moveTo(0);
                printCalResult();
                cal = Cal::FINISH;
            } else {
                stepper.runSpeed();
            }
            break;

        case Cal::FINISH:                        // ease to center, then idle
            stepper.run();
            if (stepper.distanceToGo() == 0) cal = Cal::IDLE;
            break;

        case Cal::IDLE: {
            static uint32_t last_print = 0;
            if (now - last_print >= 1000) { last_print = now; printCalResult(); }
        } break;
    }
}

// --------------------------- Motor self-test -----------------------------
// Gently oscillates +/- STEP_TEST_AMP, limit-guarded (won't drive into a
// pressed limit). Prints pos so we can see whether the motor follows the steps.
static void runStepTest(uint32_t now) {
    setEnergized(true);
    if (cfg::USE_ENABLE_PIN && (now - energized_ms < cfg::ENA_SETTLE_MS)) return;

    static uint32_t t0 = 0;
    static long target = cfg::STEP_TEST_AMP;
    if (now - t0 >= cfg::STEP_TEST_PERIOD_MS) {
        t0 = now;
        target = -target;                 // flip direction
        stepper.moveTo(target);
    }
    runMotionGuarded();
}

// ---------------------------- CAN handling -------------------------------
static void handleCan() {
    CAN_message_t msg;
    while (can.read(msg)) {
        if (msg.id != DBW_ID_STEER_CMD_V2) continue;
        uint8_t p[3];
        if (dbw_unpack_cmd(msg, DBW_ID_STEER_CMD_V2, rx, p, 3)) {
            cmd_estop = (p[0] != 0u);
            if ((p[1] & DBW_STEER_REFERENCE) && cmd_estop && !limitR() && !limitL()) {
                // Operator has physically centered the wheels with motor power
                // off. Establish a reference without generating any step pulse.
                stepper.setCurrentPosition(0);
                homed = true;
                return_to_center = false;
                setpoint = 0.0f;
            } else if (!cmd_estop) {
                setpoint = (p[1] & DBW_STEER_CENTER) ? 0.0f :
                    clamp11((float)(int8_t)p[2] / 127.0f);
            }
            last_valid_cmd_ms = millis();
        }
    }
}

static void sendStatus(uint8_t flags) {
    const long cur = stepper.currentPosition();
    int8_t pct;
    if (cur < 0) pct = (int8_t)((cur * 127) / (-cfg::STEPS_LEFT));
    else         pct = (int8_t)((cur * 127) / cfg::STEPS_RIGHT);
    const int8_t sp_i8 = (int8_t)lroundf(clamp11(setpoint) * 127.0f);

    CAN_message_t m;
    m.id  = DBW_ID_STEER_STAT;
    m.len = 8;
    m.buf[0] = (uint8_t)(cur & 0xFFu);
    m.buf[1] = (uint8_t)((cur >> 8) & 0xFFu);
    m.buf[2] = (uint8_t)((cur >> 16) & 0xFFu);
    m.buf[3] = (uint8_t)((cur >> 24) & 0xFFu);
    m.buf[4] = (uint8_t)pct;
    m.buf[5] = flags;
    m.buf[6] = (uint8_t)sp_i8;
    m.buf[7] = homed ? 1u : 0u;
    can.write(m);
}

// ------------------------------ Setup/loop -------------------------------
void setup() {
    Serial.begin(115200);

    pinMode(cfg::PIN_LIM_R, INPUT_PULLUP);
    pinMode(cfg::PIN_LIM_L, INPUT_PULLUP);
    analogReadResolution(10);

    if (cfg::USE_ENABLE_PIN) {
        pinMode(cfg::PIN_ENA, OUTPUT);
        digitalWrite(cfg::PIN_ENA, cfg::ENA_ACTIVE_HIGH ? LOW : HIGH); // start de-energized
    }

    stepper.setPinsInverted(cfg::DIR_INVERT, false, false);
    stepper.setMinPulseWidth(cfg::MIN_PULSE_US);
    stepper.setMaxSpeed(cfg::MAX_SPEED);
    stepper.setAcceleration(cfg::ACCEL);
    stepper.setCurrentPosition(0);

    can.begin();
    can.setBaudRate(DBW_CAN_BITRATE);

    last_valid_cmd_ms = millis();
    Serial.print("STEER node up @ 250k  MOTION=");
    Serial.println(cfg::MOTION_ENABLED ? "ENABLED" : "DISABLED (diagnostic)");
    Serial.print("CENTER_TRIM=");
    Serial.println(cfg::CENTER_TRIM, 2);
}

void loop() {
    handleCan();
    // CAN reception timestamps commands with millis(). Sample now AFTER that
    // handler: an earlier snapshot can underflow if a millisecond ticks in RX.
    const uint32_t now = millis();
    stale = !rx.inited || (now - last_valid_cmd_ms > cfg::WATCHDOG_MS);

    const bool lr = limitR();
    const bool ll = limitL();
    const bool fault = lr && ll;                 // both asserted = implausible
    bool allow_motion = false;

    if (cfg::CALIBRATE) {
        runCalibration(now);
        allow_motion = true;                     // motor is moving during the sweep
    } else if (cfg::STEP_TEST) {
        runStepTest(now);
        allow_motion = true;
    } else {
        allow_motion = cfg::MOTION_ENABLED && homed && !cmd_estop && !stale && !fault;
        if (allow_motion) {
            setEnergized(true);
            // Respect the CL57T ENA-before-DIR settle time before stepping.
            const bool settled = !cfg::USE_ENABLE_PIN || (now - energized_ms >= cfg::ENA_SETTLE_MS);
            if (settled) {
                runOperationalMotion(lr, ll);
                allow_motion = homed;
            }
        } else {
            // Diagnostic / fault / stale: hold target at current, de-energize, no stepping.
            setEnergized(false);
            stepper.setCurrentPosition(stepper.currentPosition());
            if (stale || fault) {
                homed = false;
                return_to_center = false;
            }
        }
    }

    static uint32_t last_stat = 0;
    if (now - last_stat >= cfg::STATUS_PERIOD_MS) {
        last_stat = now;
        uint8_t flags = 0;
        if (lr)           flags |= SF_LIM_R;
        if (ll)           flags |= SF_LIM_L;
        if (!homed)       flags |= SF_NOTHOMED;
        if (stale)        flags |= SF_STALE;
        if (fault)        flags |= SF_FAULT;
        if (allow_motion) flags |= SF_MOVING;
        if (energized)    flags |= SF_ENERGIZED;
        if (cmd_estop)    flags |= SF_ESTOP;
        sendStatus(flags);

        // Human-readable limit-check line on the steer node's OWN USB serial.
        // Plug the PC into THIS Teensy and open a 115200 monitor to use it.
        Serial.print("R_limit="); Serial.print(lr ? "PRESSED" : "open");
        Serial.print("  L_limit="); Serial.print(ll ? "PRESSED" : "open");
        Serial.print("  FAULT=");   Serial.print(fault ? "YES" : "no");
        Serial.print("  pos=");     Serial.print(stepper.currentPosition());
        Serial.print("  enc=");     Serial.print(analogRead(cfg::PIN_ENC));
        Serial.print("  L_steps="); Serial.print(cfg::STEPS_LEFT);
        Serial.print("  R_steps="); Serial.print(cfg::STEPS_RIGHT);
        Serial.print("  fw=STEER_LIMITS_Z_RETURN_R02 ref=MANUAL pulse_us=");
        Serial.println(cfg::MIN_PULSE_US);
    }
}
