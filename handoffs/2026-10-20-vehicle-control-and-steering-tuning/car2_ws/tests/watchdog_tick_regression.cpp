// Reproduce a real clock tick while the CAN handler accepts a fresh command.
#include <cassert>
#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Encoder.h>
#include "../firmware/teensys/dbw_can.h"
namespace steering {
#include "../firmware/teensys/steer.cpp"
}
namespace throttle {
#include "../firmware/teensys/throttle.cpp"
}
namespace brake {
#include "../firmware/teensys/brake.cpp"
}

int main() {
    steering::setup(); throttle::setup(); brake::setup();
    uint8_t steer_seq = 0, throttle_seq = 0, brake_seq = 0;
    for (uint32_t start : {2000u, 0xFFFFFFFFu}) {
        mock_now = start;
        CAN_message_t message;
        uint8_t payload[3] = {1, DBW_STEER_REFERENCE, 0};
        dbw_pack_cmd(message, DBW_ID_STEER_CMD_V2, steer_seq, payload, 3);
        steering::can.incoming.push_back(message);
        steering::can.advanceOnNextReadMs = 1;
        steering::loop();
        printf("steering clock=%lu received=%lu stale=%d reference=%d\n",
               (unsigned long)millis(), (unsigned long)steering::last_valid_cmd_ms,
               steering::stale, steering::homed);
        fflush(stdout);
        assert(!steering::stale && steering::homed && steering::cmd_estop && steering::stepper.pulses == 0);
        mock_now += steering::cfg::WATCHDOG_MS + 1;
        steering::loop();
        assert(steering::stale && !steering::homed && steering::stepper.pulses == 0);

        mock_now = start;
        uint8_t throttle_payload[3] = {0, 0, 'N'};
        dbw_pack_cmd(message, DBW_ID_THROTTLE_CMD, throttle_seq, throttle_payload, 3);
        throttle::can.incoming.push_back(message);
        throttle::can.advanceOnNextReadMs = 1;
        throttle::loop();
        assert(!throttle::stale && !throttle::cmd_estop);
        mock_now += throttle::cfg::WATCHDOG_MS + 1;
        throttle::loop();
        assert(throttle::stale && throttle::cur_gear == throttle::Gear::N);

        mock_now = start;
        uint8_t brake_payload[2] = {0, 0};
        dbw_pack_cmd(message, DBW_ID_BRAKE_CMD, brake_seq, brake_payload, 2);
        brake::can.incoming.push_back(message);
        brake::can.advanceOnNextReadMs = 1;
        brake::loop();
        assert(!brake::stale && !brake::cmd_estop);
        mock_now += brake::cfg::WATCHDOG_MS + 1;
        brake::loop();
        assert(brake::stale);
    }
    // Throttle independently clamps a full-scale CAN request to 85%, even if
    // it comes from a source other than the capped master.
    uint8_t throttle_seq_cap = 0;
    throttle::rx = {};
    for (uint8_t requested : {uint8_t(140), uint8_t(217), uint8_t(255)}) {
        CAN_message_t message;
        uint8_t payload[3] = {0, requested, 'D'};
        dbw_pack_cmd(message, DBW_ID_THROTTLE_CMD, throttle_seq_cap, payload, 3);
        throttle::can.incoming.push_back(message);
        throttle::handleCan();
        const float expected = requested == 140 ? 140.0f / 255.0f : 0.85f;
        assert(fabsf(throttle::cmd_throttle - expected) < 0.00001f);
        throttle::stale = false;
        throttle::cur_gear = throttle::Gear::D;
        throttle::shift_phase = throttle::ShiftPhase::Idle;
        throttle::controlStep();
        assert(fabsf(throttle::applied_v - (0.8f + 2.5f * expected)) < 0.00001f);
    }
    puts("PASS actuator watchdogs and throttle 85% CAN/DAC ceiling");
}
