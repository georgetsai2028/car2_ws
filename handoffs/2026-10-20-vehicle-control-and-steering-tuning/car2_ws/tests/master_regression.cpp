#include <cassert>
#include "../firmware/teensys/master.cpp"
static void steerPayload(uint8_t* payload) {
    const auto& msg = can.outgoing.back();
    assert(msg.id == DBW_ID_STEER_CMD_V2 && msg.len == 5);
    DbwRxState state = {};
    assert(dbw_unpack_cmd(msg, DBW_ID_STEER_CMD_V2, state, payload, 3));
}
int main() {
    setup();
    g_steer = -0.4f; g_center = true;
    sendCommands();
    uint8_t payload[3]; steerPayload(payload);
    assert(payload[0] == 1 && payload[1] == 0 && payload[2] == 0);
    char reference[] = "Z";
    parseLine(reference); sendCommands(); steerPayload(payload);
    assert(payload[0] == 1 && payload[1] == DBW_STEER_REFERENCE);
    sendCommands(); steerPayload(payload);
    assert(payload[1] == 0); // reference is a one-shot

    fb_thr_status_ms = fb_steer_status_ms = fb_brake_status_ms = millis();
    fb_steer_homed = 1; g_estop = false; g_steer = 0.5f;
    sendCommands(); steerPayload(payload);
    assert(!g_estop && payload[0] == 0 && (int8_t)payload[2] == 64);
    parseLine(reference); sendCommands(); steerPayload(payload);
    assert(payload[1] == 0); // zeroing while active is forbidden

    // Missing node status remains telemetry-only; it does not latch E-stop.
    uint32_t* times[] = {&fb_thr_status_ms, &fb_steer_status_ms, &fb_brake_status_ms};
    for (auto timer : times) {
        fb_thr_status_ms = fb_steer_status_ms = fb_brake_status_ms = millis();
        *timer = 0; g_estop = false; g_throttle = 0.5f; g_mode = 'D';
        sendCommands(); steerPayload(payload);
        assert(!g_estop && g_throttle == 0.5f && g_mode == 'D');
        assert(payload[0] == 0 && (int8_t)payload[2] == 64);
    }
    // The 85% target passes through; larger requests are capped on the CAN wire.
    fb_thr_status_ms = millis(); fb_thr_gear = 1; g_brake = 0.0f;
    DbwRxState throttle_rx = {};
    for (float requested : {0.55f, 0.85f, 1.0f}) {
        can.outgoing.clear(); g_throttle = requested;
        sendCommands();
        uint8_t throttle_payload[3];
        assert(dbw_unpack_cmd(can.outgoing.front(), DBW_ID_THROTTLE_CMD,
                              throttle_rx, throttle_payload, 3));
        const float expected = requested > 0.85f ? 0.85f : requested;
        assert(throttle_payload[0] == 0 && throttle_payload[1] == toU8(expected));
    }
    auto expectThrottle = [](char mode, float demand) {
        fb_thr_status_ms = millis();
        can.outgoing.clear(); sendCommands();
        DbwRxState state = {};
        uint8_t p[3];
        assert(dbw_unpack_cmd(can.outgoing.front(), DBW_ID_THROTTLE_CMD, state, p, 3));
        assert(p[1] == toU8(demand) && p[2] == mode);
    };
    // Both reversal directions hold for 500 ms, then wait for gear feedback.
    for (char previous : {'D', 'R'}) {
        const char next = previous == 'D' ? 'R' : 'D';
        resetDirectionGuard(); g_last_motion_gear = previous;
        fb_thr_gear = previous == 'D' ? 1 : 3;
        g_mode = next; g_throttle = 0.3f; g_brake = 0;
        expectThrottle('N', 0); assert(g_dir_guard_active);
        g_mode = 'N'; g_throttle = 0; g_brake = 1; fb_thr_gear = 0;
        mock_now += 499;
        expectThrottle('N', 0); assert(g_dir_guard_active);
        mock_now += 1;
        expectThrottle('N', 0); assert(!g_dir_guard_active);
        g_mode = next; g_throttle = 0.3f; g_brake = 0;
        expectThrottle(next, 0); // still Neutral, so torque must wait
        fb_thr_gear = next == 'D' ? 1 : 3;
        expectThrottle(next, 0.3f);

        // A completed teleop Neutral/brake dwell must not add a second hold.
        resetDirectionGuard(); g_last_motion_gear = previous;
        fb_thr_gear = previous == 'D' ? 1 : 3;
        g_mode = 'N'; g_throttle = 0; g_brake = 1;
        expectThrottle('N', 0);
        mock_now += 500; fb_thr_gear = 0;
        expectThrottle('N', 0); assert(g_neutral_brake_satisfied);
        g_mode = next; g_throttle = 0.3f; g_brake = 0;
        expectThrottle(next, 0); assert(!g_dir_guard_active);
        fb_thr_gear = next == 'D' ? 1 : 3;
        expectThrottle(next, 0.3f);
    }
    fb_thr_status_ms = fb_steer_status_ms = fb_brake_status_ms = millis();
    fb_steer_homed = 0; g_estop = false; sendCommands(); assert(g_estop);
    mock_now += 301;
    assert(!statusFresh(fb_steer_status_ms, millis()));
    sendTelemetry();
    assert(Serial.output.find("\"tr\":0,\"sr\":0,\"br\":0") != std::string::npos);
    puts("PASS master: E-stop wire payload, reference gate, node freshness, reference requirement");
}
