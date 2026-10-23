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
    fb_thr_status_ms = fb_steer_status_ms = fb_brake_status_ms = millis();
    fb_steer_homed = 0; g_estop = false; sendCommands(); assert(g_estop);
    mock_now += 301;
    assert(!statusFresh(fb_steer_status_ms, millis()));
    sendTelemetry();
    assert(Serial.output.find("\"tr\":0,\"sr\":0,\"br\":0") != std::string::npos);
    puts("PASS master: E-stop wire payload, reference gate, node freshness, reference requirement");
}
