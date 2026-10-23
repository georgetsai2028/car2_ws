#include <cassert>
#include "../firmware/teensys/brake.cpp"

static void feedback(int value) { mock_analog_read = value; fbk_filt = value; }

int main() {
    setup();
    stale = false; cmd_estop = false;
    feedback(cfg::ADC_RELEASED); cmd_brake = 1.0f; controlStep();
    assert(mock_pwm[19] > 0 && mock_pwm[18] == 0 && motion == MOTION_EXTEND);
    feedback(cfg::ADC_APPLIED); cmd_brake = 0.0f; controlStep();
    assert(mock_pwm[18] > 0 && mock_pwm[19] == 0 && motion == MOTION_RETRACT);
    feedback(cfg::ADC_APPLIED); cmd_brake = 1.0f; controlStep();
    assert(mock_pwm[18] == 0 && mock_pwm[19] == 0 && motion == MOTION_STOP);
    feedback(cfg::ADC_RELEASED); cmd_brake = 0.0f; cmd_estop = true; controlStep();
    assert(mock_pwm[19] > 0 && mock_pwm[18] == 0);
    cmd_estop = false; stale = true; controlStep();
    assert(mock_pwm[19] > 0 && mock_pwm[18] == 0);
    puts("PASS brake: apply pin19, release pin18, target stop, E-stop and stale apply");
}
