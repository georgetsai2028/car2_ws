#include <cassert>
#include "../firmware/teensys/brake_bench_test.cpp"

int main() {
    setup();
    startPulse(Motion::Apply, millis(), cfg::SHORT_PULSE_MS);
    assert(mock_pwm[19] == cfg::PULSE_PWM && mock_pwm[18] == 0);
    mock_now += cfg::SHORT_PULSE_MS; loop();
    assert(mock_pwm[18] == 0 && mock_pwm[19] == 0);
    mock_now += cfg::COOLDOWN_MS;
    startPulse(Motion::Release, millis(), cfg::SHORT_PULSE_MS);
    assert(mock_pwm[18] == cfg::PULSE_PWM && mock_pwm[19] == 0);
    puts("PASS brake bench: matching apply/release polarity and automatic pulse stop");
}
