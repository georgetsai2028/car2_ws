#include <cassert>
#include <cstdio>
#include "../firmware/teensys/steering_profile.h"
using namespace steering_profile;
int main() {
    Profile p = {};
    assert(!valid(p));
    p.left_steps = -1000; p.right_steps = 1400;
    p.left_adc = 1000; p.center_adc = 1800; p.right_adc = 3000;
    p.margin_permille = 80; seal(p);
    assert(valid(p));
    assert(softLeft(p) == -920 && softRight(p) == 1288);
    assert(rawToSteps(p, 1000) == -1000 && rawToSteps(p, 1800) == 0 && rawToSteps(p, 3000) == 1400);
    assert(rawToSteps(p, 1400) == -500 && rawToSteps(p, 2400) == 700);
    assert(stepsToRaw(p, -500) == 1400 && stepsToRaw(p, 700) == 2400);
    assert(rawPlausible(p, 1000) && rawPlausible(p, 3000));
    assert(!rawPlausible(p, 0) && !rawPlausible(p, 4095) && !rawPlausible(p, 950));
    Profile corrupt = p; corrupt.center_adc++; assert(!valid(corrupt));
    p.left_adc = 3000; p.right_adc = 1000; p.center_adc = 1800; seal(p);
    assert(valid(p) && rawToSteps(p, 3000) == -1000 && rawToSteps(p, 1000) == 1400);
    p.center_adc = 3100; seal(p); assert(!valid(p)); // Wrap/center outside both endpoints.
    p.center_adc = 1800; p.margin_permille = 0; seal(p); assert(!valid(p));
    p.margin_permille = 80; p.right_steps = 0; seal(p); assert(!valid(p));
    puts("PASS measured steering profile: CRC, asymmetric endpoints, reversed sensor direction, soft margins, rail/wrap/invalid-center rejection");
}
