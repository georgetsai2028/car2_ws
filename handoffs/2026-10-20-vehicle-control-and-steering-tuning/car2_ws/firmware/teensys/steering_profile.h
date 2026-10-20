#pragma once
// Measured, single-turn, monotonic analog steering calibration. No old-car values.
// ADC codes describe the safely scaled A0 signal, not physical wheel degrees.
#include <stdint.h>
#include <stddef.h>

namespace steering_profile {
constexpr uint32_t MAGIC = 0x53434131u;
constexpr uint16_t VERSION = 1;
constexpr int EEPROM_ADDRESS = 128;
constexpr int ADC_MAX = 4095;  // This profile requires 12-bit ADC readers.
struct Profile {
    uint32_t magic;
    uint16_t version, bytes;
    int32_t left_steps, right_steps;
    uint16_t left_adc, center_adc, right_adc, margin_permille;
    uint32_t crc;
};
static_assert(sizeof(Profile) == 28, "Unexpected persistent calibration layout");

inline uint32_t checksum(const Profile& p) {
    const uint8_t* data = reinterpret_cast<const uint8_t*>(&p);
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < offsetof(Profile, crc); ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320u : 0u);
    }
    return ~crc;
}
inline void seal(Profile& p) {
    p.magic = MAGIC; p.version = VERSION; p.bytes = sizeof(Profile); p.crc = checksum(p);
}
inline int magnitude(int x) { return x < 0 ? -x : x; }
inline bool valid(const Profile& p) {
    if (p.magic != MAGIC || p.version != VERSION || p.bytes != sizeof(Profile) || p.crc != checksum(p)) return false;
    if (p.left_steps > -50 || p.right_steps < 50 || p.left_steps < -200000 || p.right_steps > 200000) return false;
    if (p.left_adc < 8 || p.left_adc > ADC_MAX - 8 || p.center_adc < 8 || p.center_adc > ADC_MAX - 8 ||
        p.right_adc < 8 || p.right_adc > ADC_MAX - 8) return false;
    const int a = (int)p.center_adc - p.left_adc, b = (int)p.right_adc - p.center_adc;
    // Reject a wrap/discontinuity across the calibrated range, weak sensor span,
    // wrong center ordering, or a center captured outside measured endpoints.
    if (!((a >= 8 && b >= 8) || (a <= -8 && b <= -8))) return false;
    if (magnitude((int)p.right_adc - p.left_adc) < 32) return false;
    return p.margin_permille >= 20 && p.margin_permille <= 200;
}
inline int32_t softLeft(const Profile& p) {
    return p.left_steps + ((-p.left_steps) * p.margin_permille) / 1000;
}
inline int32_t softRight(const Profile& p) {
    return p.right_steps - (p.right_steps * p.margin_permille) / 1000;
}
inline int32_t rawToSteps(const Profile& p, int raw) {
    const int direction = p.right_adc > p.left_adc ? 1 : -1;
    if ((raw - (int)p.center_adc) * direction < 0) {
        return (int32_t)(((int64_t)(raw - (int)p.center_adc) * p.left_steps) /
                         ((int)p.left_adc - p.center_adc));
    }
    return (int32_t)(((int64_t)(raw - (int)p.center_adc) * p.right_steps) /
                     ((int)p.right_adc - p.center_adc));
}
inline int stepsToRaw(const Profile& p, int32_t steps) {
    return p.center_adc + (int)(((int64_t)steps *
        ((steps < 0 ? (int)p.left_adc : (int)p.right_adc) - p.center_adc)) /
        (steps < 0 ? p.left_steps : p.right_steps));
}
inline bool rawPlausible(const Profile& p, int raw) {
    const int lo = p.left_adc < p.right_adc ? p.left_adc : p.right_adc;
    const int hi = p.left_adc > p.right_adc ? p.left_adc : p.right_adc;
    return raw >= 4 && raw <= ADC_MAX - 4 && raw >= lo - 12 && raw <= hi + 12;
}
}
