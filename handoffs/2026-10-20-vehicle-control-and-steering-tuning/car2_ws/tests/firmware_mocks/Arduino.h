#pragma once
#include <cstdint>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <deque>
#include <string>
using std::uint8_t;
using std::uint32_t;
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define A0 14
#define F(x) x
static uint32_t mock_now = 1000;
static int mock_pins[64] = {};
static int mock_pwm[64] = {};
inline uint32_t millis() { return mock_now; }
inline void delay(uint32_t ms) { mock_now += ms; }
static unsigned long mock_microsecond_delays = 0;
inline void delayMicroseconds(unsigned int value) { mock_microsecond_delays += value; }
inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value) { mock_pins[pin] = value; }
inline int digitalRead(int pin) { return mock_pins[pin]; }
static int mock_analog_read = 750;
inline int analogRead(int) { return mock_analog_read; }
inline void analogReadResolution(int) {}
inline void analogWriteResolution(int) {}
inline void analogWriteFrequency(int, float) {}
inline void analogWrite(int pin, int value) { mock_pwm[pin] = value; }
struct MockSerial {
    bool connected = true;
    operator bool() const { return connected; }
    std::string output;
    std::deque<char> input;
    void begin(int) {}
    int available() { return (int)input.size(); }
    char read() { char c = input.front(); input.pop_front(); return c; }
    template<class T> void print(T) {}
    template<class T> void print(T, int) {}
    void println() { output += '\n'; }
    void println(const char* s) { output += s; output += '\n'; }
    void println(char* s) { println((const char*)s); }
    template<class T> void println(T) {}
    void println(float, int) {}
    void printf(const char* format, ...) {
        char buf[512];
        va_list args; va_start(args, format);
        vsnprintf(buf, sizeof(buf), format, args); va_end(args);
        output += buf;
    }
};
static MockSerial Serial;
