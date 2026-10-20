// Temporary acceleration-only diagnostic; restore throttle.hex for vehicle use.
// No CAN control, no EEPROM writes, no automatic motion on boot or USB connect.
// MCP4725 read format: Microchip DS20002039E fig. 6-3 (five-byte repeat).
#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <cstring>

namespace bench {
constexpr uint8_t B = 33, C = 34, D = 35;
constexpr uint16_t IDLE = 993;
constexpr uint32_t LINK_TIMEOUT = 250;
constexpr uint32_t DURATIONS[] = {500, 500, 1000, 1000, 1000, 1000, 1000, 500};
// Same counts mapping as vehicle firmware: idle + 0/5/10/15/20/25% of span.
constexpr uint16_t COUNTS[] = {993, 993, 1148, 1303, 1458, 1613, 1768, 993};
}
Encoder motorEncoder(2, 3);
static uint8_t address = 0;
static bool ready = false, running = false, fault = false, drive = false;
static int requested = -1, readCount = -1, pd = -1, wireError = 0;
static uint32_t writesOk = 0, writesFailed = 0, mismatches = 0;
static uint32_t phaseStarted = 0, heartbeat = 0, lastCheck = 0;
static uint8_t phase = 0;
static const char* reason = "BOOT_NO_OUTPUT_WRITES";

static void neutral() {
    digitalWrite(bench::B, HIGH); digitalWrite(bench::C, HIGH);
    digitalWrite(bench::D, HIGH); drive = false;
}
static bool readDac() {
    uint8_t data[5];
    if (Wire.requestFrom(address, (uint8_t)5) != 5) return false;
    for (uint8_t i = 0; i < 5; ++i) data[i] = Wire.read();
    readCount = ((uint16_t)data[1] << 4) | (data[2] >> 4);
    pd = (data[0] >> 1) & 3;
    return (data[0] & 0x80) != 0;
}
static bool writeDac(uint16_t counts) {
    requested = counts;
    Wire.beginTransmission(address);
    Wire.write((uint8_t)0x40); // volatile register only, never EEPROM
    Wire.write((uint8_t)(counts >> 4));
    Wire.write((uint8_t)(counts << 4));
    wireError = Wire.endTransmission();
    if (wireError) { ++writesFailed; return false; }
    ++writesOk;
    if (!readDac() || readCount != counts || pd != 0) {
        ++mismatches; return false;
    }
    return true;
}
static void stop(const char* why, bool failed = false) {
    // Select Neutral first even if I2C is stuck or the DAC cannot be reached.
    neutral(); running = false; reason = why;
    if (failed) fault = true;
    if (ready && !writeDac(bench::IDLE)) {
        fault = true; reason = "STOP_DAC_UNVERIFIED";
    }
}
static void probe() {
    if (running) { stop("PROBE_DURING_RUN", true); return; }
    neutral(); ready = false; address = 0; requested = readCount = pd = -1;
    fault = false; reason = "NO_DAC_ACK";
    // Pin levels are observations of the digital bus, not analog voltages.
    Serial.printf("{\"event\":\"bus\",\"sda18\":%d,\"scl19\":%d}\n",
                  digitalRead(18), digitalRead(19));
    uint8_t found = 0;
    for (uint8_t a = 0x60; a <= 0x67; ++a) {
        Wire.beginTransmission(a);
        const int e = Wire.endTransmission();
        Serial.printf("{\"event\":\"i2c_probe\",\"addr\":%u,\"error\":%d}\n", a, e);
        if (e == 0) { ++found; address = a; }
    }
    if (found != 1) {
        reason = found ? "MULTIPLE_DAC_ADDRESSES" : "NO_DAC_ACK";
        address = 0; return;
    }
    // Do not guess the DAC family from its address. MCP4728 also uses 0x60.
    uint8_t raw[30];
    const uint8_t n = Wire.requestFrom(address, (uint8_t)sizeof(raw));
    Serial.printf("{\"event\":\"dac_read\",\"addr\":%u,\"n\":%u,\"hex\":\"", address, n);
    for (uint8_t i = 0; i < n; ++i) { raw[i] = Wire.read(); Serial.printf("%02X", raw[i]); }
    Serial.println("\"}");
    if (n != sizeof(raw)) { reason = "SHORT_DAC_READ"; return; }
    bool repeated = (raw[0] & 0x80) != 0;
    for (uint8_t i = 5; i < sizeof(raw); ++i) repeated &= raw[i] == raw[i % 5];
    if (!repeated) { reason = "NOT_MCP4725_READ_FORMAT"; return; }
    // Restrict writes to the two addresses supported by the installed firmware.
    if (address != 0x60 && address != 0x62) { reason = "UNCONFIGURED_DAC_ADDRESS"; return; }
    ready = readDac();
    reason = ready ? "MCP4725_READ_COMPATIBLE" : "DAC_NOT_READY";
}
static void start() {
    if (running) return; // A repeated RUN cannot extend the finite sequence.
    if (!ready || fault) { reason = "RUN_REFUSED_PREFLIGHT"; return; }
    neutral();
    if (!writeDac(bench::IDLE)) { stop("INITIAL_IDLE_UNVERIFIED", true); return; }
    phase = 0; phaseStarted = heartbeat = lastCheck = millis();
    running = true; reason = "FINITE_FORWARD_TEST";
}
static void tick() {
    if (!running) return;
    const uint32_t now = millis();
    if (!Serial || now - heartbeat > bench::LINK_TIMEOUT) {
        stop("USB_KEEPALIVE_TIMEOUT", true); return;
    }
    if (now - phaseStarted >= bench::DURATIONS[phase]) {
        ++phase;
        if (phase == sizeof(bench::DURATIONS) / sizeof(bench::DURATIONS[0])) {
            stop("COMPLETE"); return;
        }
        phaseStarted = now;
        if (phase == 1) {
            digitalWrite(bench::B, LOW); drive = true; // D=LHH; no reverse command.
        }
        if (!writeDac(bench::COUNTS[phase])) { stop("DAC_WRITE_OR_READBACK_FAILED", true); return; }
    }
    if (now - lastCheck >= 50) {
        lastCheck = now;
        if (!readDac() || readCount != requested || pd != 0) {
            ++mismatches; stop("DAC_READBACK_LOST", true);
        }
    }
}
static void status() {
    static long previousCount = 0;
    static uint32_t previousTime = 0;
    const long encoder = motorEncoder.read();
    const uint32_t now = millis(), dt = now - previousTime;
    const float rpm = previousTime && dt ? (encoder - previousCount) * 25.0f / dt : 0;
    previousCount = encoder; previousTime = now;
    Serial.printf("{\"fw\":\"THROTTLE_USB_TEST_V1\",\"ready\":%d,\"running\":%d,\"fault\":%d,"
                  "\"addr\":%u,\"phase\":%u,\"gear\":\"%s\",\"bcd\":\"%d%d%d\","
                  "\"requested\":%d,\"readCount\":%d,\"pd\":%d,\"wireError\":%d,"
                  "\"writesOk\":%lu,\"writesFailed\":%lu,\"mismatches\":%lu,"
                  "\"encoder\":%ld,\"rpm\":%.2f,\"reason\":\"%s\"}\n",
                  ready, running, fault, address, phase, drive ? "D" : "N",
                  digitalRead(bench::B), digitalRead(bench::C), digitalRead(bench::D),
                  requested, readCount, pd, wireError,
                  (unsigned long)writesOk, (unsigned long)writesFailed, (unsigned long)mismatches,
                  encoder, rpm, reason);
}
void setup() {
    // Preload HIGH before enabling each output; avoid a low pulse on gear B.
    digitalWrite(bench::B, HIGH); digitalWrite(bench::C, HIGH); digitalWrite(bench::D, HIGH);
    pinMode(bench::B, OUTPUT); pinMode(bench::C, OUTPUT); pinMode(bench::D, OUTPUT);
    neutral(); Serial.begin(115200); Wire.begin(); Wire.setClock(100000);
    // Host must request P (read-only probe) before R (one finite sequence).
}
void loop() {
    tick(); // A command arriving after link timeout cannot revive an expired run.
    while (Serial.available()) {
        const char c = Serial.read();
        if (c == 'x' || c == 'X') stop("USB_STOP");
        else if (c == 'K' && running) heartbeat = millis();
        else if (c == 'P') probe();
        else if (c == 'R') start();
        else if (c == '?') status();
    }
    tick();
    static uint32_t lastStatus = 0;
    if (millis() - lastStatus >= 100) { lastStatus = millis(); status(); }
}
