#pragma once
// Only hardware step generation is mocked. Firmware CAN parsing, interlocks,
// state transitions and stepper calls execute from the actual .cpp source.
struct AccelStepper {
    enum { DRIVER };
    long current = 0, target = 0;
    unsigned pulses = 0;
    unsigned minimumPulseUs = 1;
    AccelStepper(int, int, int) {}
    void setPinsInverted(bool, bool, bool) {}
    void setMaxSpeed(float) {}
    void setAcceleration(float) {}
    void setMinPulseWidth(unsigned value) { minimumPulseUs = value; }
    void setSpeed(float) {}
    void setCurrentPosition(long value) { current = target = value; }
    long currentPosition() { return current; }
    void moveTo(long value) { target = value; }
    long distanceToGo() { return target - current; }
    void run() {
        if (current != target) {
            _direction = target > current;
            current += _direction ? 1 : -1; ++pulses;
            step1(current);
        }
    }
    void runSpeed() { ++pulses; }
protected:
    bool _direction = false;
    virtual void step1(long) {}
};
