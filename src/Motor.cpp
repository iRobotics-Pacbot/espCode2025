#include <Arduino.h>
#include <Encoder.h>

#include <Motor.h>
#include <algorithm>
// some rough testing suggests min 5k based on electrical time constant
#define FREQ 10000
#define RES 15

Motor::Motor(uint8_t ph, uint8_t en): ph(ph), en(en) {
    pinMode(en, OUTPUT);
    pinMode(ph, OUTPUT);
    
    analogWriteFrequency(FREQ);
    analogWriteResolution(RES);
}

void Motor::setThrottle(double throttle) {
    throttle = std::min(std::max(throttle,(double) -1), (double)1);
    int16_t pwm_duty = abs(throttle) * 0x7fffp0;
    if (throttle > 0) {
        digitalWrite(ph, HIGH);
    } else {
        digitalWrite(ph, LOW);
    }
    analogWrite(en, pwm_duty);
}