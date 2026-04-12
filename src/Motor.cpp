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

    Serial.println("Called");
}

void Motor::setThrottle(double throttle) {
    throttle = min(max(throttle,(double) -1), (double)1);
    int16_t pwm_duty = abs(throttle) * 255;
    // Serial.println(pwm_duty);
    // Serial.println(en);
    if (throttle > 0) {
        digitalWrite(ph, HIGH);
    } else {
        digitalWrite(ph, LOW);
    }
    analogWrite(en, pwm_duty);
}