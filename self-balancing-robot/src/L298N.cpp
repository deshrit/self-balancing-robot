#include <Arduino.h>

#include "L298N.hpp"

L298N::L298N():en1(33), in1(25), in2(26), en2(12), in3(27), in4(14), forward(true) {}

void L298N::pin_mode() {
    pinMode(en1, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(en2, OUTPUT);
}

void L298N::move_motors(uint8_t speed) {
    /* Clip negligible PWM speed to 0 */
    if (speed < 20)
        speed = 0;

    /* Write Speed */
    analogWrite(en1, speed);
    analogWrite(en2, speed);
    if (forward)
    {
        /* Motor right */
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        /* Motor left */
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        return;
    }
    /* Motor right */
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    /* Motor left */
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}