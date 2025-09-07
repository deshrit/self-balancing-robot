#include <Arduino.h>

#include "L298N.hpp"

L298N::L298N():EN1(33), IN1(25), IN2(26), EN2(12), IN3(27), IN4(14), forward(true), MAX_SPEED(200), RIGHT_MOTOR_CAL_OFFSET(18) {}

void L298N::pin_mode() {
    pinMode(EN1, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(EN2, OUTPUT);
}

void L298N::move_motors(uint16_t speed = 0) {

    if(speed > MAX_SPEED) speed = MAX_SPEED;

    Serial.print("\tspd:");
    Serial.print(speed);
    /* Motor right speed (assumed taking one side as forward reference of bot)*/
    analogWrite(EN1, (speed > RIGHT_MOTOR_CAL_OFFSET && speed < MAX_SPEED - RIGHT_MOTOR_CAL_OFFSET) ? speed - RIGHT_MOTOR_CAL_OFFSET : speed); // Cheap motor offset
    /* Motor left speed */
    analogWrite(EN2, speed);
    if (forward)
    {
        /* Motor right */
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        /* Motor left */
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        return;
    }
    /* Motor right */
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    /* Motor left */
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}