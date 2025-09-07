#pragma once

#include <cstdint>

class L298N
{
private:
    const uint8_t EN1, EN2, IN1, IN2, IN3, IN4, MAX_SPEED, RIGHT_MOTOR_CAL_OFFSET;

public:
    L298N();

    bool forward;
    void pin_mode();
    void move_motors(uint16_t speed);
};
