#pragma once

#include <cstdint>

class L298N
{
private:
    const uint8_t en1, en2, in1, in2, in3, in4;

public:
    L298N();

    bool forward;
    void pin_mode();
    void move_motors(uint8_t speed);
};
