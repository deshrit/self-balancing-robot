#pragma once

#include <cstdint>

class MPU6050
{
private:
    /* Accel data */
    int16_t acc_x_LSB, acc_y_LSB, acc_z_LSB;
    float acc_x, acc_y, acc_z, acc_total, acc_angle_pitch, acc_angle_roll;

    /* Gyro data */
    int16_t gyro_x_LSB, gyro_y_LSB, gyro_z_LSB;
    float gyro_x, gyro_y, gyro_z, gyro_angle_roll, gyro_angle_pitch;

    const uint8_t MPU_ADDR;

    void read_gyro();
    void read_acc();

public:
    MPU6050();

    float angle_pitch, angle_roll;

    void configure();
    void get_gyro_calibration_offset();
    void get_acc_calibration_offset();
    void process_MPU6050_data();
};
