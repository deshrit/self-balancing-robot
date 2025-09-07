#include <Arduino.h>
#include <Wire.h>

#include "MPU6050.hpp"

MPU6050::MPU6050():MPU_ADDR(0x68) {}

void MPU6050::configure()
{
    /* Set the clock speed of I2C */
    Wire.setClock(400000);
    Wire.begin();
    delay(250);

    /* Start MPU6050 in power mode */
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); /* PWR_MGMT_1 register */
    Wire.write(0x8);  /* Disable temp sensor and wake all other sensors */
    Wire.endTransmission(true);

    /* Configure gyroscope output range */
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); /* GYRO_CONFIG register */
    Wire.write(0x8);  /* +-500 deg/s, Should output 65.5 LSB for 1 deg/s */
    Wire.endTransmission();

    /* Configure accelerometer output range */
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); /* ACCEL_CONFIG register */
    Wire.write(0x8);  /* +-4g, Should output 8192 LSB for 1g according to dataset */
    Wire.endTransmission();
}


void MPU6050::read_gyro()
{
    /* Read gyroscope data */ 
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR, 6);
    gyro_x_LSB = Wire.read() << 8 | Wire.read();
    gyro_y_LSB = Wire.read() << 8 | Wire.read();
    gyro_z_LSB = Wire.read() << 8 | Wire.read();

    /* 
    Calibrated offsets.

    These Values are obtained after running `get_gyro_calibration_offset` 
    in the `setup` function keeping the bot flat.
    */
    gyro_x_LSB += 85;
    gyro_y_LSB -= 35;
    gyro_z_LSB += 80;

    /* Angular velocity around respective axes in deg/s */
    gyro_x = gyro_x_LSB / 65.5;
    gyro_y = gyro_y_LSB / 65.5;
    gyro_z = gyro_z_LSB / 65.5;
}

void MPU6050::get_gyro_calibration_offset()
{
    long gyro_cal_x_LSB = 0, gyro_cal_y_LSB = 0, gyro_cal_z_LSB = 0;
    Serial.print("\nTo calibrate gyro");
    for (int i = 0; i < 2000; i++)
    {
        read_gyro();
        if (i % 100 == 0)
        {
            Serial.print(".");
        }
        gyro_cal_x_LSB += gyro_x_LSB;
        gyro_cal_y_LSB += gyro_y_LSB;
        gyro_cal_z_LSB += gyro_z_LSB;
    }
    gyro_cal_x_LSB /= 2000;
    gyro_cal_y_LSB /= 2000;
    gyro_cal_z_LSB /= 2000;
    Serial.print("\ngryo_cal_x_LSB: ");
    Serial.print(gyro_cal_x_LSB);
    Serial.print("\tgryo_cal_y_LSB: ");
    Serial.print(gyro_cal_y_LSB);
    Serial.print("\tgryo_cal_z_LSB: ");
    Serial.print(gyro_cal_z_LSB);
}

void MPU6050::read_acc()
{
    /* Read accelerometer data */
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR, 6);
    acc_x_LSB = Wire.read() << 8 | Wire.read();
    acc_y_LSB = Wire.read() << 8 | Wire.read();
    acc_z_LSB = Wire.read() << 8 | Wire.read();

    /* 
    Calibrated offsets.

    These Values are obtained after running `get_gyro_calibration_offset` 
    in the `setup` function keeping the bot flat.
    */
    acc_x_LSB -= 300;
    acc_y_LSB -= 50;
    acc_z_LSB += 600;

    /* Linear acceleration in respective axes in g */
    acc_x = acc_x_LSB / 8192.0;
    acc_y = acc_y_LSB / 8192.0;
    acc_z = acc_z_LSB / 8192.0;
}

void MPU6050::get_acc_calibration_offset()
{
    long acc_cal_x_LSB = 0, acc_cal_y_LSB = 0, acc_cal_z_LSB = 0;
    Serial.print("\nTo calibrate acc");
    for (int i = 0; i < 2000; i++)
    {
        read_acc();
        if (i % 100 == 0)
        {
            Serial.print(".");
        }
        acc_cal_x_LSB += acc_x_LSB;
        acc_cal_y_LSB += acc_y_LSB;
        acc_cal_z_LSB += acc_z_LSB;
    }
    acc_cal_x_LSB /= 2000;
    acc_cal_y_LSB /= 2000;
    acc_cal_z_LSB /= 2000;
    Serial.print("\nacc_cal_x_LSB: ");
    Serial.print(acc_cal_x_LSB);
    Serial.print("\tacc_cal_y_LSB: ");
    Serial.print(acc_cal_y_LSB);
    Serial.print("\tacc_cal_z_LSB: ");
    Serial.print(acc_cal_z_LSB);
}

void MPU6050::process_MPU6050_data()
{
    read_gyro();
    read_acc();

    /* Gyro angle integration for 100 Hz loop */
    gyro_angle_pitch += gyro_y * 0.004;
    gyro_angle_roll += gyro_x * 0.004;

    /* Yawed transfer the roll angle to the pitch angle */
    gyro_angle_pitch += gyro_angle_roll * sin(gyro_z * 0.000000698);
    /* Yawed transfer the pitch angle to the roll angle */
    gyro_angle_roll -= gyro_angle_pitch * sin(gyro_z * 0.000000698);

    /* Acc angle */
    acc_total = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
    acc_angle_pitch = asin(-acc_x / acc_total) * (180 / 3.141592);
    acc_angle_roll = asin(acc_y / acc_total) * (180 / 3.141592);


    angle_pitch = 0.95 * gyro_angle_pitch + 0.05 * acc_angle_pitch;
    angle_roll = 0.95 * gyro_angle_roll + 0.05 * acc_angle_roll;

    gyro_angle_pitch = angle_pitch;
    gyro_angle_roll = angle_roll;

    // Serial.print("angle_pitch:");
    // Serial.print(angle_pitch);
    // Serial.print(",");
    // Serial.print("angle_roll:");
    // Serial.println(angle_roll);
}