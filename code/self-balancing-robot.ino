#include <Wire.h>

// MPU6050
const uint8_t MPU_ADDR = 0x68;

// L298N
const uint8_t en1 = 33;
const uint8_t in1 = 25;
const uint8_t in2 = 26;
const uint8_t in3 = 27;
const uint8_t in4 = 14;
const uint8_t en2 = 12;
bool forward = true;

// Accel data
int16_t acc_x_LSB, acc_y_LSB, acc_z_LSB;
float acc_x, acc_y, acc_z, acc_total, acc_angle_pitch, acc_angle_roll;

// Gyro data
int16_t gyro_x_LSB, gyro_y_LSB, gyro_z_LSB;
float gyro_x, gyro_y, gyro_z, gyro_angle_roll, gyro_angle_pitch;

// General
float angle_pitch, angle_roll;
bool set_starting_gyro_angle = true, set_starting_angles = true;
unsigned long loop_timer;

// PID values and constants
float setpoint = 0.0;
unsigned int kp = 25, ki = 0, kd = 10;
float error, prev_error;
float pid_p, pid_i, pid_d, pid_total;

void setup()
{

    pinMode(en1, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(en2, OUTPUT);

    Serial.begin(115200);
    delay(50);

    // Set the clock speed of I2C
    Wire.setClock(400000);
    Wire.begin();
    delay(250);

    configure_MPU6050_registers();

    // get_gyro_calibration_offset();

    loop_timer = micros();
}

void loop()
{
    process_MPU6050_data();

    error = setpoint - angle_pitch;

    // P value
    pid_p = kp * error;

    // I value
    if (angle_pitch > -5.0 && angle_pitch < 5.0)
        pid_i = pid_i + ki * error;
    else
        pid_i = 0;

    // D value
    pid_d = kd * (prev_error - error);
    prev_error = error;

    // PID total
    pid_total = pid_p + pid_i + pid_d;

    Serial.print(angle_pitch);
    Serial.print(" ");
    Serial.print(angle_roll);
    Serial.print(" ");
    Serial.print(error);
    Serial.print(" ");
    Serial.print(pid_p);
    Serial.print(" ");
    Serial.println(pid_total);

    // Clamp PID value to maximum motor speed
    if (pid_total > 255.0)
        pid_total = 255.0;
    if (pid_total < -255.0)
        pid_total = -255.0;

    // Fall safety
    if (angle_pitch > 30.0 || angle_pitch < -30.0 || angle_roll > 15.0 || angle_roll < -15.0)
        pid_total = 0;

    if (pid_total < 0)
    {
        forward = false;
        move_motors(forward, abs(pid_total));
    }
    else if (pid_total > 0)
    {
        forward = true;
        move_motors(forward, abs(pid_total));
    }
    else
    {
        move_motors(forward, 0);
    }

    // Delay to make 250Hz main loop
    while (micros() - loop_timer < 4000)
        ;
    loop_timer = micros();
}

void read_gyro()
{
    // Read gyroscope data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR, 6);
    gyro_x_LSB = Wire.read() << 8 | Wire.read();
    gyro_y_LSB = Wire.read() << 8 | Wire.read();
    gyro_z_LSB = Wire.read() << 8 | Wire.read();

    // Calibrated offset
    gyro_x_LSB -= -59;
    gyro_y_LSB -= 46;
    gyro_z_LSB -= 89;

    // Angular velocity around respective axes in deg/s
    gyro_x = gyro_x_LSB / 65.5;
    gyro_y = gyro_y_LSB / 65.5;
    gyro_z = gyro_z_LSB / 65.5;
}

void read_acc()
{
    // Read accelerometer data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR, 6);
    acc_x_LSB = Wire.read() << 8 | Wire.read();
    acc_y_LSB = Wire.read() << 8 | Wire.read();
    acc_z_LSB = Wire.read() << 8 | Wire.read();
    // Linear acceleration in respective axes in g
    acc_x = acc_x_LSB / 8192.0;
    acc_y = acc_y_LSB / 8192.0;
    acc_z = acc_z_LSB / 8192.0;
}

void process_MPU6050_data()
{
    read_gyro();
    read_acc();
    // Gyro angle integration for 250 Hz loop
    gyro_angle_pitch += gyro_y * 0.004;
    gyro_angle_roll += gyro_x * 0.004;

    // Yawed transfer the roll angle to the pitch angle
    gyro_angle_pitch += gyro_angle_roll * sin(gyro_z * 0.000000698);
    // Yawed transfer the pitch angle to the roll angle
    gyro_angle_roll -= gyro_angle_pitch * sin(gyro_z * 0.000000698);

    // Acc angle
    acc_total = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
    acc_angle_pitch = asin(-acc_x / acc_total) * (180 / 3.141592);
    acc_angle_roll = asin(acc_y / acc_total) * (180 / 3.141592);

    // Calibrated offset
    acc_angle_pitch -= -2.33;
    acc_angle_roll -= 0.6557;

    if (set_starting_gyro_angle)
    {
        gyro_angle_pitch = acc_angle_pitch;
        gyro_angle_roll = acc_angle_roll;
        set_starting_gyro_angle = false;
    }
    else
    {
        // Gyro and acc angle combined
        gyro_angle_pitch = gyro_angle_pitch * 0.9996 + acc_angle_pitch * 0.0004;
        gyro_angle_roll = gyro_angle_roll * 0.9996 + acc_angle_roll * 0.0004;
    }

    if (set_starting_angles)
    {
        angle_pitch = gyro_angle_pitch;
        angle_roll = gyro_angle_roll;
        set_starting_angles = false;
    }
    else
    {
        // Final output with complementary filter
        angle_pitch = angle_pitch * 0.9 + gyro_angle_pitch * 0.1;
        angle_roll = angle_roll * 0.9 + gyro_angle_roll * 0.1;
    }
}

void move_motors(bool forward, uint8_t speed) // forward, left and right depends on motor driver and sensor placement
{
    // Clip negligible PWM speed to 0
    if (speed < 20)
        speed = 0;

    // Write Speed
    analogWrite(en1, speed);
    analogWrite(en2, speed);
    if (forward)
    {
        // motor right
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        // motor left
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        return;
    }
    // motor right
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // motor left
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}

void configure_MPU6050_registers()
{
    // Start MPU6050 in power mode
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0x8);  // disable temp sensor and wake all other sensors
    Wire.endTransmission(true);

    // Configure gyroscope output range
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B); // GYRO_CONFIG register
    Wire.write(0x8);  // +-500 deg/s, should output 65.5 LSB for 1 deg/s
    Wire.endTransmission();

    // Configure accelerometer output range
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C); // ACCEL_CONFIG register
    Wire.write(0x8);  // +-4g, should output 8192 LSB for 1g according to dataset
    Wire.endTransmission();
}

void get_gyro_calibration_offset()
{
    long gyro_cal_x_LSB = 0, gyro_cal_y_LSB = 0, gyro_cal_z_LSB = 0;
    Serial.print("\nCalibrating gyro.");
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