#include <Wire.h>

// MPU6050
const uint8_t mpu_addr = 0x68;

// L298N
const uint8_t en1 = 33;
const uint8_t in1 = 25;
const uint8_t in2 = 26;
const uint8_t in3 = 27;
const uint8_t in4 = 14;
const uint8_t en2 = 12;
bool forward = true;

// Data
float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, acc_total, angle_roll, angle_pitch;

unsigned long loop_timer;

// PID values and constants
float setpoint = 0.0;
unsigned int kp = 40, ki = 0, kd = 0;
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

    // Start gyro in power mode
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Set low pass filter for both gyro and acc
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    loop_timer = micros();
}

void loop()
{
    get_current_inclination();

    error = setpoint - angle_pitch;

    Serial.print("Angle roll: ");
    Serial.print(angle_roll);
    Serial.print("\tAngle pitch: ");
    Serial.print(angle_pitch);

    pid_p = kp * error;

    if (angle_pitch > -5 && angle_pitch < 5)
        pid_i = pid_i + ki * error;
    else
        pid_i = 0;

    pid_d = kd * (prev_error - error);
    prev_error = error;

    pid_total = pid_p + pid_i + pid_d;

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

    Serial.print("\terror: ");
    Serial.print(error);
    Serial.print("\tpid_p: ");
    Serial.print(pid_p);
    Serial.print("\tpid_i: ");
    Serial.print(pid_i);
    Serial.print("\tpid_d: ");
    Serial.print(pid_d);
    Serial.print("\tpid_total: ");
    Serial.println(pid_total);

    // wait before continuing the loop to make 250Hz
    while (micros() - loop_timer < 4000)
        ;
    loop_timer = micros();
}

void read_acc()
{
    // Configure accelerometer output => +-8g
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    // Read accelerometer data
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(mpu_addr, 6);
    int16_t acc_x_LSB = Wire.read() << 8 | Wire.read();
    int16_t acc_y_LSB = Wire.read() << 8 | Wire.read();
    int16_t acc_z_LSB = Wire.read() << 8 | Wire.read();
    // LSB to g
    acc_x = (float)acc_x_LSB / 4096;
    acc_y = (float)acc_y_LSB / 4096;
    acc_z = (float)acc_z_LSB / 4096;
}

void read_gyro()
{
    // Sensitivity scale factor => 65.5 LSB per deg
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x1B);
    Wire.write(0x8);
    Wire.endTransmission();
    // Read gyroscope data
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(mpu_addr, 6);
    int16_t gyro_x_LSB = Wire.read() << 8 | Wire.read();
    int16_t gyro_y_LSB = Wire.read() << 8 | Wire.read();
    int16_t gyro_z_LSB = Wire.read() << 8 | Wire.read();
}

void read_MPU6050_data()
{
    read_acc();
    read_gyro();
}

void get_current_inclination()
{
    read_MPU6050_data();
    angle_roll = atan(acc_y / sqrt(acc_x * acc_x + acc_z * acc_z)) * 57.323;
    angle_pitch = -atan(acc_x / sqrt(acc_y * acc_y + acc_z * acc_z)) * 57.323;
}

void move_motors(bool forward, uint8_t speed)
{
    // speed
    analogWrite(en1, speed);
    analogWrite(en2, speed);
    if (forward)
    {
        // motor1 dir
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        // motor2 dir
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        return;
    }
    // motor1 dir
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // motor2 dir
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
}
