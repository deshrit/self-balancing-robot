// ############################### IN PROGRESS #########################################
// #####################################################################################

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
uint8_t speed = 200; // upto 255 pwm max

// acc
float acc_X, acc_Y, acc_Z;

void setup()
{
    Serial.begin(9600);

    pinMode(en1, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(en2, OUTPUT);

    // Initiate wire library
    Wire.setClock(400000);
    Wire.begin();
    delay(250);
    // Power up sensor register
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
}

void loop()
{
    read_imu_data();

    Serial.print("X:");
    Serial.print(acc_X);
    Serial.print(",");
    Serial.print("Y:");
    Serial.print(acc_Y);
    Serial.print(",");
    Serial.print("Z:");
    Serial.println(acc_Z);

    if (acc_X >= 0.18 && acc_X < 0.7)
    {
        move_motors(true, speed);
    }
    else if (acc_X <= -0.18 && acc_X > -0.7)
    {
        move_motors(false, speed);
    }
    else
    {
        move_motors(true, 0);
    }

    delay(50);
}

void read_acc()
{
    // low pass filter
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission();

    // configure accelerometer output => +-8g
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    // Read accelerometer data
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x3B);
    Wire.endTransmission();

    Wire.requestFrom(mpu_addr, 6);
    int16_t acc_X_LSB = Wire.read() << 8 | Wire.read();
    int16_t acc_Y_LSB = Wire.read() << 8 | Wire.read();
    int16_t acc_Z_LSB = Wire.read() << 8 | Wire.read();

    // LSB to g
    acc_X = (float)acc_X_LSB / 4096;
    acc_Y = (float)acc_Y_LSB / 4096;
    acc_Z = (float)acc_Z_LSB / 4096;
}

void read_imu_data()
{

    read_acc();
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
