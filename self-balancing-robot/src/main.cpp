#include <Arduino.h>
#include <Wire.h>

#include "L298N.hpp"
#include "MPU6050.hpp"
#include "RobotServer.hpp"
#include "ArduinoJson.h"

unsigned long loop_timer;

/* PID values and constants */
float setpoint = 0.0;
float kp = 60.0, ki = 0.0, kd = 0.0;
float error = 0.0, prev_error = 0.0;
float pid_p = 0.0, pid_i = 0.0, pid_d = 0.0, pid_total = 0.0;

L298N driver;
MPU6050 sensor;
// RobotServer robot_server

/* Web socket data send rate */
// unsigned long last_sent = 0;
// const unsigned long BROADCAST_INTERVAL_MS = 100;
// JsonDocument doc;
// String json;


void setup()
{
    Serial.begin(115200);
    delay(50);

    driver.pin_mode();
    sensor.configure();

    // pinMode(LED_BUILTIN, OUTPUT);
    // digitalWrite(LED_BUILTIN, HIGH);

    // robot_server.configure();
    
    // delay(3000);
    // digitalWrite(LED_BUILTIN, LOW);

    /* Get calibration offsets */
//   sensor.get_gyro_calibration_offset();
//   sensor.get_acc_calibration_offset();
//   for(;;)

  loop_timer = micros();
}

void loop()
{
    sensor.process_MPU6050_data();

    /* Send data to web socket */
    // doc["angle_pitch"] = sensor.angle_pitch;
    // doc["angle_roll"] = sensor.angle_roll;
    // if(millis() - last_sent >= BROADCAST_INTERVAL_MS) {
    //     if(robot_server.ws.count() > 0) {
    //         robot_server.send_data(doc, json);
    //     }
    //     last_sent = millis();
    // }
    error = setpoint - sensor.angle_pitch;

    /* Dead zone */
    if(fabs(error) < 1.0) {
        error = 0.0;
    }

    /* Fall safety */
    if (sensor.angle_pitch > 35.0 || sensor.angle_pitch < -35.0 || sensor.angle_roll > 15.0 || sensor.angle_roll < -15.0) {
        error = 0.0;
        pid_i = 0.0;
    }

    /* P value */
    pid_p = kp * error;

    // /* I value */
    // if (sensor.angle_pitch > -15.0 && sensor.angle_pitch < 15.0)
    pid_i = pid_i + ki * error;
    // else {
    //     pid_i = 0;
    // }

    /* D value */
    pid_d = kd * (prev_error - error);
    prev_error = error;

    /* PID total */
    pid_total = pid_p + pid_i - pid_d;

    Serial.print("\nang:");
    Serial.print(sensor.angle_pitch);
    Serial.print("\tkp:");
    Serial.print(kp);
    Serial.print("\tki:");
    Serial.print(ki);
    Serial.print("\tkd:");
    Serial.print(kd);
    Serial.print("\terr:");
    Serial.print(error);
    Serial.print("\tp: ");
    Serial.print(pid_p);
    Serial.print("\ti: ");
    Serial.print(pid_d);
    Serial.print("\td: ");
    Serial.print(pid_d);
    Serial.print("\tpid:");
    Serial.print(pid_total);


    /* Clamp PID value to maximum motor speed */
    pid_total = constrain(pid_total, -255.0, 255.0);

    if(pid_total > 0.0) driver.forward = true;
    else if(pid_total < 0.0) driver.forward = false;
    else driver.move_motors(0);

    pid_total = fabs(pid_total);
    // if(pid_total < 60.0) pid_total = 0.0;
    driver.move_motors(pid_total);

    /* Delay to make 250Hz main loop */
    while (micros() - loop_timer < 4000)
        ;

    loop_timer = micros();
}