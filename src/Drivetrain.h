#pragma once
#include "Motor.h"
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include "sfTk/sfDevOTOS.h"
// #include <Wire.h>
#include "dataTypes.h"
#include "Encoder.h"
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// extern QueueHandle_t tofQueue;
// VL53L4CX sensor_vl53l4cx_sat(&Wire, 0);

class Drivetrain{
// esp32's usb port is "backward"
public:
    Drivetrain(int leftMotorPwm1, int leftMotorPwm2, int rightMotorPwm1, int rightMotorPwm2, int leftEncoderA, int leftEncoderB, int rightEncoderA, int rightEncoderB);
    void setSpeeds(double leftDutyCycle, double rightDutyCycle);
    void readSensors();
    void zero();

    sfe_otos_pose2d_t otosPoseMeasurement;
    sfe_otos_pose2d_t otosVelocityMeasurement;
    EncoderData encoderMeasurements;

private:
    Motor* leftMotor;
    Motor* rightMotor;

    Encoder* leftEncoder;
    Encoder* rightEncoder;

    double leftDutyCycle;
    double rightDutyCycle;

    float xOffset = 0;
    float yOffset = 0;

    // QwiicOTOS otos; // optical tracking odometry sensor
    Adafruit_BNO08x* imu;
    sh2_SensorValue_t sensorValue;
    bool imuReady = false;
};
