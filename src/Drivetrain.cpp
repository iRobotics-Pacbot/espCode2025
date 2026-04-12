#include "Drivetrain.h"

Drivetrain::Drivetrain(int leftMotorPwm1, int leftMotorPwm2, int rightMotorPwm1, int rightMotorPwm2, int leftEncoderA, int leftEncoderB, int rightEncoderA, int rightEncoderB) {
    leftMotor = new Motor(leftMotorPwm1, leftMotorPwm2);
    rightMotor = new Motor(rightMotorPwm1, rightMotorPwm2);

    // if (otos.begin() == false) {
    //     Serial.println("OTOS not detected");
    //     while (1);
    // }

    while(!otos.begin()) {
        Serial.println("OTOS not detected");
    }

    otos.calibrateImu();

    leftEncoder = new Encoder(leftEncoderA, leftEncoderB);
    rightEncoder = new Encoder(rightEncoderA, rightEncoderB);
}

void Drivetrain::setSpeeds(double leftDutyCycle, double rightDutyCycle) {
    if (leftDutyCycle > 0) {
        leftMotor->setThrottle(1 - leftDutyCycle);
    } else {
        leftMotor->setThrottle(leftDutyCycle);
    }
    rightMotor->setThrottle(rightDutyCycle);
}

void Drivetrain::readSensors() {
    otos.getPosition(otosPoseMeasurement);
    otos.getVelocity(otosVelocityMeasurement);

    otosPoseMeasurement.x = otosPoseMeasurement.x - xOffset;
    otosPoseMeasurement.y = otosPoseMeasurement.y - yOffset;

    // h = otosVelocityMeasurement.h;
    // w = otos_velocity_measurement.h;
    encoderMeasurements.leftEncoderX = leftEncoder->read();
    encoderMeasurements.rightEncoderX = rightEncoder->read();
}

void Drivetrain::zero() {
    xOffset = otosPoseMeasurement.x;
    yOffset = otosPoseMeasurement.y;
}