#include "Drivetrain.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>

Drivetrain::Drivetrain(int leftMotorPwm1, int leftMotorPwm2, int rightMotorPwm1, int rightMotorPwm2, int leftEncoderA, int leftEncoderB, int rightEncoderA, int rightEncoderB) {
    leftMotor = new Motor(leftMotorPwm1, leftMotorPwm2);
    rightMotor = new Motor(rightMotorPwm1, rightMotorPwm2);

    // if (otos.begin() == false) {
    //     Serial.println("OTOS not detected");
    //     while (1);
    // }

    // while(!otos.begin()) {
    //     Serial.println("OTOS not detected");
    // }

    // otos.calibrateImu();

    leftEncoder = new Encoder(leftEncoderA, leftEncoderB);
    rightEncoder = new Encoder(rightEncoderA, rightEncoderB);

    imu = new Adafruit_BNO08x();

    int retries = 0;
    while (!imu->begin_I2C()) {
        Serial.printf("BNO085 begin failed! retry %d\n", retries);
        delay(100);
        retries++;
        if (retries > 10) {
            Serial.println("BNO085 begin failed after 10 retries, continuing without IMU");
            imuReady = false;
            return;
        }
    }
    Serial.println("BNO085 begin OK");

    // Enable gyroscope and accelerometer reports at 50ms intervals (20Hz)
    // Matches the sensorTask vTaskDelay(50) to prevent event buildup
    if (!imu->enableReport(SH2_GYROSCOPE_CALIBRATED, 10000)) { 
        Serial.println("BNO085: failed to enable gyroscope report");
    }
    if (!imu->enableReport(SH2_ACCELEROMETER, 10000)) { 
        Serial.println("BNO085: failed to enable accelerometer report");
    }
    if (!imu->enableReport(SH2_ROTATION_VECTOR, 10000)) { 
        Serial.println("BNO085: failed to enable rotation vector report");
    }

    Serial.println("BNO085 fully configured");
    imuReady = true;

    // Zero the IMU in constructor by waiting for the first valid rotation vector
    Serial.println("Waiting for initial IMU reading to zero heading...");
    unsigned long startWait = millis();
    bool zeroed = false;
    while (millis() - startWait < 2000 && !zeroed) {
        if (imu->getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
                double q_real = sensorValue.un.rotationVector.real;
                double q_x = sensorValue.un.rotationVector.i;
                double q_y = sensorValue.un.rotationVector.j;
                double q_z = sensorValue.un.rotationVector.k;
                
                hOffset = std::atan2(
                    2 * (q_real * q_z + q_x * q_y),
                    1 - 2 * (q_y * q_y + q_z * q_z)
                );
                zeroed = true;
                Serial.printf("IMU Zeroed! Initial heading offset: %.2f\n", hOffset);
            }
        }
        delay(10);
    }
    if (!zeroed) {
        Serial.println("Warning: IMU failed to provide initial reading for zeroing.");
    }
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
    // otos.getPosition(otosPoseMeasurement);
    // otos.getVelocity(otosVelocityMeasurement);

    otosPoseMeasurement.x = otosPoseMeasurement.x - xOffset;
    otosPoseMeasurement.y = otosPoseMeasurement.y - yOffset;

    // h = otosVelocityMeasurement.h;
    // w = otos_velocity_measurement.h;
    encoderMeasurements.leftEncoderX = leftEncoder->read();
    encoderMeasurements.rightEncoderX = rightEncoder->read();

    if (imuReady) {
        // Drain all available sensor events from the BNO085
        while (imu->getSensorEvent(&sensorValue)) {
            switch (sensorValue.sensorId) {
                case SH2_GYROSCOPE_CALIBRATED:
                    Serial.printf("Gyro: x=%.2f y=%.2f z=%.2f\n",
                        sensorValue.un.gyroscope.x,
                        sensorValue.un.gyroscope.y,
                        sensorValue.un.gyroscope.z);
                    break;
                case SH2_ACCELEROMETER:
                    // sensorValue.un.accelerometer.x, .y, .z available
                    break;
                case SH2_ROTATION_VECTOR:
                    // sensorValue.un.rotationVector.i, .j, .k, .real available
                    double q_real = sensorValue.un.rotationVector.real;
                    double q_x = sensorValue.un.rotationVector.i;
                    double q_y = sensorValue.un.rotationVector.j;
                    double q_z = sensorValue.un.rotationVector.k;
                    // Serial.printf("Rotation Vector: i=%.2f j=%.2f k=%.2f real=%.2f\n",
                    //     q_x,
                    //     q_y,
                    //     q_z,
                    //     q_real);
                    double yaw = std::atan2(
                        2 * (q_real * q_z + q_x * q_y),
                        1 - 2 * (q_y * q_y + q_z * q_z)
                    );
                    otosPoseMeasurement.h = yaw - hOffset;
                    Serial.printf("Yaw: %.2f\n", otosPoseMeasurement.h);
                    //yaw = math.atan2(2 * (q_w * q_z + q_x * q_y), 1 - 2 * (q_y**2 + q_z**2))
                    break;
            }
        }
    }
}

void Drivetrain::zero() {
    xOffset = otosPoseMeasurement.x;
    yOffset = otosPoseMeasurement.y;
    hOffset += otosPoseMeasurement.h;
}