#include "Drivetrain.h"
//#include <Wire.h>
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
    Serial.printf("Encoders - Left: %.2f, Right: %.2f\n", 
                  encoderMeasurements.leftEncoderX, 
                  encoderMeasurements.rightEncoderX);

    unsigned long currentTime = millis();
    if (lastTime > 0) {
        double dt = (currentTime - lastTime) / 1000.0;
        if (dt > 0) {
            double deltaLeft = encoderMeasurements.leftEncoderX - lastLeftEncoder;
            double deltaRight = encoderMeasurements.rightEncoderX - lastRightEncoder;
            
            // Wheel diameter: 32mm, Encoder counts per revolution: 437
            double distance_mm_per_count = (32.0 * 3.14159265359) / 437.0;
            double v_forward_mm_s = ((deltaLeft + deltaRight) / 2.0) * distance_mm_per_count / dt;
            
            otosVelocityMeasurement.x = -v_forward_mm_s * cos(otosPoseMeasurement.h);
            otosVelocityMeasurement.y = -v_forward_mm_s * sin(otosPoseMeasurement.h);
        }
    }
    lastLeftEncoder = encoderMeasurements.leftEncoderX;
    lastRightEncoder = encoderMeasurements.rightEncoderX;
    lastTime = currentTime;

    if (imuReady) {
        if (imu->getSensorEvent(&sensorValue)) {
            switch (sensorValue.sensorId) {
                case SH2_GYROSCOPE_CALIBRATED:
                    otosVelocityMeasurement.h = sensorValue.un.gyroscope.z;
                    break;
                case SH2_ROTATION_VECTOR: {
                    double q_real = sensorValue.un.rotationVector.real;
                    double q_x = sensorValue.un.rotationVector.i;
                    double q_y = sensorValue.un.rotationVector.j;
                    double q_z = sensorValue.un.rotationVector.k;
                    double yaw = std::atan2(
                        2 * (q_real * q_z + q_x * q_y),
                        1 - 2 * (q_y * q_y + q_z * q_z)
                    );
                    yaw = fmod(yaw, 2 * M_PI);
                    otosPoseMeasurement.h = yaw - hOffset;
                    break;
                }
                default:
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