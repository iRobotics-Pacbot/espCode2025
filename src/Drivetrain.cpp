#include "Drivetrain.h"
#include <Wire.h>
#include <ISM330DLCSensor.h>

// ISM330DLC register addresses for software reset
#define ISM330DLC_CTRL3_C 0x12

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

    imu = new ISM330DLCSensor(&Wire);

    // Software reset the IMU to clear any stale I2C state
    // This is critical when sharing the I2C bus with ToF sensors
    uint8_t resetVal = 0x01; // SW_RESET bit in CTRL3_C
    Wire.beginTransmission(0x6B); // ISM330DLC default address (0xD6 >> 1)
    Wire.write(ISM330DLC_CTRL3_C);
    Wire.write(resetVal);
    byte resetErr = Wire.endTransmission();
    if (resetErr != 0) {
        // Try alternate address 0x6A (SDO/SA0 pulled LOW)
        Wire.beginTransmission(0x6A);
        Wire.write(ISM330DLC_CTRL3_C);
        Wire.write(resetVal);
        resetErr = Wire.endTransmission();
        if (resetErr != 0) {
            Serial.printf("IMU software reset failed at both 0x6B and 0x6A, error: %d\n", resetErr);
        } else {
            Serial.println("IMU found at 0x6A (not default 0x6B)");
        }
    }
    delay(50); // Wait for reset to complete

    int retries = 0;
    while (imu->begin() != 0) {
        Serial.printf("IMU begin() failed! retry %d\n", retries);
        delay(100);
        retries++;
        if (retries > 10) {
            Serial.println("IMU begin() failed after 10 retries, continuing without IMU");
            imuReady = false;
            return;
        }
    }
    Serial.println("IMU begin() OK");

    if (imu->Enable_X() != 0) {
        Serial.println("IMU Enable_X() failed!");
        imuReady = false;
        return;
    }
    Serial.println("IMU Enable_X() OK");

    if (imu->Enable_G() != 0) {
        Serial.println("IMU Enable_G() failed!");
        imuReady = false;
        return;
    }
    Serial.println("IMU Enable_G() OK");
    imuReady = true;

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
        int32_t acceleration[3];
        int32_t angular_rate[3];
        if (imu->Get_X_Axes(acceleration) != 0) {
            Serial.println("IMU Get_X_Axes failed");
        }
        if (imu->Get_G_Axes(angular_rate) != 0) {
            Serial.println("IMU Get_G_Axes failed");
        } else {
            Serial.print("IMU gyro Z: ");
            Serial.println(angular_rate[2]);
        }
    }
}

void Drivetrain::zero() {
    xOffset = otosPoseMeasurement.x;
    yOffset = otosPoseMeasurement.y;
}