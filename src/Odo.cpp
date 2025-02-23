#include "Odo.h"

Odo::Odo(SafeStruct<OdoPose>& odom): odom(odom) {
    
    while(otos.begin() == false)
    {
        Serial.println("Failed to setup odom");
        delay(100);
    }
    Serial.println("OTOS connected!");

    Serial.println("Calib imu");

    otos.calibrateImu();
    otos.resetTracking();
    

}

void Odo::update(){
    OdoPose update;

    otos.getPosition(update.pos);
    otos.getVelocity(update.vel);
    otos.getAcceleration(update.acc);

    otos.getPositionStdDev(update.pos_std);
    otos.getVelocityStdDev(update.vel_std);

    odom.set(update);

}

