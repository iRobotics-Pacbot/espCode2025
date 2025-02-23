#pragma once
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "dataTypes.h"
#include "Wire.h"

class Odo{
public:
    Odo(SafeStruct<OdoPose>& odom);
    void update();

private:
    SafeStruct<OdoPose>& odom;
    QwiicOTOS otos;
};