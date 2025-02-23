#pragma once
#include <Wire.h>
// #include <vl53l4cx_class.h>
#include "dataTypes.h"

extern QueueHandle_t tofQueue;
// VL53L4CX sensor_vl53l4cx_sat(&Wire, 0);

class TOF{
public:
    TOF(SafeStruct<TOF_t>& tof_t);
    void update(uint8_t sensorID);

private:
    SafeStruct<TOF_t>& tof_t;
};
