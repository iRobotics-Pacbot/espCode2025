#include "tofs.h"
#include "Wire.h"
tofs::tofs(){
     //, {&Wire, 15}, {&Wire, 14}, {&Wire, 13}};
}

void tofs::tofInit() {
    // Serial.printf("tof init\n");
    // auto& tof = sensors[0];

    // tof.begin();
    // tof.VL53L4CX_Off();
    // tof.setXShutPin(12);
    // tof.InitSensor(0x12);
    // tof.VL53L4CX_ClearInterruptAndStartMeasurement();

    // Serial.println("READY -----------");
    for (int i = 0; i < NUM_TOFS; i++) {
        auto& tof = sensors[i];
        int status = tof.begin();
        if (status != 0) {
            while (true) {
                Serial.printf("tof %d: begin: %d\n", i, status);
                delay(1000);
            }
        }
        delay(1000);
        tof.VL53L4CX_Off(); // from STM example
        delay(1000);
        status = tof.InitSensor(0x12);
        // if (status != 0) {
        //     while (true) {
        //         Serial.printf("tof %d: init sensor: %d\n", i, status);
        //         delay(1000);
        //     }
        // }
        // if (status != 0) {
        //     while (true) {
        //         Serial.printf("tof %d: timing budget: %d\n", i, status);
        //     }
        // }
        // status = tof.VL53L4CX_StartMeasurement();
        // if (status != 0) {
        //     while (true) {
        //         Serial.printf("tof %d: start: %d\n", i, status);
        //         delay(1000);
        //     }
        // }
        status = tof.VL53L4CX_ClearInterruptAndStartMeasurement();
        // if (status != 0) {
        //     while (true) {
        //         Serial.printf("tof %d: clear: %d\n", i, status);
        //         delay(1000);
        //     }
        // }

        // the number of microseconds that the sensor will take for each measurement
        status = tof.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(8000);
        // if(status != 0){
        //     while (true){
        //         Serial.printf("tof %d: budget: %d\n", i, status);
        //         delay(1000);
        //     }
        // }
        
        // the maximum distance that the sensor can read, not sure what units
        status = tof.VL53L4CX_SetTuningParameter(VL53L4CX_TUNINGPARM_RESET_MERGE_THRESHOLD, 1000);
        // if(status != 0){
        //     while (true){
        //         Serial.printf("tof %d: merge thresh: %d\n", i, status);
        //         delay(1000);
        //     }
        // }
        
    }
}

void tofs::tofBeginReadings() {
    Serial.printf("begin\n");
    for (int i = 0; i < NUM_TOFS; i++) {
        auto& tof = sensors[i];
        int status = tof.VL53L4CX_ClearInterruptAndStartMeasurement();
        if (status != 0) {
            Serial.printf("tof %d: clear: %d\n", i, status);
        }
    }
}

void tofs::tofUpdateReadings() {
    for (int i = 0; i < NUM_TOFS; i++) {
        auto& tof = sensors[i];

        int status;
        status = tof.VL53L4CX_WaitMeasurementDataReady();
        if (status != 0) {
            Serial.printf("tof %d: data ready: %d\n", i, status);
            continue;
        }

        VL53L4CX_MultiRangingData_t results;
        status = tof.VL53L4CX_GetMultiRangingData(&results);
        if (status != 0) {
            Serial.printf("tof %d: get data: %d\n", i, status);
            continue;
        }

        // uint16_t min_distance = 0xffff;
        // for (int j = 0; j < results.NumberOfObjectsFound; j++) {
        //     int16_t range = results.RangeData[j].RangeMilliMeter;
        //     if (range > 0 && range < min_distance) {
        //         min_distance = results.RangeData[j].RangeMilliMeter;
        //     }
        // }
        // tofDistance[i] = min_distance;

        tofDistance[i] = results.RangeData->RangeMinMilliMeter;

        Serial.printf("tof %d: %d\n", i, tofDistance[i]);
    }
}


double tofs::tofGet(int which) {
    Serial.printf("Get TOF %d: %d\n", which, tofDistance[which]);
    return tofDistance[which] / 25.4; // convert millimeters to inches
}

