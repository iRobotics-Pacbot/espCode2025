#include "TOF.h"

QueueHandle_t tofQueue;


void IRAM_ATTR tofISR0();
void IRAM_ATTR tofISR1();
void IRAM_ATTR tofISR2();
void IRAM_ATTR tofISR3();
void IRAM_ATTR tofISR4();
void IRAM_ATTR tofISR5();

TOF::TOF(SafeStruct<TOF_t>& tof_t) : tof_t(tof_t) {
    tofQueue = xQueueCreate(10, sizeof(uint8_t));


    // Setup tofs
    // @tofo someone actually do it
    

    // Attach correct ISRS
    // @todo make them the right pins
    attachInterrupt(digitalPinToInterrupt(2), tofISR0, FALLING);
    attachInterrupt(digitalPinToInterrupt(3), tofISR1, FALLING);
    attachInterrupt(digitalPinToInterrupt(4), tofISR2, FALLING);
    attachInterrupt(digitalPinToInterrupt(5), tofISR3, FALLING);
    attachInterrupt(digitalPinToInterrupt(4), tofISR4, FALLING);
    attachInterrupt(digitalPinToInterrupt(5), tofISR5, FALLING);
}

void TOF::update(uint8_t sensorID) {
    // @todo actually update the value

}

// ISRs
void IRAM_ATTR tofISR0() {
    uint8_t id = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(tofQueue, &id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR tofISR1() {
    uint8_t id = 1;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(tofQueue, &id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR tofISR2() {
    uint8_t id = 2;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(tofQueue, &id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR tofISR3() {
    uint8_t id = 3;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(tofQueue, &id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR tofISR4() {
    uint8_t id = 4;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(tofQueue, &id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR tofISR5() {
    uint8_t id = 5;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(tofQueue, &id, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}