#ifndef DATATYPES_H
#define DATATYPES_H

struct odom {
    xSemaphoreHandle lock;
    float x;
    float y;
    float vx;
    float vy;
    size_t ct;

    odom(){
        vSemaphoreCreateBinary(lock);
    }
};

struct tof {
    xSemaphoreHandle lock;
    float tofA;
    float tofB;
    float tofC;
    size_t ct;

    tof(){
        vSemaphoreCreateBinary(lock);
    }
};

struct mclPose {
    xSemaphoreHandle lock;
    float x;
    float y;
    float vx;
    float vy;
    float oldX;
    float oldY;

    mclPose(){
        vSemaphoreCreateBinary(lock);
    }
};

struct velos {
    xSemaphoreHandle lock;
    float vx;
    float vy;

    velos(){
        vSemaphoreCreateBinary(lock);
    }
};


// struct TaskInfo{
//     void (*taskFunc)();
//     const char *name;
//     uint16_t stackSize;
//     UBaseType_t priority;
//     uint16_t delayMs;     
// };

struct TaskInfo {
    std::function<void()> taskFunc;
    const char *name;
    uint16_t stackSize;
    UBaseType_t priority;
    TickType_t delayMs;
};





#endif // DATATYPES_H