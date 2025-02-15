#ifndef DATATYPES_H
#define DATATYPES_H

struct odom {
    float x;
    float y;
};

struct tof {
    float tofA;
    float tofB;
    float tofC;
};

struct mclPose {
    float x;
    float y;
    float vx;
    float vy;
};

struct velos {
    float vx;
    float vy;
};


struct TaskInfo{
    void (*taskFunc)();
    const char *name;
    uint16_t stackSize;
    UBaseType_t priority;
    uint16_t delayMs;     
};










#endif // DATATYPES_H