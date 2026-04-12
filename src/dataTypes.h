#ifndef DATATYPES_H
#define DATATYPES_H
#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"

typedef sfe_otos_pose2d_t Pose2D;

constexpr uint8_t TOF_COUNT = 6;
struct TOF_t {
    float distances[TOF_COUNT];
    float stds[TOF_COUNT];
};


template <typename T> class SafeStruct {
private:
    SemaphoreHandle_t lock = nullptr;
    T data;
public:
    SafeStruct() {}
    void init() { lock = xSemaphoreCreateMutex(); } // call this from setup()
    T get() { xSemaphoreTake(lock, portMAX_DELAY); T out = data; xSemaphoreGive(lock); return out; }
    void set(T d) { xSemaphoreTake(lock, portMAX_DELAY); data = d; xSemaphoreGive(lock); }
};

struct MclPose {
    float x;
    float y;
    float vx;
    float vy;
    float oldX;
    float oldY;
};

struct EncoderData {
    float leftEncoderX;
    float rightEncoderX;
};

struct Path {
    float targetX;
    float targetY;
};

struct Velos {
    float vx;
    float vy;
};

struct OdoPose {
    Pose2D pos;
    Pose2D vel;
    Pose2D acc;

    Pose2D pos_std;
    Pose2D vel_std;
};
 

struct TaskInfo {
    std::function<void()> taskFunc;
    const char *name;
    uint16_t stackSize;
    UBaseType_t priority;
    TickType_t delay;
};



#endif // DATATYPES_H