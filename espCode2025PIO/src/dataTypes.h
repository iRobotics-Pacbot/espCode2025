#ifndef DATATYPES_H
#define DATATYPES_H

struct Odom {
    float x;
    float y;
    float vx;
    float vy;
    float stdvx;
    float stdvy;

};

constexpr uint8_t TOF_COUNT = 6;
struct TOF {
    float distances[TOF_COUNT];
    float stds[TOF_COUNT];
};


template <typename T> class SafeStruct {
    private:
        xSemaphoreHandle lock;
        T data;
        // #ifdef DEBUG
        // size_t ct; 
        // #endif

    public:
        SafeStruct() {vSemaphoreCreateBinary(lock);}
        T get() {xSemaphoreTake(lock, portMAX_DELAY); T out = data; xSemaphoreGive(lock); return out;}
        set(T data) {xSemaphoreTake(lock, portMAX_DELAY); this->data = data; xSemaphoreGive(lock);}
};

struct MclPose {
    float x;
    float y;
    float vx;
    float vy;
    float oldX;
    float oldY;
};

struct Velos {
    float vx;
    float vy;
};

struct TaskInfo {
    std::function<void()> taskFunc;
    const char *name;
    uint16_t stackSize;
    UBaseType_t priority;
    TickType_t delay;
};





#endif // DATATYPES_H