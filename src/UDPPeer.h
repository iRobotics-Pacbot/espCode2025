#ifndef UDPEER_H
#define UDPEER_H

#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include "dataTypes.h"

class UDPPeer {
public:
    UDPPeer(SafeStruct<Odom>& odom, SafeStruct<TOF>& tof, SafeStruct<MclPose>& mclpose);

    ~UDPPeer();

    void Update();


private:
    WiFiUDP udp;
    SafeStruct<Odom>& odomRef;
    SafeStruct<TOF>& tofRef;
    SafeStruct<MclPose>& mclposeRef;

    static constexpr char* LAPTOP_IP = "192.168.0.101";
    static constexpr uint16_t PORT = 8089;
    static constexpr char* ssid = "Pacbot_Server";
    static constexpr char* password = "Pacbot#2024!";
    static constexpr char* mdns = "uiucpacbot";
    static constexpr size_t MAX_PACKET_SIZE = 1024;

    struct Packet {
        uint8_t type;
        uint8_t buf[MAX_PACKET_SIZE];
    };

    struct DataToMCL {
        float distances[TOF_COUNT];
        float stds[TOF_COUNT];
        float x;
        float y;
        float vx;
        float vy;
        float stdvx;
        float stdvy;

        void set(float dist[TOF_COUNT], float std[TOF_COUNT], float x, float y, float vx, float vy, float stdvx, float stdvy) {
            for (uint8_t i =0; i < TOF_COUNT; i++) {
                distances[i] = dist[i];
                stds[i] = std[i];
            }
            this->x = x;
            this->y = y;
            this->vx = vx;
            this->vy = vy;
            this->stdvx = stdvx;
            this->stdvy = stdvy;
        }
    };

    struct DataFromMCL {
        float x = 0;
        float y = 0;
        float vx = 0;
        float vy = 0;
        float oldX = 0;
        float oldY = 0;
    };

    void sendData();
    void receiveData();
    void shutDown();
};

#endif