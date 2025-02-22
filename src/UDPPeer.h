#ifndef UDPEER_H
#define UDPEER_H

#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include "dataTypes.h"


struct dataToMCL;
struct dataFromMCL;

class UDPPeer {
public:
    UDPPeer(odom& odom, tof& tof, mclPose& mclpose);

    ~UDPPeer();

    void Update();


private:
    WiFiUDP udp;
    odom& odomRef;
    tof& tofRef;
    mclPose& mclposeRef;

    const char* LAPTOP_IP = "192.168.0.101";
    const uint16_t PORT = 8089;
    const char* ssid = "Pacbot_Server";
    const char* password = "Pacbot#2024!";
    const char* mdns = "uiucpacbot";

    struct dataToMCL {
        float tofA = 0;
        float tofB = 0;
        float tofC = 0;
        float x = 0;
        float y = 0;
        float vx = 0;
        float vy = 0;
        float time = 0;
    };

    struct dataFromMCL {
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