#ifndef UDPEER_H
#define UDPEER_H

#include <WiFi.h>
#include <freertos/FreeRTOS.h>


struct dataToMCL;
struct dataFromMCL;

class UDPPeer {
public:
    UDPPeer(const char *peerIP, uint16_t peerPort, QueueHandle_t *sendBuf, QueueHandle_t *recvBuf, size_t ticksToWait);

    ~UDPPeer();

    void Update();


private:
    const char *peerIP;
    uint16_t peerPort;
    size_t ticksToWait;
    WiFiUDP udp;

    const char* ssid = "Pacbot_Server";
    const char* password = "Pacbot#2024!";
    const char* mdns = "uiucpacbot";

    QueueHandle_t *sendBuf;
    QueueHandle_t *recvBuf;

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
        float time = 0;
    };

    void sendData();
    void receiveData();
    void shutDown();
};

#endif