#include "UDPPeer.h"
#include <ESPmDNS.h>

UDPPeer::UDPPeer(const char *peerIP, uint16_t peerPort, QueueHandle_t *sendBuf, QueueHandle_t *recvBuf, size_t ticksToWait) {
    this->peerIP = peerIP;
    this->peerPort = peerPort;
    this->sendBuf = sendBuf;
    this->recvBuf = recvBuf;
    this->ticksToWait = ticksToWait;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500); 
    }
    
    Serial.println("\nConnected to WiFi!");

    IPAddress resolvedIP;
    if (MDNS.begin(mdns)) {
        resolvedIP = MDNS.IP(0); 
        Serial.printf("Resolved IP: %s\n", resolvedIP.toString().c_str());
    } else {
        Serial.println("MDNS query failed!");
        return;
    }

    udp.begin(resolvedIP, peerPort);
    Serial.println("\nUDP socket initialized");
}

UDPPeer::~UDPPeer() {
    shutDown();
}

void UDPPeer::Update() {
    sendData();
    receiveData();
}

void UDPPeer::sendData() {
    dataToMCL dout;
    if (xQueueReceive(*sendBuf, &dout, ticksToWait)) {
        //Serial.println("pop smth from send Q");
        udp.beginPacket(peerIP, peerPort);
        udp.write((uint8_t*)&dout, sizeof(dataToMCL));
        udp.endPacket();
        Serial.println("sent packet");
    } else {
        //Serial.println("send Q pop fail");
    }
}

void UDPPeer::receiveData() {
    dataToMCL din;
    int packetSize = udp.parsePacket();
    if (packetSize) {
        udp.read((uint8_t*)&din, sizeof(dataToMCL));
        Serial.println("rcv packet");
        xQueueOverwrite(*recvBuf, &din);
    }
}

void UDPPeer::shutDown() {

}