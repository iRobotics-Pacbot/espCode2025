#include "UDPPeer.h"
#include <ESPmDNS.h>

UDPPeer::UDPPeer(odom& odom, tof& tof, mclPose& mclpose) : odomRef(odom), tofRef(tof), mclposeRef(mclpose) {
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

    udp.begin(resolvedIP, PORT);
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
    xSemaphoreTake(odomRef.lock, portMAX_DELAY);
    dout.x = odomRef.x;
    dout.y = odomRef.y;
    dout.vx = odomRef.vx;
    dout.vy = odomRef.vy;
    xSemaphoreGive(odomRef.lock);
    udp.beginPacket(LAPTOP_IP, PORT);
    udp.write((uint8_t*)&dout, sizeof(dataToMCL));
    udp.endPacket();
    Serial.println("sent packet");
}

void UDPPeer::receiveData() {
    dataFromMCL din;
    int packetSize = udp.parsePacket();
    if (packetSize) {
        udp.read((uint8_t*)&din, sizeof(dataFromMCL));
        Serial.println("rcv packet");
        xSemaphoreTake(mclposeRef.lock, portMAX_DELAY);
        mclposeRef.x = din.x;
        mclposeRef.y = din.y;
        mclposeRef.vx = din.vx;
        mclposeRef.vy = din.vy;
        mclposeRef.oldX = din.oldX;
        mclposeRef.oldY = din.oldY;
        xSemaphoreGive(mclposeRef.lock);
    }
}

void UDPPeer::shutDown() {

}