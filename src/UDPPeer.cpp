#include "UDPPeer.h"
#include <ESPmDNS.h>

UDPPeer::UDPPeer(SafeStruct<OdoPose>& odom, SafeStruct<TOF_t>& tof, SafeStruct<MclPose>& mclpose) : odomRef(odom), tofRef(tof), mclposeRef(mclpose) {
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
    DataToMCL dout;
    OdoPose odom;
    TOF_t tof;
    odom = odomRef.get();
    tof = tofRef.get();
    dout.set(tof.distances, tof.stds, odom.pos.x, odom.pos.y, odom.vel.x, odom.vel.y, odom.vel_std.x, odom.vel_std.y);
    udp.beginPacket(LAPTOP_IP, PORT);
    udp.write((uint8_t*)&dout, sizeof(DataToMCL));
    udp.endPacket();
    Serial.println("sent packet");
}

void UDPPeer::receiveData() {
    Packet packet;
    int packetSize = udp.parsePacket();
    if (packetSize <= sizeof(Packet)) {
        udp.read(reinterpret_cast<uint8_t*>(&packet), packetSize);
        Serial.println("rcv packet");
        switch(packet.type) {
            case 'r':
                DataFromMCL* din = reinterpret_cast<DataFromMCL*>(&packet.buf);
                MclPose mclPose;
                mclPose.x = din->x;
                mclPose.y = din->y;
                mclPose.vx = din->vx;
                mclPose.vy = din->vy;
                mclPose.oldX = din->oldX;
                mclPose.oldY = din->oldY;
                mclposeRef.set(mclPose);
                break;
        }
    }
}

void UDPPeer::shutDown() {

}