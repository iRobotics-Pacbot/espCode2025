#include "UDPPeer.h"
#include <ESPmDNS.h>
using namespace std;


#define DEBUG

UDPPeer::UDPPeer(SafeStruct<OdoPose>& odom, SafeStruct<TOF_t>& tof, SafeStruct<MclPose>& mclpose, SafeStruct<Path>& path) : odomRef(odom), tofRef(tof), mclposeRef(mclpose), pathRef(path){
    Serial.println("entered UDP constructor");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        #ifdef DEBUG
            Serial.print(".");
        #endif
        delay(500); 
    }
    
    Serial.println("\nConnected to WiFi!");

    IPAddress resolvedIP;
    if (MDNS.begin(mdns)) {
        resolvedIP = MDNS.IP(0); 
        #ifdef DEBUG
            Serial.printf("Resolved IP: %s\n", resolvedIP.toString().c_str());
        #endif
    } else {
        Serial.println("MDNS query failed!");
        return;
    }

    udp.begin(resolvedIP, PORT);
    #ifdef DEBUG
        Serial.println("\nUDP socket initialized");
    #endif
}

UDPPeer::~UDPPeer() {
    shutDown();
}

void UDPPeer::Update() {
    sendMCLData();
    receiveData();
}

void UDPPeer::sendMCLData() {
    OdoPose odom;
    TOF_t tof;
    odom = odomRef.get();
    tof = tofRef.get();
    struct {
        char packetID;
        OdoPose odom;
        TOF_t tof;
    } dout;
    dout.packetID = 'a';
    dout.odom = odom;
    dout.tof = tof;
    sendGeneric(&dout, sizeof(dout));
}

void UDPPeer::sendString(char* string, size_t len) {
    struct {
        char packetID;
        char string[64];
    } dout;
    string[63] = 0; //just make sure we got that null term
    dout.packetID = 'b';
    strncpy(dout.string, string, len);
    sendGeneric(&dout, len + 1);
}

void UDPPeer::sendGeneric(void* structToSend, size_t structSize) {
    if (!structToSend) {
        #ifdef DEBUG
            Serial.print("asked to send null");
        #endif
        return;
    }
    udp.beginPacket(LAPTOP_IP, PORT);
    udp.write((uint8_t*)structToSend, structSize);
    udp.endPacket();
    #ifdef DEBUG
        Serial.println("sent packet");
    #endif
}

void UDPPeer::receiveData() {
    Packet packet;
    int packetSize = udp.parsePacket();
    if (packetSize && packetSize <= sizeof(Packet)) {
        udp.read(reinterpret_cast<uint8_t*>(&packet), packetSize);
        udp.flush();
        switch(packet.type) {
            case 'a': {
                #ifdef DEBUG
                    Serial.println("recv mcl data");
                #endif
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
            case 'b': {
                #ifdef DEBUG
                    Serial.println("recv path data");
                #endif
                DataFromPathPlanner* din = reinterpret_cast<DataFromPathPlanner*>(&packet.buf);
                Path path;
                path.targetX = din->targetX;
                path.targetY = din->targetY;
                pathRef.set(path);
                break;
            }
            default : { //gonna treat it as a char array and print
                #ifdef DEBUG
                    Serial.printf("NonstandardPackedID:%c Contents:%.*s\n", packet.type, packetSize-1, packet.buf);
                #endif
                break;
            }
        }
    }
}

void UDPPeer::shutDown() {

}