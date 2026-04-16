#include "UDPPeer.h"
#include <ESPmDNS.h>
using namespace std;


#define DEBUG

UDPPeer::UDPPeer(SafeStruct<OdoPose>& odom, SafeStruct<TOF_t>& tof, SafeStruct<MclPose>& mclpose, SafeStruct<Path>& path) : odomRef(odom), tofRef(tof), mclposeRef(mclpose), pathRef(path){
    // WiFi.mode(WIFI_STA);
    // WiFi.disconnect();
    // WiFi.begin(ssid, password);
    // delay(10000);

    int n = WiFi.scanNetworks();
    for (int i = 0; i < n; i++) {
        Serial.printf("SSID: %s  BSSID: %s  Channel: %d  RSSI: %d\n",
            WiFi.SSID(i).c_str(),
            WiFi.BSSIDstr(i).c_str(),
            WiFi.channel(i),
            WiFi.RSSI(i));
    }

    // WiFi.mode(WIFI_STA);
    // WiFi.disconnect(true);  // true = also erase stored credentials
    // delay(1000);            // give it a moment to fully reset
    // WiFi.begin(ssid, password);

    WiFi.mode(WIFI_OFF);        // fully power down radio
    delay(1000);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);       // disable power save
    WiFi.setTxPower(WIFI_POWER_19_5dBm);  // max TX power
    WiFi.disconnect(true, true); // erase credentials too (second true = erase NVS)
    delay(1000);
    WiFi.begin(ssid, password);
    // WiFi.begin(ssid, password, 9, bssid, true);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
            Serial.printf("Disconnect reason: %d\n", info.wifi_sta_disconnected.reason);
        }
    });
    
    while (WiFi.status() != WL_CONNECTED) {
        Serial.printf("\nFailed! Status code: %d\n", WiFi.status());
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

char UDPPeer::Update() {
    sendMCLData();
    return receiveData();
}

void UDPPeer::sendMCLData() {
    OdoPose odom;
    TOF_t tof;
    odom = odomRef.get();
    tof = tofRef.get();
    // for (int i = 0; i < TOF_COUNT; i++) {
    //     Serial.printf("Distance %d: %f mm\n", i, tof.distances[i]);
    // }
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
        // Serial.println("sent packet");
    #endif
}

char UDPPeer::receiveData() {
    Packet packet;
    int packetSize = udp.parsePacket();
    if (packetSize && packetSize <= sizeof(Packet)) {
        udp.read(reinterpret_cast<uint8_t*>(&packet), packetSize);
        udp.flush();
        switch(packet.type) {
            case 'a': {
                #ifdef DEBUG
                    // Serial.print("recv mcl data");
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
                // Serial.print(din->x);
                // Serial.print(" | ");
                // Serial.print(din->y);
                // Serial.println("");
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

    return 'F';
}

void UDPPeer::shutDown() {

}