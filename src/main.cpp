#include <Arduino.h>
#include <atomic>
#include "dataTypes.h"
#include "UDPPeer.h"
#include "Wire.h"
#include "Odo.h"
#include "TOF.h"
#include "Encoder_test.h"


void testUDP(UDPPeer* udp);

//class task instantiations 
UDPPeer *myPeer;
TOF *tof;
Odo *odo;
//....

void updTask(void* param)
{
  UDPPeer *myPeer = (UDPPeer*) param;

  while(1) {
    myPeer->Update();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void odoTask(void* param)
{
  Odo *odo = (Odo*) param;

  while(1) {
    odo->update();
    vTaskDelay(50/ portTICK_PERIOD_MS);
  }
}

void tofTask(void* param)
{
  TOF *tof = (TOF*) param;

  uint8_t sensorID;
  while(true) {
    if (xQueueReceive(tofQueue, &sensorID, portMAX_DELAY))
    {
      tof->update(sensorID);
    }
  }
}




void setup() {
  // Setup Serial
  Serial.begin(115200);

  // Setup Wire
  Wire.begin();
  Wire.setClock(100000);


  //Safestruct instantiation 
  SafeStruct<OdoPose> odoStruct;
  SafeStruct<TOF_t> tofStruct;
  SafeStruct<MclPose> mclPoseStruct;
  SafeStruct<Velos> veloStruct; 
  SafeStruct<Path> pathStruct;

  //task class instantiation
  myPeer = new UDPPeer(odoStruct, tofStruct, mclPoseStruct, pathStruct);
  // tof = new TOF(tofStruct);
  // odo = new Odo(odoStruct);

  
  // xTaskCreate(updTask, "UDP Task", 2048, (void*)myPeer, 1, NULL);
  // xTaskCreate(tofTask, "TOF Task", 2048, (void*)tof, 1, NULL);
  // xTaskCreate(odoTask, "Odo Task", 2048, (void*)odo, 1, NULL);
  //testEncoder(); //Added this for testing
  testUDP(myPeer);


}
void loop() {;}







//TEST FUNCTIONS
void testUDP(UDPPeer* udp) {
  size_t strSize = 26; //keep this under 64
  char data[strSize] = "abcdefghijklmnopqrstuvwxyz"; 

  size_t ct = 0;
  while (1) {
    udp->sendString(data, strSize);
    udp->receiveData();
    delay(1000);
    ct++;

    char end = data[strSize-1];
    for (uint8_t i = strSize; i > 0; i--) {
      data[i] = data[i-1];
    }
    data[0] = end;
  }
}
