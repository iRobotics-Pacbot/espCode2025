#include <Arduino.h>
#include <atomic>
#include "dataTypes.h"
#include "UDPPeer.h"
#include "Wire.h"
#include "Odo.h"
#include "TOF.h"


//class task instantiations 
UDPPeer *myPeer;
TOF *tof;
Odo *odo;
//....

TaskInfo Tasks[] = {
  {[]() { myPeer->Update(); }, "refreshUDP", 2048, 1, pdMS_TO_TICKS(1)}

//list other tasks here in array


};

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


void genericTask(void *param) {
    TaskInfo *task = (TaskInfo *)param;
    while (1) {
        task->taskFunc();
        vTaskDelay(task->delay);
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

  //task class instantiation
  myPeer = new UDPPeer(odoStruct, tofStruct, mclPoseStruct);
  tof = new TOF(tofStruct);
  odo = new Odo(odoStruct);

  
  
  xTaskCreate(tofTask, "TOF Task", 2048, (void*)tof, 1, NULL);
  xTaskCreate(odoTask, "Odo Task", 2048, (void*)odo, 1, NULL);

  //task kickoff
  // @todo remove this
  uint8_t numTasks = sizeof(Tasks) / sizeof(Tasks[0]);
  for (uint8_t i = 0; i < numTasks; i++) { 
    xTaskCreate(genericTask, Tasks[i].name, Tasks[i].stackSize, (void*)&Tasks[i], Tasks[i].priority, NULL);
  }
}
void loop() {;}



