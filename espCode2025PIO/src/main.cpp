#include <Arduino.h>
#include <atomic>
#include "dataTypes.h"
#include "UDPPeer.h"



//class task instantiations 
UDPPeer *myPeer;
//....

TaskInfo Tasks[] = {
  {[]() { myPeer->Update(); }, "refreshUDP", 2048, 1, pdMS_TO_TICKS(1)}

//list other tasks here in array




};


void genericTask(void *param) {
    TaskInfo *task = (TaskInfo *)param;
    while (1) {
        task->taskFunc();
        vTaskDelay(task->delay);
    }
}


void setup() {
  //Safestruct instantiation 
  SafeStruct<Odom> odoStruct;
  SafeStruct<TOF> tofStruct;
  SafeStruct<MclPose> mclPoseStruct;
  SafeStruct<Velos> veloStruct; 

  //task class instantiation
  myPeer = new UDPPeer(odoStruct, tofStruct, mclPoseStruct);

  
  

  //task kickoff
  uint8_t numTasks = sizeof(Tasks) / sizeof(Tasks[0]);
  for (uint8_t i = 0; i < numTasks; i++) { 
    xTaskCreate(genericTask, Tasks[i].name, Tasks[i].stackSize, (void*)&Tasks[i], Tasks[i].priority, NULL);
  }
}
void loop() {;}



