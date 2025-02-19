#include <Arduino.h>
#include <atomic>
#include "dataTypes.h"
#include "UDPPeer.h"


//global vars that each task will atomically interact with
odom odom_g;
tof tof_g;
mclPose mcl_g;
velos velo_g;



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
        vTaskDelay(pdMS_TO_TICKS(task->delayMs));
    }
}


void setup() {
  //task class instantiation
  myPeer = new UDPPeer(odom_g, tof_g, mcl_g);

  //task kickoff
  uint8_t numTasks = sizeof(Tasks) / sizeof(Tasks[0]);
  for (uint8_t i = 0; i < numTasks; i++) { 
    xTaskCreate(genericTask, Tasks[i].name, Tasks[i].stackSize, (void*)&Tasks[i], Tasks[i].priority, NULL);
  }
}
void loop() {;}


