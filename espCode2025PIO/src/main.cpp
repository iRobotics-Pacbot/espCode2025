#include <Arduino.h>
#include <atomic>
#include "dataTypes.h"
#include "UDPPeer.h"


//global vars that each task will atomically interact with
std::atomic<odom> odom_data;
std::atomic<tof> tof_data;
std::atomic<mclPose> mcl_pose_data;
std::atomic<velos> velo_data;

void odomPoller();
void tofPoller();
void refreshUDP();
void refreshControl(); //does this drive the motors?
void refreshPLL();

TaskInfo Tasks[] = {
  {odomPoller, "odomPoller", 512, 1, pdMS_TO_TICKS(1)},
  {tofPoller, "tofPoller", 512, 1, pdMS_TO_TICKS(1)},
  {refreshUDP, "refreshUDP", 2048, 1, pdMS_TO_TICKS(1)},
  {refreshControl, "refreshControl", 4096, 1, pdMS_TO_TICKS(1)},
  {refreshPLL, "refreshPLL", 4096, 1, pdMS_TO_TICKS(1)},
}; 

void genericTask(void *param) {
    TaskInfo *task = (TaskInfo *)param;
    while (1) {
        task->taskFunc();
        vTaskDelay(pdMS_TO_TICKS(task->delayMs));
    }
}


void setup() {
  uint8_t numTasks = sizeof(Tasks) / sizeof(Tasks[0]);
  for (uint8_t i = 0; i < numTasks; i++) { 
    xTaskCreate(genericTask, Tasks[i].name, Tasks[i].stackSize, (void*)&Tasks[i], Tasks[i].priority, NULL);
  }
}
void loop() {;}


void odomPoller(){};
void tofPoller(){};
void refreshUDP(){};
void refreshControl(){}; //does this drive the motors?
void refreshPLL(){};

