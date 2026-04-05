#include <Arduino.h>
#include <atomic>
#include <cmath>
#include <vl53l4cx_class.h> 
#include "dataTypes.h"
#include "UDPPeer.h"
#include "Wire.h"
#include "Odo.h"
#include "TOF.h"
#include "Encoder_test.h"
#include "RRT.h"
#include "Astar.h"
#include "WallDetector.h"

const int LDO2_ENABLE_PIN = 17;
const int xshutPins[6] = {38, 39, 40, 41, 42, 12};

VL53L4CX sensors[6] = {
  VL53L4CX(&Wire, xshutPins[0]),
  VL53L4CX(&Wire, xshutPins[1]),
  VL53L4CX(&Wire, xshutPins[2]),
  VL53L4CX(&Wire, xshutPins[3]),
  VL53L4CX(&Wire, xshutPins[4]),
  VL53L4CX(&Wire, xshutPins[5])
};

uint8_t sensorAddresses[6] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
std::vector<double> sensor_measurements;

double tree_width = 500;
double tree_height = 500;

std::vector<std::vector<double>> sensor_poses = {{41, 0, 0},
                                                 {49, 25, 0.25 * M_PI},
                                                 {-49, 25, 0.75 * M_PI},
                                                 {-41, 0, M_PI},
                                                 {-49, -25, 1.25 * M_PI},
                                                 {49, -25, 1.75 * M_PI}};

std::vector<Line> vert;
std::vector<Line> horiz;
WallDetector detector = WallDetector(sensor_poses);

void plannerStep() {
    std::vector<Line> tree;
    tree.push_back({{0, 0}, {0, 0}});

    int i = 0;
    int last_tree_len = tree.size();
    int path_search_radius = 50;

    while (i < 100) {
        Point point = createRandomPoint(tree_width, tree_height);
        std::vector<Point> closestPoints = findClosestPoints(tree, point, 50);
        std::vector<Line> newLines;

        for (const Point& cp : closestPoints) {
            newLines.push_back(pathBetweenPoints(point, cp));
        }

        for (const Line& line : newLines) {
            if (!pointIsBehindWall(line, point, vert, horiz)) {
                tree.push_back(line);
            }
        }

        if (tree.size() > last_tree_len) {
            i++;
            last_tree_len = tree.size();
        }
    }
    Serial.println("created tree");

    Direction dir = Direction::DOWN;

    std::vector<Point> goalPoints = findGoals(
        dir,
        tree,
        path_search_radius,
        tree_width / 2,
        tree_height / 2
    );

    if (goalPoints.empty()) {
        goalPoints.push_back(
            findGoal(dir, tree, tree_width / 2, tree_height / 2)
        );
    }

    std::vector<std::vector<Point>> paths;
    std::vector<double> g_scores;

    for (int i = 0; i < goalPoints.size(); i++) {
      double cost;
      std::vector<Point> path = Astar({0, 0}, goalPoints[i], tree, &cost);
      paths.push_back(path);
      g_scores.push_back(cost);
    }


    int min_index = 0;
    for (int i = 0; i < g_scores.size(); i++) {
        if (g_scores[i] < g_scores[min_index]) {
            min_index = i;
        }
    }
    std::vector<Point> min_path= paths[min_index];
    for(const Point& point: min_path) {
      Serial.print("[ ");
      Serial.print(point.x);
      Serial.print(", ");
      Serial.print(point.y);
      Serial.println(" ]");
    }
    Serial.println(g_scores[min_index]);
}

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

  Serial.println("Starting 6-sensor initialization...");

  // initialize sensors one at a time

  for (int i = 0; i < 6; i++) {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }
  delay(20);

  for (int i = 0; i < 6; i++) {
    digitalWrite(xshutPins[i], HIGH);
    delay(10);

    if (sensors[i].begin() != 0) {
      Serial.print("Failed to begin sensor ");
      Serial.println(i + 1);
    }

    sensors[i].InitSensor(sensorAddresses[i] << 1);
    sensors[i].VL53L4CX_StartMeasurement();
    
    Serial.print("Sensor "); 
    Serial.print(i + 1);
    Serial.print(" ready at address 0x");
    Serial.println(sensorAddresses[i], HEX);
  }

  Serial.println("Setup complete!");

  //Safestruct instantiation 
  SafeStruct<OdoPose> odoStruct;
  SafeStruct<TOF_t> tofStruct;
  SafeStruct<MclPose> mclPoseStruct;
  SafeStruct<Velos> veloStruct; 
  SafeStruct<Path> pathStruct;

  //task class instantiation
  //myPeer = new UDPPeer(odoStruct, tofStruct, mclPoseStruct, pathStruct);
  // tof = new TOF(tofStruct);
  // odo = new Odo(odoStruct);

  
  // xTaskCreate(updTask, "UDP Task", 2048, (void*)myPeer, 1, NULL);
  // xTaskCreate(tofTask, "TOF Task", 2048, (void*)tof, 1, NULL);
  // xTaskCreate(odoTask, "Odo Task", 2048, (void*)odo, 1, NULL);
  //testEncoder(); //Added this for testing
  pinMode(LDO2_ENABLE_PIN, OUTPUT);
  digitalWrite(LDO2_ENABLE_PIN, HIGH);
  //testUDP(myPeer);
  //delay(1000);
  randomSeed(analogRead(0));  // randomness for rrt

  Serial.println("planner initialized");
}

void loop() {
  sensor_measurements.clear();

  int i = 0;
  while(i < 6) {
    VL53L4CX_MultiRangingData_t MultiRangingData;
    uint8_t NewDataReady = 0;

    // Check if the sensor has a new reading
    sensors[i].VL53L4CX_GetMeasurementDataReady(&NewDataReady);

    if (NewDataReady) { 
      sensors[i].VL53L4CX_GetMultiRangingData(&MultiRangingData);
      
      // If at least one object is found
      if (MultiRangingData.NumberOfObjectsFound > 0) {
        // RangeData[0] is typically the closest target
        sensor_measurements.push_back(MultiRangingData.RangeData[0].RangeMilliMeter);
        Serial.println(MultiRangingData.RangeData[0].RangeMilliMeter);
      } else {
        Serial.print("No target\t");
      }
      
      // Clear interrupt to prepare for next measurement
      sensors[i].VL53L4CX_ClearInterruptAndStartMeasurement();
      i++;
    }
  }
  Serial.println(); 
  delay(50); 
  detector.newCalc(sensor_measurements, tree_width, tree_height, vert, horiz);
  for(Line line: vert) {
    Serial.print("[");
    Serial.print("[ ");
    Serial.print(line.start.x);
    Serial.print(", ");
    Serial.print(line.start.y);
    Serial.print(" ]");
    Serial.print("[ ");
    Serial.print(line.start.x);
    Serial.print(", ");
    Serial.print(line.start.y);
    Serial.print(" ]");
    Serial.println("]");
  }
  plannerStep();

  delay(200);  // prevent flooding + give time slice 
}

//TEST FUNCTIONS
void testUDP(UDPPeer* udp) {
  size_t strSize = 26; //keep this under 64
  char data[strSize] = "abcdefghijklmnopqrstuvwxyz"; 

  size_t ct = 0;
  while (ct < 10) {
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