#include <Arduino.h>
#include <atomic>
#include "dataTypes.h"
#include "UDPPeer.h"
//#include "Wire.h"
#include "Odo.h"
//#include "TOF.h"
#include "Encoder_test.h"
#include "Motor.h"
#include "Encoder.h"
#include <vl53l4cx_class.h>
#include "Drivetrain.h"
#include "pid.h"
#include <random>
#include <cmath>

void testUDP(UDPPeer* udp);

const int xshutPins[6] = {38, 39, 40, 41, 42, 12};
// const int LDO2_ENABLE_PIN = 17;

VL53L4CX sensors[6] = {
  VL53L4CX(&Wire, xshutPins[0]),
  VL53L4CX(&Wire, xshutPins[1]),
  VL53L4CX(&Wire, xshutPins[2]),
  VL53L4CX(&Wire, xshutPins[3]),
  VL53L4CX(&Wire, xshutPins[4]),
  VL53L4CX(&Wire, xshutPins[5])
};

uint8_t sensorAddresses[6] = {0x30, 0x31, 0x32, 0x33, 0x34, 0x36};

//Safestruct instantiation 
SafeStruct<OdoPose> odoStruct;
SafeStruct<TOF_t> tofStruct;
SafeStruct<MclPose> mclPoseStruct;
SafeStruct<Velos> veloStruct; 
SafeStruct<Path> pathStruct;

//class task instantiations 
UDPPeer *myPeer;
// TOF *tof;
Odo *odo;

// Motor *motor;
Encoder *encoder;
// VL53L4CX *sensor1;

Drivetrain* drive;
// QwiicOTOS myOTOS;

std::random_device rd; 

std::mt19937 gen(rd()); 

std::uniform_real_distribution<double> dis(0.0, 1.0);
unsigned long latestTime;
int count = 0;

double alpha = 0.25;

double leftSpeed, leftSpeedTarget, rightSpeed, rightSpeedTarget;

QueueHandle_t sendQueue;

SemaphoreHandle_t sensorDoneSem;

PID headingPID(0.45, 0.0, 0.0, -10, 10, true); // right+, left- for positive rotation
PID distancePID(0.05, 0.0, 0.0, -10, 10, false);

double speed = 0.3;
float dist = 0.0;
float front = 0.0;
float back = 0.0;
bool rotate = false;

double correction = 0.0;
double dist_control = 0.0;

float x;
float y;

// void updTask(void* param)
// {
//   UDPPeer *myPeer = (UDPPeer*) param;

//   while(1) {
//     myPeer->Update();
//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

float clamp(float x, float min, float max) {
  if (x < min) {
    return min;
  }

  if (x > max) {
    return max;
  }

  return x;
}



void sensorTask(void *pvParameters) {
  while(1) {
    auto data = tofStruct.get(); // snapshot

    for (int i = 0; i < 6; i++) {
      VL53L4CX_MultiRangingData_t MultiRangingData;
      uint8_t NewDataReady = 0;

      sensors[i].VL53L4CX_GetMeasurementDataReady(&NewDataReady);
      if (NewDataReady) {
        sensors[i].VL53L4CX_GetMultiRangingData(&MultiRangingData);
        // Serial.print("S");
        // Serial.print(i + 1);
        // Serial.print(": ");
        if (MultiRangingData.NumberOfObjectsFound > 0) {
          // Serial.print(MultiRangingData.RangeData[0].RangeMilliMeter);
          // Serial.print("mm\t");
          data.distances[(i + 2) % 6] = MultiRangingData.RangeData[0].RangeMilliMeter;
          data.stds[(i + 2) % 6] = sqrt(MultiRangingData.RangeData[0].SigmaMilliMeter);
        } else {
          Serial.print("No target\t");
        }
      }
      sensors[i].VL53L4CX_ClearInterruptAndStartMeasurement();
    }
    Serial.println();

    // vTaskDelay(pdMS_TO_TICKS(25));

    //drive->readSensors();
    drive->setSpeeds(clamp(correction - dist_control, -0.7, 0.7), clamp(-correction - dist_control, -0.7, 0.7));

    tofStruct.set(data); // single atomic write after all sensors are polled

    auto data2 = odoStruct.get();
    data2.pos.x = drive->otosPoseMeasurement.x;
    data2.pos.y = drive->otosPoseMeasurement.y;
    data2.pos.h = drive->otosPoseMeasurement.h;

    data2.vel.x = drive->otosVelocityMeasurement.x;
    data2.vel.y = drive->otosVelocityMeasurement.y;
    data2.vel.h = drive->otosVelocityMeasurement.h;

    odoStruct.set(data2);

    // Serial.println("Read");
    // Serial.print("data test: ");
    // Serial.println(tofStruct.get().distances[0]);
    // Serial.println(tofStruct.get().stds[0]);

    xSemaphoreGive(sensorDoneSem);

    vTaskDelay(pdMS_TO_TICKS(50));
  }

  headingPID.reset();
}

void updTask(void* param) {
  UDPPeer *myPeer = (UDPPeer*) param;
  char outBuf[64];

  while(1) {
    xSemaphoreTake(sensorDoneSem, portMAX_DELAY);

    if (myPeer->Update() == 'a') {
      drive->zero();
    }
    // Drain send queue
    // while (xQueueReceive(sendQueue, outBuf, 0) == pdTRUE) {
      // myPeer->sendString(outBuf, strlen(outBuf));
      // myPeer->Update();
      // Serial.println(strlen(outBuf));
    // }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// void odoTask(void* param)
// {
//   Odo *odo = (Odo*) param;

//   while(1) {
//     odo->update();
//     vTaskDelay(50/ portTICK_PERIOD_MS);
//   }
// }

// void tofTask(void* param)
// {
//   TOF *tof = (TOF*) param;

//   uint8_t sensorID;
//   while(true) {
//     if (xQueueReceive(tofQueue, &sensorID, portMAX_DELAY))
//     {
//       tof->update(sensorID);
//     }
//   }
// }

void odometryTask(void* param)
{
  // drive->otosPoseMeasurement.x, drive->otosPoseMeasurement.y, drive->otosPoseMeasurement.h,
  //           drive->otosVelocityMeasurement.x,  drive->otosVelocityMeasurement.y,  drive->otosVelocityMeasurement.h,
  //           drive->encoderMeasurements.leftEncoderX, drive->encoderMeasurements.rightEncoderX,
  while(true) {
    auto data = odoStruct.get();
    data.pos.x = drive->otosPoseMeasurement.x;
    data.pos.y = drive->otosPoseMeasurement.y;
    data.pos.h = drive->otosPoseMeasurement.h;

    data.vel.x = drive->otosVelocityMeasurement.x;
    data.vel.y = drive->otosVelocityMeasurement.y;
    data.vel.h = drive->otosVelocityMeasurement.h;

    odoStruct.set(data);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

#define XSHUT_PIN 38
void setup() {
  // Setup Serial
  Serial.begin(115200);

  // Setup Wire
  Wire.begin(8, 9);
  Wire.setClock(100000);

  //task class instantiation
  pinMode(17, OUTPUT);
  digitalWrite(17, HIGH);

  odoStruct.init();
  tofStruct.init();
  mclPoseStruct.init();
  veloStruct.init(); 
  pathStruct.init();

  // pinMode(XSHUT_PIN, OUTPUT);
  // digitalWrite(XSHUT_PIN, LOW);
  // delay(10);
  // digitalWrite(XSHUT_PIN, HIGH); // Wake up the sensor
  // delay(10);
    
  // analogWriteFrequency(100000);
  // analogWriteResolution(15);
  myPeer = new UDPPeer(odoStruct, tofStruct, mclPoseStruct, pathStruct);

  // In setup():
  sendQueue = xQueueCreate(10, 64); // 10 messages, 128 bytes each

  sensorDoneSem = xSemaphoreCreateBinary();

  // Give updTask more stack and have it drain the queue:
// 

  // if (motor == nullptr) {
    // motor = new Motor(37, 35);
    // motor = new Motor(15, 16);
  // }
  // encoder = new Encoder(36, 34);
  // sensor1 = new VL53L4CX(&Wire, 38);

  // sensor1->begin();
  // sensor1->VL53L4CX_Off();
  // sensor1->InitSensor(0x30); // Initialize and set I2C address to 0x30
  // delay(10);

  // sensor1->VL53L4CX_On();
  // delay(10);
  // sensor1->VL53L4CX_StartMeasurement();
  // Serial.println("Sensor 1 Online at 0x30");


  // tof = new TOF(tofStruct);
  // odo = new Odo(odoStruct);
  // xTaskCreate(updTask, "UDP Task", 2048, (void*)myPeer, 1, NULL);
  // xTaskCreate(tofTask, "TOF Task", 2048, (void*)tof, 1, NULL);
  // xTaskCreate(odoTask, "Odo Task", 2048, (void*)odo, 1, NULL);
  // testEncoder(); //Added this for testing
  // testUDP(myPeer);

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

  // Scan I2C bus to see what devices are present
  Serial.println("Scanning I2C bus...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", address);
    }
  }
  
  Wire.beginTransmission(0x6A);
  byte err6A = Wire.endTransmission();
  Serial.print("IMU at 0x6A: "); Serial.println(err6A);
  
  Wire.beginTransmission(0x6B);
  byte err6B = Wire.endTransmission();
  Serial.print("IMU at 0x6B: "); Serial.println(err6B);

  // Allow I2C bus to stabilize after TOF sensor initialization
  delay(100);
  
  drive = new Drivetrain(37, 35, 15, 16, 36, 34, 13, 14);

  xTaskCreate(sensorTask, "Sensor Task", 8192, NULL, 1, NULL);

  // xTaskCreate(odometryTask, "Odometry Task", 8192, NULL, 1, NULL);
  xTaskCreate(updTask, "UDP Task", 8192, (void*)myPeer, 1, NULL);
}


void loop() {  
  // Serial.println("Hello, ESP8266!");
  delay(100);
  // motor->setThrottle(0.0);

  // drive->setSpeeds(0.0, 0.0);

  
  // Serial.println(count);
  // Serial.println("left");
  // Serial.println(drive->encoderMeasurements.leftEncoderX);
  // Serial.println("right");
  // Serial.println(drive->encoderMeasurements.rightEncoderX);

  // size_t strSize = 4; //keep this under 64
  // char data[strSize] = "abcdefghijklmnopqrstuvwxyz"; 
  // myPeer->sendGeneric(&drive->otosPoseMeasurement, sizeof(drive->otosPoseMeasurement));

  // char buffer[128];

  // latestTime = millis();

  // snprintf(buffer, sizeof(buffer),
  //           "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
  //           latestTime,
  //           drive->otosPoseMeasurement.x, drive->otosPoseMeasurement.y, drive->otosPoseMeasurement.h,
  //           drive->otosVelocityMeasurement.x,  drive->otosVelocityMeasurement.y,  drive->otosVelocityMeasurement.h,
  //           drive->encoderMeasurements.leftEncoderX, drive->encoderMeasurements.rightEncoderX,
  //           leftSpeed, rightSpeed
  //         );
  // Serial.println(buffer);
  // if (myPeer != nullptr) {
  //   myPeer->sendString(buffer, strlen(buffer));
  // } else {
  //   Serial.println("FAIL");
  // }

  // if (sendQueue != nullptr) {
  //   // xQueueSend(sendQueue, buffer, 0); // non-blocking, drops if full
  //   xQueueSend(sendQueue, "apple", 0); // non-blocking, drops if full
  // }

  // count++;

  // if (count > 5) {
  //   // Serial.println("reset");
  //   leftSpeedTarget = dis(gen) * 2 - 1;
  //   // Serial.println(random_val);
  //   rightSpeedTarget = dis(gen) * 2 - 1;
  //   // Serial.println(random_val);

  //   // Serial.println(leftSpeedTarget);
  //   // Serial.println(rightSpeedTarget);

  //   // drive->setSpeeds(leftSpeed, rightSpeed);

  //   count = 0;
  // } else {
  //   count++;
  // }

  // leftSpeed = (1.0 - alpha) * leftSpeed + alpha * leftSpeedTarget;
  // rightSpeed = (1.0 - alpha) * rightSpeed + alpha * rightSpeedTarget;

  // if (tofStruct.get().distances[1] != 0 && tofStruct.get().distances[5] != 0) {
  //   leftSpeed = 0.6 * tofStruct.get().distances[1] / (tofStruct.get().distances[1] + tofStruct.get().distances[5]);
  //   rightSpeed = 0.6 * tofStruct.get().distances[5] / (tofStruct.get().distances[1] + tofStruct.get().distances[5]);
  // } else {
  //   leftSpeed = 0.0;
  //   rightSpeed = 0.0;
  // }

  // Serial.println();
  // dist = tofStruct.get().distances[0];

  // if (rotate) {
  //   if (dist > 200) {
  //     rotate = false;
  //     leftSpeed = speed;
  //     rightSpeed = speed;
  //   }
  // } else {
  //   if (dist < 150 && dist > 0) {
  //     rotate = true;
  //     if (rand() < 0.5) {
  //       leftSpeed = -speed;
  //       rightSpeed = speed;
  //     } else {
  //       leftSpeed = speed;
  //       rightSpeed = -speed;
  //     }
  //   }
  // }

  front = clamp(tofStruct.get().distances[0], 0.0, 500.0);
  back = clamp(tofStruct.get().distances[3], 0.0, 500.0);

  // Serial.println(front);
  // Serial.println(back);
  // Serial.println(0.002 * (front - back));
  // Serial.println(leftSpeed);
  // Serial.println(rightSpeed);
  // Serial.println("-----------------");

  // 
  // drive->setSpeeds(leftSpeed, rightSpeed);

  // if (count < 50) {
  //   drive->setSpeeds((double) 0.002 * (front / 2 - back), (double) 0.002 * (front / 2 - back));
  // } else {
  //   drive->setSpeeds(-0.7, 0.7);
  // }

  // Serial.println(count);

  // count++;

  // if (count > 60) {
  //   count = 0;
  // }

  x = (1 - alpha) * x + alpha * (mclPoseStruct.get().x + drive->otosPoseMeasurement.x);
  y = (1 - alpha) * y + alpha * (mclPoseStruct.get().y + drive->otosPoseMeasurement.y);

  // dist = sqrt((x - 730) * (x - 730) + (y - 1080) * (y - 1080));

  Serial.print("heading: ");
  Serial.println(drive->otosPoseMeasurement.h);
  Serial.println(String("atan2:") + atan2(2227 - y, 127 - x));

  correction = clamp(headingPID.update(atan2(2227 - y, 127 - x), drive->otosPoseMeasurement.h, 0.1), -0.7, 0.7);
  if (correction >=-.1 && correction <= .1){
    correction = 0.0;
  }
  // dist_control = clamp(distancePID.update(0, dist, 0.1), -0.7, 0.7);
  
  // // Serial.print(", correction: ");
  Serial.print(correction);
  // // Serial.print("\n");
  count++;

  // if (count > 50) {
  //   drive->setSpeeds(clamp(correction - dist_control, -0.7, 0.7), clamp(-correction - dist_control, -0.7, 0.7));
  // }
  // drive->setSpeeds(0.0, 0.0);

  // x = mclPoseStruct.get().x;
  // y = mclPoseStruct.get().y;

  // Serial.print(x);
  // Serial.print(", ");
  // Serial.print(y);
  // Serial.print(", ");
  // Serial.print(clamp(correction, -0.7, 0.7));
  // Serial.print(", ");
  // Serial.print(clamp(dist_control, -0.7, 0.7));

  // Serial.print(", ");
  // Serial.print(drive->otosPoseMeasurement.h);

  // Serial.print(", ");
  // Serial.print(atan2(1080 - y, 730 - x) * 180.0 / 3.14159265358979323846);

  // Serial.print("\n");

  // if (currentMillis % 1000 == )
  // double random_val = dis(gen);
  // std::cout << "Random double: " << random_val << std::endl;
  
  // myPeer->sendString(buffer, strlen(buffer));

  // Serial.print(digitalRead(14));
  // Serial.print(", ");
  // Serial.print(digitalRead(13));
  // Serial.print(", ");
  // Serial.println(digitalRead(12));

  // byte error;
  // for (byte address = 1; address < 127; address++) {
  //   Wire.beginTransmission(address);
  //   error = Wire.endTransmission();

  //   if (error == 0) {
  //     Serial.printf("I2C device found at address 0x%02X\n", address);
  //     // nDevices++;
  //   } else {
  //     Serial.println("error: ");
  //     Serial.println(address);
  //     Serial.println(error);
  //   }
  // }
  // uint8_t NewDataReady = 0;
  // VL53L4CX_MultiRangingData_t results1;

  // Read Sensor 1
  // sensor1->VL53L4CX_StartMeasurement();
  // sensor1->VL53L4CX_GetMeasurementDataReady(&NewDataReady);
  // if (NewDataReady) {
  //   sensor1->VL53L4CX_GetMultiRangingData(&results1);
  //   if (results1.NumberOfObjectsFound > 0) {
  //     Serial.print("S1 Dist: ");
  //     Serial.print(results1.RangeData[0].RangeMilliMeter);
  //     Serial.print("mm | \n");
  //   }
  //   sensor1->VL53L4CX_ClearInterruptAndStartMeasurement();
  // }
}



//TEST FUNCTIONS
void testUDP(UDPPeer* udp) {
  size_t strSize = 26; //keep this under 64
  char data[strSize] = "abcdefghijklmnopqrstuvwxyz"; 

  size_t ct = 0;
  while (1) {
    udp->sendString(data, strSize);
    udp->receiveData();
    Serial.println(data);
    delay(1000);
    ct++;

    char end = data[strSize-1];
    for (uint8_t i = strSize; i > 0; i--) {
      data[i] = data[i-1];
    }
    data[0] = end;
  }
}

// void setup() {
//   Serial.begin(115200);
//   Wire.begin();

//   pinMode(LDO2_ENABLE_PIN, OUTPUT);
//   digitalWrite(LDO2_ENABLE_PIN, HIGH);

//   Serial.println("Starting 6-sensor initialization...");

//   // initialize sensors one at a time

//   for (int i = 0; i < 6; i++) {
//     pinMode(xshutPins[i], OUTPUT);
//     digitalWrite(xshutPins[i], LOW);
//   }
//   delay(20);

//   for (int i = 0; i < 6; i++) {
//     digitalWrite(xshutPins[i], HIGH);
//     delay(10);

//     if (sensors[i].begin() != 0) {
//       Serial.print("Failed to begin sensor ");
//       Serial.println(i + 1);
//     }

//     sensors[i].InitSensor(sensorAddresses[i] << 1);
//     sensors[i].VL53L4CX_StartMeasurement();
    
//     Serial.print("Sensor "); 
//     Serial.print(i + 1);
//     Serial.print(" ready at address 0x");
//     Serial.println(sensorAddresses[i], HEX);
//   }

//   Serial.println("Setup complete!");

//   xTaskCreate(sensorTask, "Sensor Task", 2048, NULL, 1, NULL);
// }