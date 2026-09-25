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
#include "WallFollower.h"

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

    drive->readSensors();
    // drive->setSpeeds(clamp(correction - dist_control, -0.7, 0.7), clamp(-correction - dist_control, -0.7, 0.7));
    drive->setSpeeds(leftSpeed, rightSpeed);

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
  randomSeed(micros());

}

constexpr float LEARNING_RATE = 0.002f;
constexpr float TOLERANCE = 0.0001f;
constexpr unsigned MAX_STEPS = 20000;
constexpr unsigned VARIABLE_COUNT = 200;
constexpr unsigned PROGRESS_INTERVAL = 100;

static float quadraticVariables[VARIABLE_COUNT];
static float quadraticTarget[VARIABLE_COUNT];
static float quadraticError[VARIABLE_COUNT];
static float quadraticBasisProduct[VARIABLE_COUNT];
static float quadraticGradient[VARIABLE_COUNT];
static float quadraticDirection[VARIABLE_COUNT];

float quadraticBasis(unsigned row, unsigned column) {
  uint32_t value = 2166136261u;
  value ^= row + 0x9e3779b9u;
  value *= 16777619u;
  value ^= column + 0x85ebca6bu;
  value *= 16777619u;
  return (static_cast<float>(value % 201u) - 100.0f) / 100.0f;
}

float quadraticLoss(const float variables[], const float target[]) {
  float value = 0.0f;
  for (unsigned row = 0; row < VARIABLE_COUNT; ++row) {
    quadraticError[row] = variables[row] - target[row];
  }

  for (unsigned basis = 0; basis < VARIABLE_COUNT; ++basis) {
    quadraticBasisProduct[basis] = 0.0f;
    for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
      quadraticBasisProduct[basis] +=
          quadraticBasis(basis, variable) * quadraticError[variable];
    }
    value += quadraticBasisProduct[basis] * quadraticBasisProduct[basis];
  }
  for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
    value += 0.5f * quadraticError[variable] * quadraticError[variable];
  }
  return value;
}

void quadraticGradientAt(const float variables[], const float target[],
                         float gradient[]) {
  for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
    quadraticError[variable] = variables[variable] - target[variable];
    gradient[variable] = quadraticError[variable];
  }
  for (unsigned basis = 0; basis < VARIABLE_COUNT; ++basis) {
    quadraticBasisProduct[basis] = 0.0f;
    for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
      quadraticBasisProduct[basis] +=
          quadraticBasis(basis, variable) * quadraticError[variable];
    }
    for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
      gradient[variable] +=
          2.0f * quadraticBasis(basis, variable) *
          quadraticBasisProduct[basis];
    }
  }
}

void buildQuadraticProblem(float target[]) {
  for (unsigned row = 0; row < VARIABLE_COUNT; ++row) {
    target[row] = random(-500, 501) / 100.0f;
  }
}


void loop() {  
  for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
    quadraticVariables[variable] = 0.0f;
  }
  buildQuadraticProblem(quadraticTarget);

  float previousLoss = quadraticLoss(quadraticVariables, quadraticTarget);
  bool converged = false;

  Serial.printf("\nMinimize %u-variable random quadratic; target:", VARIABLE_COUNT);
  for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
    Serial.printf(" %.3f", quadraticTarget[variable]);
  }
  Serial.println();
  Serial.printf("step=0 loss=%.8f\n", previousLoss);

  const unsigned long convergenceStart = micros();
  for (unsigned step = 1; step <= MAX_STEPS; ++step) {
    const unsigned long stepStart = micros();
    quadraticGradientAt(quadraticVariables, quadraticTarget,
                        quadraticDirection);

    float alpha0 = 0.0f;
    float alpha1 = LEARNING_RATE;
    float derivative0 = 0.0f;
    float derivative1 = 0.0f;
    for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
      derivative0 -= quadraticDirection[variable] * quadraticDirection[variable];
      quadraticVariables[variable] -= alpha1 * quadraticDirection[variable];
    }
    quadraticGradientAt(quadraticVariables, quadraticTarget, quadraticGradient);
    for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
      derivative1 -= quadraticGradient[variable] * quadraticDirection[variable];
      quadraticVariables[variable] += alpha1 * quadraticDirection[variable];
    }

    float alpha = alpha1;
    const float denominator = derivative1 - derivative0;
    if (fabsf(denominator) > 0.00000001f) {
      // Secant update: alpha = alpha1 - f(alpha1) * (alpha1-alpha0)/(f(alpha1)-f(alpha0)).
      alpha = alpha1 - derivative1 * (alpha1 - alpha0) / denominator;
    }
    if (!isfinite(alpha) || alpha <= 0.0f) {
      alpha = alpha1;
    }
    for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
      quadraticVariables[variable] -= alpha * quadraticDirection[variable];
    }

    const float currentLoss = quadraticLoss(quadraticVariables, quadraticTarget);

    if (!isfinite(currentLoss) || currentLoss > previousLoss) {
      for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
        quadraticVariables[variable] += alpha * quadraticDirection[variable];
      }
      Serial.printf("step=%u rejected; loss=%.8f; alpha=%.8g; elapsed=%.3f s\n",
                    step, currentLoss, alpha,
                    (micros() - convergenceStart) / 1000000.0f);
      continue;
    }
    if (currentLoss < TOLERANCE) {
      Serial.printf("PASS: converged at step %u; loss < %.6f.\n",
                    step, TOLERANCE);
      converged = true;
      break;
    }
    previousLoss = currentLoss;
    if (step % PROGRESS_INTERVAL == 0) {
      Serial.printf("step=%u; loss=%.8f; alpha=%.8g; step time=%.3f ms; elapsed=%.3f s\n",
            step, currentLoss, alpha,
                    (micros() - stepStart) / 1000.0f,
                    (micros() - convergenceStart) / 1000000.0f);
    }
     }

  const unsigned long convergenceTime = micros() - convergenceStart;
  if (converged) {
    Serial.printf("Convergence time: %lu us (%.3f ms, %.6f s)\n",
                  convergenceTime, convergenceTime / 1000.0f,
                  convergenceTime / 1000000.0f);
    Serial.printf("Minimum loss: %.8f; solution:",
                  quadraticLoss(quadraticVariables, quadraticTarget));
    for (unsigned variable = 0; variable < VARIABLE_COUNT; ++variable) {
      Serial.printf(" %.3f", quadraticVariables[variable]);
    }
    Serial.println();
  } else {
    Serial.println("Stopped without convergence.");
  }
  Serial.println("Restarting demo in 5 seconds.");
  delay(5000);

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