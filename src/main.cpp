#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

bool blinkState = false;

// Variable to handle the rotation
float ypr[3];            // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float avgPR[2] = {0};    // [pitch, roll]        array to store the average pitch and roll
float offsetPR[2] = {0}; // [pitch, roll]       array to store the offset pitch and roll
int smoothTimer = 900;   // Time (in ms) to smooth the rotation
int MPUTaskDelay = 25;   // Time (in ms) to wait between each loop

// Task to handle the MPU
TaskHandle_t MPUTaskHandler = NULL;
void MPUTask(void *pvParameters) {
    vTaskSuspend(NULL);
    int smootherSize = (smoothTimer + 100) / MPUTaskDelay; // Number of values to store to smooth the rotation

    // Declaration of variables that handle the orientation/rotation
    Quaternion q;        // [w, x, y, z]         quaternion container
    VectorFloat gravity; // [x, y, z]            gravity vector

    // Declaration of variables that handle, control and store the status data of the MPU
    bool dmpReady = false;                              // set true if DMP init was successful
    uint8_t fifoBuffer[64];                             // FIFO storage buffer
    float rotationSmootherArray[smootherSize][2] = {0}; // [pitch, roll] array to store the last X values of pitch and roll to calculate and smooth the rotation
    int rotationSmootherIndex = 0;                      // Index to store the current value of the rotationSmootherArray

    Wire.begin();
    Wire.setClock(400000);

    mpu.initialize();
    int devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(1803); // 1803
    mpu.setYGyroOffset(1437); // 1437
    mpu.setZGyroOffset(1755); // 1755
    mpu.setZAccelOffset(22);  // 22

    if (devStatus == 0) {
        mpu.CalibrateAccel(12);
        mpu.CalibrateGyro(12);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        dmpReady = true;
    } else {
        Serial.println("DMP Initialization failed");
    }

    const TickType_t taskDelay = MPUTaskDelay / portTICK_PERIOD_MS;
    TickType_t lastExecutionTime = xTaskGetTickCount();

    int timer = 0;
    for (;;) {
        vTaskDelayUntil(&lastExecutionTime, taskDelay);
        if (!dmpReady) {
            continue;
        }

        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            rotationSmootherArray[rotationSmootherIndex][0] = ypr[1] * 180 / M_PI;
            rotationSmootherArray[rotationSmootherIndex][1] = ypr[2] * 180 / M_PI;

            // Increment the index, wrapping back to 0 if it exceeds smootherSize
            rotationSmootherIndex = (rotationSmootherIndex + 1) % smootherSize;

            // Calculate the average pitch and roll
            for (int i = 0; i < smootherSize; i++) {
                avgPR[0] += rotationSmootherArray[i][0];
                avgPR[1] += rotationSmootherArray[i][1];
            }
            avgPR[0] /= smootherSize;
            avgPR[1] /= smootherSize;

            Serial.print(" pr\t");
            Serial.print(avgPR[0]);
            Serial.print("\t");
            Serial.print(ypr[1] * 180 / M_PI);
            Serial.print("\t");
            Serial.print(avgPR[1]);
            Serial.print("\t");
            Serial.println(ypr[2] * 180 / M_PI);
        }
    }
}

// BLE Mouse Task
TaskHandle_t BLEMouseTaskHandler = NULL;
void BLEMouseTask(void *pvParameters) {
    vTaskSuspend(NULL);
    for (;;) {
        vTaskDelay(1000);
    }
}


void setup() {
    Serial.begin(115200);

    delay(1000);
    xTaskCreatePinnedToCore(MPUTask, "MPUTask", 20000, NULL, 1, &MPUTaskHandler, 1);
    xTaskCreatePinnedToCore(BLEMouseTask, "BLEMouseTask", 10000, NULL, 1, &BLEMouseTaskHandler, 1);
    pinMode(2, INPUT_PULLDOWN);
}

int timer = 0;
int taskCreated = 0;
void loop() {
    if (digitalRead(2) == HIGH) {
        if (taskCreated) {
            vTaskSuspend(MPUTaskHandler);
        } else {
            vTaskResume(MPUTaskHandler);
        }
        Serial.println(eTaskGetState(MPUTaskHandler));
        taskCreated = !taskCreated;
        delay(500);
    }
    // if the task isnt running, is suspended or deleted
    if (eTaskGetState(MPUTaskHandler) == eDeleted || eTaskGetState(MPUTaskHandler) == eSuspended) {
        Serial.println("MPU Task not runnung");
    }
    delay(100);
}
