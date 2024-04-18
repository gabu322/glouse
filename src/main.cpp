#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Arduino.h>
#include <BleMouse.h>

MPU6050 mpu;

bool blinkState = false;

// Variable to handle the rotation
float offsetPR[2] = {0},                                       // [pitch, roll]       array to store the offset pitch and roll
    calculatedRotations[2] = {0},                              // [pitch, roll]       array to store the calculated pitch and roll
    maxRotation[2][2] = {{-60.0, 60.0},                            // [[minPitch, maxPitch],
                         {-45.0, 75.0}};                           // [minRoll, maxRoll]] array to store the minimun and maximum pitch and roll degrees
int smoothTimer = 200,                                         // Time (in ms) to smooth the rotation
    MPUTaskDelay = 25,                                         // Time (in ms) to wait between each loop
    screenSize[2] = {1920, 1080},                              // [width, height]     array to store the screen size
    mousePosition[2] = {screenSize[0] / 2, screenSize[1] / 2}; // [x, y]              array to store the mouse position

// Task to handle the MPU
TaskHandle_t MPUTaskHandler = NULL;
void MPUTask(void *pvParameters) {

    // Declaration of variables that handle the orientation/rotation
    Quaternion q;         // [w, x, y, z]         quaternion container
    VectorFloat gravity;  // [x, y, z]            gravity vector
    float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float avgPR[2] = {0}; // [pitch, roll]        array to store the average pitch and roll

    // Declaration of variables that handle, control and store the status data of the MPU
    bool dmpReady = false;                                              // set true if DMP init was successful
    uint8_t fifoBuffer[64];                                             // FIFO storage buffer
    float rotationSmootherArray[(smoothTimer / MPUTaskDelay)][2] = {0}; // [pitch, roll] array to store the last X values of pitch and roll to calculate and smooth the rotation
    int rotationSmootherIndex = 0;                                      // Index to store the current value of the rotationSmootherArray

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
        digitalWrite(2, HIGH);
    } else {
        vTaskDelete(NULL);
    }

    const TickType_t taskDelay = MPUTaskDelay / portTICK_PERIOD_MS;
    TickType_t lastExecutionTime = xTaskGetTickCount();
    int timer = 0;
    while (dmpReady) {
        vTaskDelayUntil(&lastExecutionTime, taskDelay);

        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // do ALL the code below in only one for
            for (int i = 0; i <= 1; i++) {
                rotationSmootherArray[rotationSmootherIndex][i] = ypr[i + 1] * 180 / M_PI;

                for (int j = 0; j < (smoothTimer / MPUTaskDelay); j++) {
                    avgPR[i] += rotationSmootherArray[j][i];
                }
                avgPR[i] = (avgPR[i] / (smoothTimer / MPUTaskDelay)) - offsetPR[i];

                // Limit the rotation to the maxRotation values
                if (avgPR[i] < maxRotation[i][0])
                    avgPR[i] = maxRotation[i][0];
                if (avgPR[i] > maxRotation[i][1])
                    avgPR[i] = maxRotation[i][1];

                calculatedRotations[i] = map(avgPR[i] * 100, maxRotation[i][0] * 100, maxRotation[i][1] * 100, screenSize[i], 0);
            }

            rotationSmootherIndex = (rotationSmootherIndex + 1) % (smoothTimer / MPUTaskDelay);
        }
    }
}

BleMouse bleMouse("Glouse", "gabu", 100);

void setup() {
    Serial.begin(115200);

    delay(500);
    xTaskCreatePinnedToCore(MPUTask, "MPUTask", 20000, NULL, 1, &MPUTaskHandler, 1);
    delay(500);
    bleMouse.begin();

    pinMode(2, OUTPUT);
}

/* Readable touches:
Tpin | GPIO | Glove
T3   | 15   |  A1
T4   | 13   |  A2
T5   | 12   |  A3
T6   | 14   |  B1
T7   | 27   |  B2
T8   | 33   |  C1
T9   | 32   |  C2
*/

typedef struct MouseButton {
    int touchPin;
    int mouseButton;
    int touchValue;
    int touchValueTreshold;
    bool pressed;
} MouseButton;

// Delaration of mouse clicks
MouseButton mouseButtons[3] = {
    {T3, MOUSE_LEFT, 100, 20, false},
    {T4, MOUSE_RIGHT, 100, 20, false},
    {T5, MOUSE_MIDDLE, 100, 20, false},
};

int minLoopTimer = 0;
bool firstRun = true;

void loop() {

    if (bleMouse.isConnected()) {
        // Set the initial position of the mouse to the center of the screen
        if (firstRun) {
            Serial.println("First run");
            bleMouse.move(-screenSize[0], -screenSize[1], 0);
            Serial.print(-screenSize[0]);
            Serial.print(" ");
            Serial.print(-screenSize[1]);
            Serial.print(" ");
            Serial.println("Moved mouse to corner");
            bleMouse.move(mousePosition[0], mousePosition[1], 0);
            Serial.print(mousePosition[0]);
            Serial.print(" ");
            Serial.print(mousePosition[1]);
            Serial.print(" ");
            Serial.println("Moved mouse to center");
            firstRun = false;
            delay(1000);
        } else {
            Serial.print("Calculated rotations: ");
            Serial.print(calculatedRotations[0]);
            Serial.print("\t");
            Serial.print(calculatedRotations[1]);
            Serial.print("\t");

            Serial.print("Mouse position: ");
            Serial.print(mousePosition[0]);
            Serial.print("\t");
            Serial.print(mousePosition[1]);
            Serial.print("\t");

            Serial.print("Difference to move: ");
            Serial.print(calculatedRotations[0] - mousePosition[0]);
            Serial.print("\t");
            Serial.print(calculatedRotations[1] - mousePosition[1]);
            Serial.print("\t");
            Serial.println();

            // Code for mouse movement
            bleMouse.move(calculatedRotations[0] - mousePosition[0], calculatedRotations[1] - mousePosition[1], 0);
            mousePosition[0] = calculatedRotations[0];
            mousePosition[1] = calculatedRotations[1];

            // Code for mouse buttons
            for (int i = 0; i < 3; i++) {
                mouseButtons[i].touchValue = touchRead(mouseButtons[i].touchPin);
                mouseButtons[i].pressed = bleMouse.isPressed(mouseButtons[i].mouseButton);
                if (mouseButtons[i].touchValue > mouseButtons[i].touchValueTreshold && mouseButtons[i].pressed) {
                    bleMouse.release(mouseButtons[i].mouseButton);
                } else if (mouseButtons[i].touchValue < mouseButtons[i].touchValueTreshold && !mouseButtons[i].pressed) {
                    bleMouse.press(mouseButtons[i].mouseButton);
                }
            }
        }
    }

    // Ensure the loop runs at a minimum rate
    if (millis() - minLoopTimer < 20) {
        delay(20 - (millis() - minLoopTimer));
    }
    minLoopTimer = millis();
}
