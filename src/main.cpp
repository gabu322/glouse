#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Arduino.h>
#include <BleMouse.h>

MPU6050 mpu;

bool blinkState = false;

// Variable to handle the rotation
float offsetPR[2] = {0};                               // [pitch, roll]       array to store the offset pitch and roll
float calculatedRotations[2] = {0};                    // [pitch, roll]       array to store the calculated pitch and roll
int smoothTimer = 100;                                 // Time (in ms) to smooth the rotation
int MPUTaskDelay = 25;                                 // Time (in ms) to wait between each loop
int smootherSize = (smoothTimer + 100) / MPUTaskDelay; // Number of values to store to smooth the rotation

// Task to handle the MPU
TaskHandle_t MPUTaskHandler = NULL;
void MPUTask(void *pvParameters) {

    // Declaration of variables that handle the orientation/rotation
    Quaternion q;         // [w, x, y, z]         quaternion container
    VectorFloat gravity;  // [x, y, z]            gravity vector
    float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float avgPR[2] = {0}; // [pitch, roll]        array to store the average pitch and roll

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
        digitalWrite(2, HIGH);
    } else {
        Serial.println("DMP Initialization failed");
    }

    const TickType_t taskDelay = MPUTaskDelay / portTICK_PERIOD_MS;
    TickType_t lastExecutionTime = xTaskGetTickCount();
    int timer = 0;
    for (;;) {
        vTaskDelayUntil(&lastExecutionTime, taskDelay);

        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // do ALL the code below in only one for
            for (int i = 0; i <= 1; i++) {
                rotationSmootherArray[rotationSmootherIndex][i] = ypr[i + 1] * 180 / M_PI;

                for (int j = 0; j < smootherSize; j++) {
                    avgPR[i] += rotationSmootherArray[j][i];
                }
                avgPR[i] = (avgPR[i] / smootherSize) - offsetPR[i];

                if (avgPR[i] > 5 || avgPR[i] < -5) {
                    calculatedRotations[i] = map(avgPR[i], -90, 90, 50, -50);
                    calculatedRotations[i] = (0.024 * calculatedRotations[i] * abs(calculatedRotations[i])) - 0.2 * abs(calculatedRotations[i]);
                } else {
                    calculatedRotations[i] = 0;
                }
            }
            rotationSmootherIndex = (rotationSmootherIndex + 1) % smootherSize;
        }
    }
}

// * BLE Mouse task

// Fill touch data

BleMouse bleMouse("Glouse", "gabu", 100);

void setup() {
    Serial.begin(115200);

    delay(200);
    xTaskCreatePinnedToCore(MPUTask, "MPUTask", 20000, NULL, 1, &MPUTaskHandler, 1);

    delay(200);
    bleMouse.begin();

    pinMode(2, OUTPUT);
}

// Readable touches:
// T3: pin 15
// T4: pin 13
// T5: pin 12
// T6: pin 14
// T7: pin 27
// T8: pin 33
// T9: pin 32
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
void loop() {

    if (bleMouse.isConnected()) {
        // Rotate the mouse, and the scroll if button X is pressed
        bleMouse.move(calculatedRotations[0], calculatedRotations[1], 0);

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

    // Ensure the loop runs at a minimum rate
    if (millis() - minLoopTimer < 20) {
        delay(20 - (millis() - minLoopTimer));
    }
    minLoopTimer = millis();
}
