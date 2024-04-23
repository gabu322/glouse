// * Libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Arduino.h>
#include <BleMouse.h>

// *  Glove pin definition
#define A1 T3 // Left click
#define A2 T7 // Back
#define A3 T8 // Foward
#define B1 T9 // Right click
#define B2 T6 // Middle click
#define C1 T4 // Scroll (special)
#define C2 T5 // Config (special)

// Glove| Tpin | GPIO | Mouse
// A1   | T3   | 15   | Left
// A2   | T7   | 13   | Back
// A3   | T8   | 12   | Foward
// B1   | T9   | 14   | Right
// B2   | T6   | 27   | Middle
// C1   | T4   | 33   | Scroll (special)
// C2   | T5   | 32   | Config (special)

// * Variables
MPU6050 mpu; // MPU6050 object
BleMouse bleMouse("Glouse", "gabu", 100);

bool blinkState = false; // State of the LED on pin 13

const int smoothTimer = 200,                   // Time (in ms) to smooth the rotation
    MPUTaskDelay = 25,                         // Time (in ms) to wait between each loop
    smoothCycles = smoothTimer / MPUTaskDelay, // Number of cycles to smooth the rotation
    screenSize[2] = {1366, 768};               // [width, height]     array to store the screen size

// Variable to handle the rotation
float calculatedPR[2] = {0}, // [pitch, roll]       array to store the calculated pitch and roll
    offsetPR[2] = {0},       // [pitch, roll]       array to store the offset pitch and roll
    previousPR[2] = {0};     // [pitch, roll]       array to store the previous pitch and roll

// Task to handle the MPU
TaskHandle_t MPUTaskHandler = NULL;
void MPUTask(void *pvParameters) {

    // Declaration of variables that handle the orientation/rotation
    Quaternion q;         // [w, x, y, z]         quaternion container
    VectorFloat gravity;  // [x, y, z]            gravity vector
    float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    float avgPR[2] = {0}; // [pitch, roll]        array to store the average pitch and roll

    // Declaration of variables that handle, control and store the status data of the MPU
    bool dmpReady = false;                                // set true if DMP init was successful
    uint8_t fifoBuffer[64];                               // FIFO storage buffer
    float rotationSmootherArray[(smoothCycles)][2] = {0}; // [pitch, roll] array to store the last X values of pitch and roll to calculate and smooth the rotation
    int rotationSmootherIndex = 0;                        // Index to store the current value of the rotationSmootherArray

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
                rotationSmootherArray[rotationSmootherIndex][i] = (ypr[i + 1] * 180 / M_PI) + 180;

                for (int j = 0; j < (smoothCycles); j++)
                    avgPR[i] += rotationSmootherArray[j][i];

                avgPR[i] = avgPR[i] / ((smoothCycles) + 1);

                if (touchRead(C2) < 20)
                    offsetPR[i] = avgPR[i];

                calculatedPR[i] = avgPR[i] - offsetPR[i];
            }

            rotationSmootherIndex = (rotationSmootherIndex + 1) % (smoothCycles);
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Creating the task to handle the MPU
    xTaskCreatePinnedToCore(MPUTask, "MPUTask", 20000, NULL, 1, &MPUTaskHandler, 1);

    delay(100);

    // Initialize the BLE Mouse
    bleMouse.begin();

    pinMode(2, OUTPUT);
}

// * Struct to handle the mouse buttons
typedef struct MouseButton {
    int touchPin;
    int mouseButton;
    int touchValue;
    bool pressed;
} MouseButton;

// Delaration of mouse clicks
MouseButton mouseButtons[5] = {
    {T3, MOUSE_LEFT, 100, false},
    {T7, MOUSE_BACK, 100, false},
    {T8, MOUSE_FORWARD, 100, false},
    {T9, MOUSE_RIGHT, 100, false},
    {T6, MOUSE_MIDDLE, 100, false},
};

int minLoopTimer = 0;
float sensitivity = 7.5;
void loop() {

    if (bleMouse.isConnected()) {

        // Don't move the mouse if the user is touching the C2 button
        if (touchRead(C2) > 20) {

            // Code for mouse movement and wheel
            if (touchRead(T4) < 20) {
                bleMouse.move(0, 0, -(calculatedPR[1] - previousPR[1]) * 1.6, -(calculatedPR[0] - previousPR[0]) * 0.9);
            } else {
                bleMouse.move(-(calculatedPR[0] - previousPR[0]) * 1.6 * sensitivity, -(calculatedPR[1] - previousPR[1]) * 0.9 * sensitivity, 0, 0);
            }
        }

        previousPR[0] = calculatedPR[0];
        previousPR[1] = calculatedPR[1];

        // Code for mouse buttons
        for (int i = 0; i < 5; i++) {
            // Read the touch value and the pressed state of the mouse button
            mouseButtons[i].touchValue = touchRead(mouseButtons[i].touchPin);
            mouseButtons[i].pressed = bleMouse.isPressed(mouseButtons[i].mouseButton);

            // Check if the button is pressed or released, and send the corresponding command
            if (mouseButtons[i].touchValue > 20 && mouseButtons[i].pressed) {
                bleMouse.release(mouseButtons[i].mouseButton);
            } else if (mouseButtons[i].touchValue < 20 && !mouseButtons[i].pressed) {
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
