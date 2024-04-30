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

// Objects to handle the MPU and Mouse
MPU6050 mpu; // MPU6050 object
BleMouse bleMouse("Glouse", "gabu", 100);

bool blinkState = false; // State of the LED on pin 13

const int
    LoopTimer = 20,                           // Time (in ms) to wait between each loop in the main loop
    MPUTaskDelay = 25,                        // Time (in ms) to wait between each loop in the MPU task
    smoothTimer = 200,                        // Time (in ms) to smooth the rotation
    smoothCycles = smoothTimer / MPUTaskDelay // Number of cycles to smooth the rotation
    ;

int minLoopTimer = 0; // Timer to ensure the loop runs at a minimum rate

float
    pointerSensitivity = 0.5, // Sensitivity of the pointer
    scrollSensitivity = 0.05, // Sensitivity of the scroll
    currentPR[2] = {0},    // [pitch, roll]       array to store the calculated pitch and roll
    previousPR[2] = {0}       // [pitch, roll]       array to store the previous pitch and roll angles
;

// Task to handle the MPU
TaskHandle_t MPUTaskHandler = NULL;
void MPUTask(void *pvParameters) {

    // Variables to control and handle the MPU
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector

    // Variables to control and handle the rotation
    float avgPR[2] = {0};                                 // [pitch, roll]        array to store the average pitch and roll
    float offsetPR[2] = {0};                              // [pitch, roll]       array to store the offset pitch and roll
    float rotationSmootherArray[(smoothCycles)][2] = {0}; // [pitch, roll] array to store the last X values of pitch and roll to calculate and smooth the rotation
    int rotationSmootherIndex = 0;                        // Index to store the current value of the rotationSmootherArray

    // Variables to control the task and time
    const TickType_t taskDelay = MPUTaskDelay / portTICK_PERIOD_MS;
    TickType_t lastExecutionTime = xTaskGetTickCount();
    int timer = 0;

    // Initialize the Wire library and set the clock to 400kHz
    Wire.begin();
    Wire.setClock(400000);

    // Initialize the MPU and set the offsets
    mpu.initialize();
    int devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(1803); // 1803
    mpu.setYGyroOffset(1437); // 1437
    mpu.setZGyroOffset(1755); // 1755
    mpu.setZAccelOffset(22);  // 22

    if (devStatus == 0) {
        // Calibration of the MPU
        mpu.CalibrateAccel(10);
        mpu.CalibrateGyro(10);

        mpu.setDMPEnabled(true);

        digitalWrite(2, HIGH);

        dmpReady = true;
    } else {
        vTaskDelete(NULL);
    }

    // Main loop to handle the MPU, only when the DMP is ready
    while (dmpReady) {
        // Wait for the next cycle
        vTaskDelayUntil(&lastExecutionTime, taskDelay);

        // Read the FIFO buffer
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

            // Get the ypr by the quaternion and gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            // Loop to handle the smoothing of the rotation
            for (int i = 0; i <= 1; i++) {
                // Convert the angle to degrees (0-360) and store it in the array
                rotationSmootherArray[rotationSmootherIndex][i] = (ypr[i + 1] * 180 / M_PI) + 180;

                // Calculate the average of the last X values of the rotation
                for (int j = 0; j < (smoothCycles); j++)
                    avgPR[i] += rotationSmootherArray[j][i];

                // Store the average value of the rotation and account for the last read value
                avgPR[i] = avgPR[i] / ((smoothCycles) + 1);

                // If the user is touching the C2 button, set the offset to the current value
                if (touchRead(C2) < 20)
                    offsetPR[i] = avgPR[i];

                // Calculate the final value of the rotation
                currentPR[i] = avgPR[i] - offsetPR[i];
            }

            // Reset the average value of the rotation
            rotationSmootherIndex = (rotationSmootherIndex + 1) % (smoothCycles);
        }
    }
}

void setup() {
    // Creating the task to handle the MPU
    xTaskCreatePinnedToCore(MPUTask, "MPUTask", 20000, NULL, 1, &MPUTaskHandler, 1);

    delay(100);

    // Initialize the BLE Mouse
    bleMouse.begin();

    pinMode(2, OUTPUT);
}

// * Struct to handle the mouse buttons
typedef struct MouseButton {
    int touchPin;    // Touch pin to read the value
    int mouseButton; // Mouse button to press or release
    int touchValue;  // Value read from the touch pin
    bool pressed;    // State of the mouse button
} MouseButton;

// Delaration of mouse clicks
MouseButton mouseButtons[5] = {
    {A1, MOUSE_LEFT, 100, false},
    {A2, MOUSE_FORWARD, 100, false},
    {A3, MOUSE_BACK, 100, false},
    {B1, MOUSE_RIGHT, 100, false},
    {B2, MOUSE_MIDDLE, 100, false},
};

void loop() {

    // Check if the BLE Mouse is connected
    if (bleMouse.isConnected()) {

        // Don't move the mouse if the user is touching the C2 button
        if (touchRead(C2) > 20) {

            // Code for mouse movement and wheel
            if (touchRead(T4) < 20) {
                bleMouse.move(0, 0, -(currentPR[1] - previousPR[1]) * scrollSensitivity * LoopTimer, -(currentPR[0] - previousPR[0]) * scrollSensitivity * LoopTimer);
            } else {
                bleMouse.move(-(currentPR[0] - previousPR[0]) * 1.6 * pointerSensitivity * LoopTimer, -(currentPR[1] - previousPR[1]) * 0.9 * pointerSensitivity * LoopTimer, 0, 0);
            }
        }

        // After moving the mouse, store the previous values of the pitch and roll
        previousPR[0] = currentPR[0];
        previousPR[1] = currentPR[1];

        // Code for mouse buttons
        for (int i = 0; i < 5; i++) {
            // Read the touch value and the pressed state of the mouse button
            mouseButtons[i].touchValue = touchRead(mouseButtons[i].touchPin);

            // Check if the button is pressed or released, and send the corresponding command
            if (mouseButtons[i].touchValue > 20 && mouseButtons[i].pressed) {
                bleMouse.release(mouseButtons[i].mouseButton);
                mouseButtons[i].pressed = false;
            } else if (mouseButtons[i].touchValue < 20 && !mouseButtons[i].pressed) {
                bleMouse.press(mouseButtons[i].mouseButton);
                mouseButtons[i].pressed = true;
            }
        }
    }

    // Ensure the loop runs at a minimum rate
    if (millis() - minLoopTimer < LoopTimer) {
        delay(LoopTimer - (millis() - minLoopTimer));
    }
    minLoopTimer = millis();
}
