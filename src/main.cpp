#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Arduino.h>
#include <BleMouse.h>

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

int processValue(int input) {
    if (input >= -2.5 && input <= 2.5) {
        return 0;
    }

    int savedValue = input;
    int mappedValue;
    // Map the saved value to a range between 0 and 30
    if (input > 0) {
        mappedValue = map(savedValue, 0, 45, 0, 30);
    } else {
        mappedValue = map(savedValue, 0, -30, 0, 30);
    }

    // Apply the formula
    int processedValue = ((mappedValue / 4) + sq(mappedValue / 3)) / 2;

    // Add the original sign to the result
    if (input < 0) {
        processedValue *= -1;
    }

    return processedValue;
}

MPU6050 mpu;
float ypr[3];
float baseRotations[3];

bool dmpReady = false; // set true if DMP init was successful
uint8_t devStatus;
uint16_t packetSize;       // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];    // FIFO storage buffer
Quaternion q;              // [w, x, y, z]         quaternion container
VectorFloat gravity;       // [x, y, z]            gravity vector
const int bufferSize = 40; // Assuming 20ms delay, this is 0.5 seconds' worth
float yprBuffer[bufferSize];
int bufferIndex = 0;
unsigned long lastBufferUpdateTime = 0;
bool isReadyToCheck = true; // Flag to indicate when to start checking

bool stable = false; // Flag to indicate when the mouse is stable
TaskHandle_t MPUTaskHandler = NULL;
void MPUTask(void *pvParameters) {
    // Suspend the task until the button is pressed
    vTaskSuspend(NULL);

    // Set the minimun delay between task executions to 20ms
    const TickType_t taskDelay = 20 / portTICK_PERIOD_MS;
    TickType_t lastExecutionTime = xTaskGetTickCount();

    // Initialize the MPU6050 And set the offsets
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                        : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(1803); // 1803
    mpu.setYGyroOffset(1437); // 1437
    mpu.setZGyroOffset(1755); // 1755
    mpu.setZAccelOffset(22);  // 22

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    int lastTaskStartTime = millis();
    digitalWrite(2, LOW);
    for (;;) {
        vTaskDelayUntil(&lastExecutionTime, taskDelay);
        if (!dmpReady) {
            Serial.println("DMP not ready!");
            continue;
        }
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            for (int i = 0; i < 3; i++) {
                ypr[i] = ypr[i] * 180 / M_PI;
            }

            if (!stable) {
                yprBuffer[bufferIndex] = ypr[0];
                bufferIndex = (bufferIndex + 1) % bufferSize;
            }

            // Check if 1 second has elapsed to set the flag isReadyToCheck
            if (millis() - lastTaskStartTime >= 1000) {
                if (isReadyToCheck) {
                    stable = true;
                    for (int i = 0; i < bufferSize; i++) {
                        if (abs(yprBuffer[i] - ypr[0]) > 0.2) {
                            stable = false;
                        }
                    }
                    if (stable) {
                        isReadyToCheck = false;
                        digitalWrite(2, HIGH);
                        for (int i = 0; i < 3; i++) {
                            baseRotations[i] = ypr[i];
                        }
                    }
                    if (Serial) {
                        Serial.print("Stabilyzing\t");
                        Serial.print(ypr[0]);
                        Serial.print("\t");
                    }
                } /*else {
                    if (Serial) {
                        Serial.print("Values: \t");
                        Serial.print(ypr[1]);
                        Serial.print("\t");
                        Serial.print(
                            -processValue(-(ypr[1] - baseRotations[1])));
                        Serial.print("\t");
                        Serial.print(ypr[2]);
                        Serial.print("\t");
                        Serial.print(
                            processValue(-(ypr[2] - baseRotations[2])));
                        Serial.println("\t");
                    }
                }*/
            }
        }
    }
}

BleMouse bleMouse;
int taskButtonPin = 23;
void setup() {
    Serial.begin(115200);
    delay(300);

    bleMouse.begin();
    delay(300);
    Wire.begin();
    Wire.setClock(400000);
    xTaskCreatePinnedToCore(MPUTask, "MPUTask", 10000, NULL, 1, &MPUTaskHandler,
                            1);
    pinMode(2, OUTPUT);
    pinMode(taskButtonPin, INPUT_PULLDOWN);
}

bool firstConnection = false;
double actualTime = 0;
void loop() {
    if (bleMouse.isConnected()) {
        if(!firstConnection){
            firstConnection = true;
            digitalWrite(2, HIGH);
        }
        if (stable) {
            if (Serial) {
                Serial.print("Values: \t");
                Serial.print(ypr[1]);
                Serial.print("\t");
                Serial.print(processValue(ypr[1] - baseRotations[1]));
                Serial.print("\t");
                Serial.print(ypr[2]);
                Serial.print("\t");
                Serial.print(-processValue(ypr[2] - baseRotations[2]));
                Serial.println("\t");
            }
            bleMouse.move(processValue(ypr[1] - baseRotations[1]), 0, 0);
            bleMouse.move(0, processValue(-(ypr[2] - baseRotations[2])), 0);
        }
        if (digitalRead(taskButtonPin) == HIGH &&
            eTaskGetState(MPUTaskHandler) == 3) {
            vTaskResume(MPUTaskHandler);
        }
        if (touchRead(4) < 15) {
            bleMouse.press(MOUSE_LEFT);
        } else {
            bleMouse.release(MOUSE_LEFT);
        }
    }
    // Delay to keep the loop from running too fast
    if (!(millis() - actualTime) < 20) {
        actualTime = millis();
        delay(20 - (millis() - actualTime));
    }
}
