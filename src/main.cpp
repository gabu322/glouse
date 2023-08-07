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

MPU6050 mpu;
float baseRotations[3];
bool dmpReady = false; // set true if DMP init was successful
uint8_t devStatus; // return status after each device operation (0 = success, !0
                   // = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float ypr[3];

const int bufferSize = 25; // Assuming 20ms delay, this is 0.5 seconds' worth
float yprBuffer[bufferSize];
int bufferIndex = 0;
unsigned long lastBufferUpdateTime = 0;
bool isReadyToCheck = true; // Flag to indicate when to start checking
bool stable = false;        // Flag to indicate when the mouse is stable
TaskHandle_t MPUTaskHandler = NULL;
void MPUTask(void *pvParameters) {
    vTaskSuspend(NULL);

    const TickType_t taskDelay = 20 / portTICK_PERIOD_MS;
    TickType_t lastExecutionTime = xTaskGetTickCount();

    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                        : F("MPU6050 connection failed"));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(1803); // 1803 factory default for my test chip
    mpu.setYGyroOffset(1437); // 1437 factory default for my test chip
    mpu.setZGyroOffset(1755); // 1755 factory default for my test chip
    mpu.setZAccelOffset(22);  // 22 factory default for my test chip

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
                        if (abs(yprBuffer[i] - ypr[0]) > 0.25) {
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
                } else {
                    if (Serial) {
                        Serial.print("ypr\t");
                        Serial.print(ypr[0]);
                        Serial.print("\t");
                        Serial.print(ypr[1]);
                        Serial.print("\t");
                        Serial.println(ypr[2]);
                    }
                }
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

double actualTime = 0;
void loop() {
    if (bleMouse.isConnected()) {
        if (stable) {
            bleMouse.move(-(baseRotations[1] - ypr[1]) / 2, 0, 0);
            bleMouse.move(0, (baseRotations[2] - ypr[2]) / 2, 0);
        }
        if (digitalRead(taskButtonPin) == HIGH &&
            eTaskGetState(MPUTaskHandler) == 3) {
            vTaskResume(MPUTaskHandler);
        }
        if(touchRead(4) < 15) {
            bleMouse.press(MOUSE_LEFT);
        } else {
            bleMouse.release(MOUSE_LEFT);
        }
    }
    if (!(millis() - actualTime) < 20) {
        actualTime = millis();
        delay(20 - (millis() - actualTime));
    }
}
