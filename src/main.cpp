// * Libraries
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Arduino.h>
#include <BleMouse.h>
#include <Preferences.h>

// *  Glove pin definition
#define A1 T0 // Left click
#define A2 T9 // Back
#define A3 T4 // Foward
#define B1 T7 // Right click
#define B2 T6 // Middle click
#define C1 T8 // Scroll (special)
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
Preferences preferences;

bool blinkState = false; // State of the LED on pin 13
bool isMPUReady = false;
bool isConfigMode = false;
bool configTogglePressed = false;
bool configLockPressed = false;
unsigned long ledBlinkTimer = 0;

enum MovementLockMode {
   MOVEMENT_UNLOCKED,
   LOCK_X_MOVEMENT,
   LOCK_Y_MOVEMENT,
};

MovementLockMode movementLockMode = MOVEMENT_UNLOCKED;

const int LoopTimer = 20;                            // Time (in ms) to wait between each loop in the main loop
const int MPUTaskDelay = 25;                         // Time (in ms) to wait between each loop in the MPU task
const int smoothTimer = 200;                         // Time (in ms) to smooth the rotation
const int smoothCycles = smoothTimer / MPUTaskDelay; // Number of cycles to smooth the rotation
const float movementConfigStep = 0.05;
const char *preferencesNamespace = "glouse";
const char *xConfigKey = "x_cfg_mult";
const char *yConfigKey = "y_cfg_mult";

int minLoopTimer = 0;            // Timer to ensure the loop runs at a minimum rate
unsigned long pin25LowTimer = 0; // Timer for the last LOW pulse start on GPIO 25
bool pin25IsLow = false;         // Tracks whether GPIO 25 is currently in the LOW pulse window

typedef struct TouchReadings {
   bool A1;
   bool A2;
   bool A3;
   bool B1;
   bool B2;
   bool C1;
   bool C2;
} TouchReadings;

TouchReadings touchedButtons = {false}; // Stores whether each touch button is touched

const unsigned long pin25LowInterval = 20000; // Time (in ms) between each GPIO 25 low write
const unsigned long pin25LowDuration = 500;   // Time (in ms) to keep GPIO 25 LOW

float
    pointerSensitivity = 0.5, // Sensitivity of the pointer
    scrollSensitivity = 0.05, // Sensitivity of the scroll
    xFreeModeMultiplier = 1.0,
    yFreeModeMultiplier = 1.0,
    xScreenMovementMultiplier = 1.9,
    yScreenMovementMultiplier = 0.9,
    currentPR[2] = {0}, // [pitch, roll]       array to store the calculated pitch and roll
    previousPR[2] = {0} // [pitch, roll]       array to store the previous pitch and roll angles
;

void saveMovementConfig() {
   preferences.putFloat(xConfigKey, xFreeModeMultiplier);
   preferences.putFloat(yConfigKey, yFreeModeMultiplier);
}

void loadMovementConfig() {
   xFreeModeMultiplier = preferences.getFloat(xConfigKey, xFreeModeMultiplier);
   yFreeModeMultiplier = preferences.getFloat(yConfigKey, yFreeModeMultiplier);
}

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
      isMPUReady = true;
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
            if (touchRead(C2) < 20) offsetPR[i] = avgPR[i];

            // Calculate the final value of the rotation
            currentPR[i] = avgPR[i] - offsetPR[i];
         }

         // Reset the average value of the rotation
         rotationSmootherIndex = (rotationSmootherIndex + 1) % (smoothCycles);
      }
   }
}

void setup() {
   preferences.begin(preferencesNamespace, false);
   loadMovementConfig();

   // Creating the task to handle the MPU
   xTaskCreatePinnedToCore(MPUTask, "MPUTask", 20000, NULL, 1, &MPUTaskHandler, 1);

   delay(100);

   // Initialize the BLE Mouse
   bleMouse.begin();

   pinMode(2, OUTPUT);
   pinMode(25, OUTPUT);
   digitalWrite(25, HIGH);
}

// * Struct to handle the mouse buttons
typedef struct MouseButton {
   int mouseButton; // Mouse button to press or release
   bool pressed;    // State of the mouse button
} MouseButton;

// Delaration of mouse clicks
MouseButton mouseButtons[5] = {
    {   MOUSE_LEFT, false},
    {MOUSE_FORWARD, false},
    {   MOUSE_BACK, false},
    {  MOUSE_RIGHT, false},
    { MOUSE_MIDDLE, false},
};

void readTouchPins() {
   touchedButtons.A1 = touchRead(A1) < 20;
   touchedButtons.A2 = touchRead(A2) < 20;
   touchedButtons.A3 = touchRead(A3) < 20;
   touchedButtons.B1 = touchRead(B1) < 20;
   touchedButtons.B2 = touchRead(B2) < 20;
   touchedButtons.C1 = touchRead(C1) < 20;
   touchedButtons.C2 = touchRead(C2) < 20;
}

bool isMouseButtonTouched(int index) {
   switch (index) {
   case 0:
      return touchedButtons.A1;
   case 1:
      return touchedButtons.A2;
   case 2:
      return touchedButtons.A3;
   case 3:
      return touchedButtons.B1;
   case 4:
      return touchedButtons.B2;
   default:
      return false;
   }
}

void keepBatteryAlivePinOn() {
   unsigned long currentTime = millis();

   if (!pin25IsLow && currentTime - pin25LowTimer >= pin25LowInterval) {
      digitalWrite(25, LOW);
      pin25IsLow = true;
      pin25LowTimer = currentTime;
   }

   if (pin25IsLow && currentTime - pin25LowTimer >= pin25LowDuration) {
      digitalWrite(25, HIGH);
      pin25IsLow = false;
   }
}

void toggleConfigMode() {
   bool wasConfigMode = isConfigMode;
   isConfigMode = !isConfigMode;
   movementLockMode = MOVEMENT_UNLOCKED;

   if (!wasConfigMode && isConfigMode) {
      for (int i = 0; i < 5; i++) {
         if (mouseButtons[i].pressed) {
            bleMouse.release(mouseButtons[i].mouseButton);
            mouseButtons[i].pressed = false;
         }
      }
   }

   if (wasConfigMode && !isConfigMode) saveMovementConfig();

   if (isConfigMode) {
      blinkState = true;
      ledBlinkTimer = millis();
      digitalWrite(2, HIGH);
   } else {
      digitalWrite(2, HIGH);
      blinkState = false;
   }
}

void handleConfigMode() {
   unsigned long currentTime = millis();

   if (currentTime - ledBlinkTimer >= 1000) {
      blinkState = !blinkState;
      ledBlinkTimer = currentTime;
      digitalWrite(2, blinkState ? HIGH : LOW);
   }

   if (touchedButtons.A1 && xFreeModeMultiplier < 5.0) xFreeModeMultiplier += movementConfigStep;
   if (touchedButtons.B1 && xFreeModeMultiplier > 0.5) xFreeModeMultiplier -= movementConfigStep;
   if (touchedButtons.A2 && yFreeModeMultiplier < 5.0) yFreeModeMultiplier += movementConfigStep;
   if (touchedButtons.A3 && yFreeModeMultiplier > 0.5) yFreeModeMultiplier -= movementConfigStep;

   if (touchedButtons.B2 && !configLockPressed) {
      if (movementLockMode == MOVEMENT_UNLOCKED) movementLockMode = LOCK_X_MOVEMENT;
      else if (movementLockMode == LOCK_X_MOVEMENT) movementLockMode = LOCK_Y_MOVEMENT;
      else movementLockMode = MOVEMENT_UNLOCKED;
   }

   configLockPressed = touchedButtons.B2;
}

void handleMouseConnection() {
   bool isConfigTogglePressed = touchedButtons.C1 && touchedButtons.C2;

   if (!isMPUReady) {
      configTogglePressed = isConfigTogglePressed;
      return;
   }

   if (isConfigTogglePressed && !configTogglePressed) toggleConfigMode();

   configTogglePressed = isConfigTogglePressed;

   if (isConfigMode) handleConfigMode();
   else configLockPressed = false;
}

void handleConnectedMouse() {
   // Don't move the mouse if the user is touching the C2 button
   if (!touchedButtons.C2) {
      float deltaXMovement = currentPR[0] - previousPR[0];
      float deltaYMovement = currentPR[1] - previousPR[1];

      float deltaX = deltaXMovement * xFreeModeMultiplier * xScreenMovementMultiplier * pointerSensitivity * LoopTimer;
      float deltaY = deltaYMovement * yFreeModeMultiplier * yScreenMovementMultiplier * pointerSensitivity * LoopTimer;

      if (movementLockMode == LOCK_X_MOVEMENT) deltaX = 0;
      else if (movementLockMode == LOCK_Y_MOVEMENT) deltaY = 0;

      // Wheel when C1 is touched
      if (touchedButtons.C1) bleMouse.move(0, 0, -deltaYMovement * scrollSensitivity * LoopTimer, -deltaXMovement * scrollSensitivity * LoopTimer);
      else bleMouse.move(deltaX, deltaY, 0, 0); // Mouse Pointer
   }

   // After moving the mouse, store the previous values of the pitch and roll.
   previousPR[0] = currentPR[0];
   previousPR[1] = currentPR[1];

   if (isConfigMode) return;

   // Code for mouse buttons
   for (int i = 0; i < 5; i++) {
      bool isTouched = isMouseButtonTouched(i);

      // Check if the button is pressed or released, and send the corresponding command
      if (!isTouched && mouseButtons[i].pressed) {
         bleMouse.release(mouseButtons[i].mouseButton);
         mouseButtons[i].pressed = false;
      } else if (isTouched && !mouseButtons[i].pressed) {
         bleMouse.press(mouseButtons[i].mouseButton);
         mouseButtons[i].pressed = true;
      }
   }
}

void loop() {
   readTouchPins();

   keepBatteryAlivePinOn();

   handleMouseConnection();

   if (bleMouse.isConnected()) handleConnectedMouse();

   // Ensure the loop runs at a minimum rate
   if (millis() - minLoopTimer < LoopTimer) delay(LoopTimer - (millis() - minLoopTimer));
   minLoopTimer = millis();
}
