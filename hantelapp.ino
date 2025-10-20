#include "LIS3DH_zhaw.h"
#include <Adafruit_Sensor.h>
#include "Particle.h"

SYSTEM_MODE(AUTOMATIC);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

Adafruit_LIS3DH lis = Adafruit_LIS3DH();
int counter = 0;
int configOk = 0;

// Step 1: Global variables for sensor readings
float accelX = 0;
float accelY = 0;
float accelZ = 0;
float accelMagnitude = 0;

// Step 2: Moving average buffer variables
const int BUFFER_SIZE = 20;
float movingAvgBuffer[20];
int bufferIndex = 0;
bool bufferFull = false;

// Step 3: Moving average calculation variables
float smoothedAccel = 0;
float prevSmoothedAccel = 0;

// Function declarations
void readSensor();
void addToMovingAvg();
void calcMovingAvg();

void readSensor() {
    lis.read();
    accelX = lis.x_g * 9.80665F;  // Convert to m/s²
    accelY = lis.y_g * 9.80665F;
    accelZ = lis.z_g * 9.80665F;

    // Calculate magnitude: sqrt(x² + y² + z²)
    accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
}

void addToMovingAvg() {
    // Store current magnitude in buffer
    movingAvgBuffer[bufferIndex] = accelMagnitude;

    // Increment buffer index
    bufferIndex++;

    // Check if buffer is full and wrap around
    if (bufferIndex >= BUFFER_SIZE) {
        bufferIndex = 0;
        bufferFull = true;
    }
}

void calcMovingAvg() {
    // Only calculate if buffer is full
    if (!bufferFull) {
        return;
    }

    // Store previous value
    prevSmoothedAccel = smoothedAccel;

    // Sum all values in buffer
    float sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        sum += movingAvgBuffer[i];
    }

    // Calculate average
    smoothedAccel = sum / BUFFER_SIZE;
}

void setup() {
    Serial.begin(9600);
    
    if (! lis.begin(0x18)) {  
		Particle.publish("LIS3DH", "configuration failed");
    } else {
        lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!
		Particle.publish("LIS3DH", "configured successfully");
		configOk = 1;
    }
    
    delay(100);
}

void loop() {
	if (configOk == 1) {
	    // Step 1: Read sensor data
	    readSensor();

	    // Step 2: Add to moving average buffer
	    addToMovingAvg();

	    // Step 3: Calculate moving average
	    calcMovingAvg();

        // Debug output: counter, raw magnitude, smoothed magnitude, buffer status
        Serial.printlnf("%d,%.2f,%.2f,%s",
            counter,
            accelMagnitude,
            smoothedAccel,
            bufferFull ? "FULL" : "FILLING");
        counter++;
	}
    delay(50);
}