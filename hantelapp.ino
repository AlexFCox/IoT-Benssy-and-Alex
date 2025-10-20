#include "LIS3DH_zhaw.h"
#include <Adafruit_Sensor.h>
#include "Particle.h"

SYSTEM_MODE(AUTOMATIC);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

Adafruit_LIS3DH lis = Adafruit_LIS3DH();
int counter = 0;
int configOk = 0;

// Global variables for sensor readings
float accelX = 0;
float accelY = 0;
float accelZ = 0;
float accelMagnitude = 0;

// Function declarations
void readSensor();

void readSensor() {
    lis.read();
    accelX = lis.x_g * 9.80665F;  // Convert to m/s²
    accelY = lis.y_g * 9.80665F;
    accelZ = lis.z_g * 9.80665F;

    // Calculate magnitude: sqrt(x² + y² + z²)
    accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);
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

        Serial.printlnf("%d,%f,%f,%f,%f", counter, accelX, accelY, accelZ, accelMagnitude);
        counter++;
	}
    delay(50);
}