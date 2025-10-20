#include "LIS3DH_zhaw.h"
#include <Adafruit_Sensor.h>
#include "Particle.h"

SYSTEM_MODE(AUTOMATIC);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

Adafruit_LIS3DH lis = Adafruit_LIS3DH();
int counter = 0;
int configOk = 0;
float x_accel, y_accel, z_accel;

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
	    lis.read();
        x_accel = lis.x_g * 9.80665F;
        y_accel = lis.y_g * 9.80665F;
        z_accel = lis.z_g * 9.80665F;
        Serial.printlnf("%d,%f,%f,%f", counter, x_accel, y_accel, z_accel);
        counter++;
	}
    delay(50);
}