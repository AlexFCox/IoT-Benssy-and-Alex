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

// Step 4: Phase detection variables
float filteredAccel = 0;
float movementThreshold = 2.0;  // Adjust this threshold based on testing
String currentState = "IDLE";
String prevState = "IDLE";
float upperThreshold = 3.0;  // Higher threshold to enter movement state
float lowerThreshold = 1.0;  // Lower threshold to exit movement state (hysteresis)
float maxThreshold = 6.0;  // Maximum threshold to filter out drops/impacts

// Step 5: Rep counting variables
unsigned long repStartTime = 0;
unsigned long lastRepTime = 0;
const unsigned long MIN_REP_TIME = 800;  // 0.8s minimum per rep
const int REP_TIMES_SIZE = 10;
unsigned long repTimes[10];
int repTimeIndex = 0;
int repCount = 0;
double avgRepTime = 0;  // Changed to double for consistency

// Step 6: Training status variables
bool trainingActive = false;
bool trainingStartEventSent = false;
const unsigned long MAX_IDLE_TIME = 30000;  // 30 seconds max idle time

// Step 7: Cloud variables and Kadenz (cadence) tracking
int prevRepCount = 0;
const int KADENZ_BUFFER_SIZE = 50;
unsigned long kadenzBuffer[50];
int kadenzIndex = 0;
double avgKadenz = 0;  // Current cadence in last 15 seconds (double for Particle.variable)
const unsigned long KADENZ_WINDOW = 15000;  // 15 seconds
unsigned long lastKadenzEventTime = 0;  // Last time we sent Kadenz event
const unsigned long KADENZ_EVENT_INTERVAL = 15000;  // Send event every 15 seconds
int totalTrainingReps = 0;  // Total reps since training started
unsigned long trainingStartTime = 0;  // When training started
double avgTrainingKadenz = 0;  // Average cadence for entire training session (double for calculations)

// Function declarations
void readSensor();
void addToMovingAvg();
void calcMovingAvg();
void detectRepPhase();
void detectMovementDir();
void countValidRep();
void checkTrainingStatus();
void resetTrainingVariables();
void updateCloudAndKadenz();

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

void detectRepPhase() {
    // Calculate filtered acceleration (remove smoothed baseline)
    filteredAccel = accelMagnitude - smoothedAccel;

    // Check if filtered acceleration exceeds positive threshold
    if (filteredAccel > movementThreshold) {
        currentState = "MOVING_UP";
        detectMovementDir();
    }
    // Check if filtered acceleration exceeds negative threshold
    else if (filteredAccel < -movementThreshold) {
        currentState = "MOVING_DOWN";
        detectMovementDir();
    }
    // Otherwise, no significant movement
    else {
        currentState = "IDLE";
        detectMovementDir();
    }
}

void detectMovementDir() {
    // State machine for detecting movement direction changes

    // Filter out extreme accelerations (drops, impacts, sudden jerks)
    if (abs(filteredAccel) > maxThreshold) {
        // Sudden impact detected - ignore this reading and force to IDLE
        currentState = "IDLE";
        return;
    }

    if (currentState == "IDLE") {
        // From IDLE: Check if movement exceeds upper threshold
        if (filteredAccel > upperThreshold) {
            currentState = "MOVING_UP";
        }
        else if (filteredAccel < -upperThreshold) {
            currentState = "MOVING_DOWN";
        }
        // else stay IDLE
    }
    else if (currentState == "MOVING_UP") {
        // From MOVING_UP: Check if movement drops below lower threshold
        if (filteredAccel < lowerThreshold) {
            currentState = "IDLE";
        }
        // else stay MOVING_UP
    }
    else if (currentState == "MOVING_DOWN") {
        // From MOVING_DOWN: Check if movement rises above negative lower threshold
        if (filteredAccel > -lowerThreshold) {
            currentState = "IDLE";
        }
        // else stay MOVING_DOWN
    }

    // Note: prevState is now updated in countValidRep() AFTER checking for transitions
}

void countValidRep() {
    unsigned long currentTime = millis();

    // Check if state changed
    if (currentState == prevState) {
        return;  // No state change, nothing to do
    }

    // Check if this completes a rep cycle
    // Rep is complete when transitioning back to IDLE from either direction
    bool repCycleComplete = (prevState == "MOVING_UP" && currentState == "IDLE") ||
                            (prevState == "MOVING_DOWN" && currentState == "IDLE");

    if (!repCycleComplete) {
        // Update prevState before returning (state changed but not a complete rep)
        prevState = currentState;
        return;  // Not completing a rep cycle
    }

    // Check if valid rep timing (must exceed minimum rep time)
    if (currentTime - repStartTime <= MIN_REP_TIME) {
        // Too fast, skip counting but reset timer
        repStartTime = currentTime;
        prevState = currentState;  // Update prevState before returning
        return;
    }

    // Valid rep detected! Increment count
    repCount++;

    // Store rep duration in circular buffer
    repTimes[repTimeIndex] = currentTime - lastRepTime;
    repTimeIndex = (repTimeIndex + 1) % REP_TIMES_SIZE;

    // Calculate average rep time from array
    unsigned long sum = 0;
    for (int i = 0; i < REP_TIMES_SIZE; i++) {
        sum += repTimes[i];
    }
    avgRepTime = sum / (double)REP_TIMES_SIZE;

    // Update timing variables
    lastRepTime = currentTime;
    repStartTime = currentTime;

    // Update prevState after processing the rep
    prevState = currentState;
}

void resetTrainingVariables() {
    // Reset all rep counting and timing variables
    repCount = 0;
    prevRepCount = 0;
    repStartTime = 0;
    lastRepTime = 0;
    repTimeIndex = 0;
    avgRepTime = 0;

    // Clear rep times buffer
    for (int i = 0; i < REP_TIMES_SIZE; i++) {
        repTimes[i] = 0;
    }

    // Reset kadenz tracking
    kadenzIndex = 0;
    avgKadenz = 0;
    avgTrainingKadenz = 0;
    lastKadenzEventTime = 0;
    totalTrainingReps = 0;
    trainingStartTime = 0;
    for (int i = 0; i < KADENZ_BUFFER_SIZE; i++) {
        kadenzBuffer[i] = 0;
    }

    // Reset training flags
    trainingStartEventSent = false;
}

void checkTrainingStatus() {
    unsigned long currentTime = millis();

    // Check if training is currently active
    if (!trainingActive) {
        // Training not active - check if we should start

        // Need at least 3 reps to start training
        if (repCount >= 3) {
            trainingActive = true;

            // Send training started event (only once)
            if (!trainingStartEventSent) {
                // Record training start time and initial rep count
                trainingStartTime = currentTime;
                totalTrainingReps = repCount;
                lastKadenzEventTime = currentTime;  // Initialize for periodic events

                Particle.publish("Training", "Training Begonnen");
                trainingStartEventSent = true;
            }
        }
        else {
            // Less than 3 reps - check if user has been idle too long
            if (lastRepTime > 0 && (currentTime - lastRepTime > MAX_IDLE_TIME)) {
                // User idle for 30+ seconds with < 3 reps, reset everything
                resetTrainingVariables();
            }
            return;
        }
    }
    else {
        // Training is active - check if we should end it

        // Check if idle time exceeds 3x average rep time
        if (lastRepTime > 0 && avgRepTime > 0 &&
            (currentTime - lastRepTime > (avgRepTime * 3))) {

            // Calculate average Kadenz for entire training session
            unsigned long trainingDuration = currentTime - trainingStartTime;  // in milliseconds
            if (trainingDuration > 0) {
                // Calculate reps per minute for entire training
                double trainingMinutes = trainingDuration / 60000.0;  // Convert ms to minutes
                avgTrainingKadenz = repCount / trainingMinutes;
            }

            // Training ended - send events with rep count and average Kadenz
            Particle.publish("Training", String::format("Training Beendet - %d Reps", repCount));
            delay(1000);  // Small delay to avoid rate limiting
            Particle.publish("Kadenz", avgTrainingKadenz);
            //Particle.publish("Kadenz", String::format("Durchschnitt: %.1f Reps/min", avgTrainingKadenz));

            // Reset everything
            resetTrainingVariables();
            trainingActive = false;
        }
    }
}

void updateCloudAndKadenz() {
    unsigned long currentTime = millis();

    // Check if we just completed a rep (repCount increased)
    if (repCount != prevRepCount) {
        // Rep count increased - add timestamp to kadenz buffer
        kadenzBuffer[kadenzIndex] = currentTime;
        kadenzIndex = (kadenzIndex + 1) % KADENZ_BUFFER_SIZE;

        // Update previous rep count
        prevRepCount = repCount;
    }

    // Calculate reps in last 15 seconds (always update, even without new rep)
    int repsIn15s = 0;
    for (int i = 0; i < KADENZ_BUFFER_SIZE; i++) {
        if (kadenzBuffer[i] > 0 && (currentTime - kadenzBuffer[i] <= KADENZ_WINDOW)) {
            repsIn15s++;
        }
    }

    // Calculate current kadenz (reps per minute)
    // repsIn15s * 4 = reps per minute
    avgKadenz = repsIn15s * 4.0;

    // Send Kadenz event every 15 seconds during active training
    if (trainingActive && (currentTime - lastKadenzEventTime >= KADENZ_EVENT_INTERVAL)) {
        Particle.publish("Kadenz", avgKadenz);
        //Particle.publish("Kadenz", String::format("Aktuelle Kadenz: %.1f Reps/min", avgKadenz));
        lastKadenzEventTime = currentTime;
    }
}

void setup() {
    Serial.begin(9600);

    // Register cloud variables
    Particle.variable("reps", repCount);
    Particle.variable("kadenz", avgKadenz);

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

	    // Step 4: Detect repetition phase
	    detectRepPhase();

	    // Step 5: Count valid repetitions
	    countValidRep();

	    // Step 6: Check training status
	    checkTrainingStatus();

	    // Step 7: Update cloud variables and calculate Kadenz
	    updateCloudAndKadenz();

        // Debug output: comprehensive training data
        Serial.printlnf("Count:%d | State:%s | Filtered:%.2f | Reps:%d | Kadenz:%.1f | Training:%s | AvgRepTime:%.0fms",
            counter,
            currentState.c_str(),
            filteredAccel,
            repCount,
            avgKadenz,
            trainingActive ? "ACTIVE" : "IDLE",
            avgRepTime);
        counter++;
	}
    delay(50);
}