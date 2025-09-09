#include <Arduino.h>

// ===== CONFIGURATION CONSTANTS =====
// Current sensor parameters
const float CURRENT_SENSOR_SENSITIVITY = 0.040; // 40mV/A for ACS758LCB-050B
const float CURRENT_SENSOR_QUIESCENT_VOLTAGE = 2.5; // Typical Vq output
const int CURRENT_SENSOR_PIN = A1; // Analog pin for current sensor

// PWM control
const int PWM_PIN = 9; // PWM output pin
const int POTENTIOMETER_PIN = A0; // Potentiometer for current setting

// Current limits
//const float MIN_CURRENT_SETTING = 5.0; // Minimum adjustable current (A)
//const float MAX_CURRENT_SETTING = 25.0; // Maximum adjustable current (A)
//const float STOP_CURRENT_THRESHOLD = -2.0; // Stop if current goes below this value (A)
const float MIN_CURRENT_SETTING = 0.5; // Minimum adjustable current (A)
const float MAX_CURRENT_SETTING = 1.5; // Maximum adjustable current (A)
const float STOP_CURRENT_THRESHOLD = -2.0; // Stop if current goes below this value (A)

// Timing parameters
const unsigned long CURRENT_MEASUREMENT_INTERVAL = 20; // ~50 Hz (20ms)
const unsigned long RETRY_INTERVAL = 60000; // 60 seconds retry delay

// EMA filter
const float EMA_ALPHA = 0.1; // EMA filter coefficient

// ===== GLOBAL VARIABLES =====
// Sensor calibration
float currentSensorOffset = 0.0; // Will be calibrated at startup

// Current measurement
float filteredCurrent = 0.0;
float targetCurrent = 0.0;

// State machine
enum ChargingState { 
  STATE_IDLE, 
  STATE_CALIBRATING, 
  STATE_CHARGING, 
  STATE_STOPPED, 
  STATE_FAULT 
};
ChargingState chargingState = STATE_CALIBRATING;

// Timing control
unsigned long lastCurrentMeasurementTime = 0;
unsigned long lastRetryTime = 0;
unsigned long stateEntryTime = 0;

// ===== FUNCTION PROTOTYPES =====
void calibrateCurrentSensor();
float readCurrent();
void updateCurrentMeasurement();
void updateChargingState();
void setPWMOutput(float currentError);
void handleIdleState();
void handleCalibratingState();
void handleChargingState();
void handleStoppedState();
void handleFaultState();

// ===== SETUP =====
void setup() {
  // Initialize pins
  pinMode(PWM_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Start with PWM off
  analogWrite(PWM_PIN, 0);
  
  // Initialize serial
  Serial.begin(115200);
  Serial.println("Battery Charger Controller Starting...");
  
  // Start calibration process
  chargingState = STATE_CALIBRATING;
  stateEntryTime = millis();
}

// ===== MAIN LOOP =====
void loop() {
  // Update current measurement at fixed interval
  if (millis() - lastCurrentMeasurementTime >= CURRENT_MEASUREMENT_INTERVAL) {
    updateCurrentMeasurement();
    lastCurrentMeasurementTime = millis();
  }
  
  // Read potentiometer to adjust target current
  int potValue = analogRead(POTENTIOMETER_PIN);
  targetCurrent = map(potValue, 0, 1023, MIN_CURRENT_SETTING, MAX_CURRENT_SETTING);
  
  // Update charging state machine
  updateChargingState();
  
  // Small delay to prevent watchdog issues
  delay(10);
}

// ===== CURRENT MEASUREMENT FUNCTIONS =====
void calibrateCurrentSensor() {
  Serial.println("Calibrating current sensor...");
  
  // Take multiple samples for better accuracy
  const int numSamples = 100;
  float sum = 0;
  
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(CURRENT_SENSOR_PIN);
    delay(5);
  }
  
  // Calculate average ADC value and convert to voltage
  float averageADC = sum / numSamples;
  float voltage = (averageADC / 1023.0) * 5.0;
  
  // Set offset (should be close to 2.5V at 0A)
  currentSensorOffset = voltage;
  
  Serial.print("Calibration complete. Offset voltage: ");
  Serial.print(currentSensorOffset, 3);
  Serial.println("V");
}

float readCurrent() {
  // Read raw ADC value
  int adcValue = analogRead(CURRENT_SENSOR_PIN);
  
  // Convert to voltage
  float voltage = (adcValue / 1023.0) * 5.0;
  
  // Calculate current using calibrated offset
  float current = (voltage - currentSensorOffset) / CURRENT_SENSOR_SENSITIVITY;
  
  return current;
}

void updateCurrentMeasurement() {
  // Read raw current
  float rawCurrent = readCurrent();
  
  // Apply EMA filter
  filteredCurrent = (EMA_ALPHA * rawCurrent) + ((1 - EMA_ALPHA) * filteredCurrent);
}

// ===== STATE MACHINE FUNCTIONS =====
void updateChargingState() {
  switch (chargingState) {
    case STATE_CALIBRATING:
      handleCalibratingState();
      break;
    case STATE_IDLE:
      handleIdleState();
      break;
    case STATE_CHARGING:
      handleChargingState();
      break;
    case STATE_STOPPED:
      handleStoppedState();
      break;
    case STATE_FAULT:
      handleFaultState();
      break;
  }
}

void handleCalibratingState() {
  // Perform one-time calibration
  if (millis() - stateEntryTime > 1000) {
    calibrateCurrentSensor();
    chargingState = STATE_IDLE;
    stateEntryTime = millis();
    Serial.println("Entering IDLE state");
  }
}

void handleIdleState() {
  // Always start with PWM off in idle
  analogWrite(PWM_PIN, 0);
  
  // Check if we should start charging
  if (filteredCurrent < STOP_CURRENT_THRESHOLD) {
    Serial.println("Reverse current detected! Staying in IDLE");
    return;
  }
  
  // Start charging (in a real system, you'd check voltage conditions here too)
  chargingState = STATE_CHARGING;
  stateEntryTime = millis();
  Serial.println("Entering CHARGING state");
}

void handleChargingState() {
  // Safety check: stop immediately if reverse current detected
  if (filteredCurrent < STOP_CURRENT_THRESHOLD) {
    Serial.println("Reverse current detected during charging!");
    chargingState = STATE_STOPPED;
    stateEntryTime = millis();
    analogWrite(PWM_PIN, 0); // Turn off immediately
    return;
  }
  
  // Calculate current error
  float currentError = targetCurrent - filteredCurrent;
  
  // Adjust PWM output based on error
  setPWMOutput(currentError);
  
  // Debug output
  Serial.print("Target: ");
  Serial.print(targetCurrent, 1);
  Serial.print("A, Actual: ");
  Serial.print(filteredCurrent, 1);
  Serial.print("A, PWM: ");
  Serial.println(analogRead(PWM_PIN));
}

void handleStoppedState() {
  // Ensure PWM is off
  analogWrite(PWM_PIN, 0);
  
  // Check if it's time to retry
  if (millis() - stateEntryTime >= RETRY_INTERVAL) {
    chargingState = STATE_IDLE;
    stateEntryTime = millis();
    Serial.println("Retry timeout reached, returning to IDLE");
  }
}

void handleFaultState() {
  // PWM remains off in fault state
  analogWrite(PWM_PIN, 0);
  
  // Flash LED to indicate fault
  digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
}

// ===== PWM CONTROL FUNCTION =====
void setPWMOutput(float currentError) {
  // Simple P-controller with clamping
  static int pwmValue = 0;
  
  // Adjust PWM based on error (you can tune this gain)
  pwmValue += currentError * 2.0;
  
  // Clamp PWM value to valid range
  pwmValue = constrain(pwmValue, 0, 255);
  
  // Update PWM output
  analogWrite(PWM_PIN, pwmValue);
}