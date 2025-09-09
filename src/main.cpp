#include <Arduino.h>

// ===== CONFIGURATION CONSTANTS =====
const float CURRENT_SENSOR_SENSITIVITY = 0.040; // 40mV/A for ACS758LCB-050B
const int CURRENT_SENSOR_PIN = A1;
const int PWM_PIN = 9;
const int POTENTIOMETER_PIN = A0;

// Current limits
const float MIN_CURRENT_SETTING = 0.0;
const float MAX_CURRENT_SETTING = 1.5;

// Fixed offset based on multimeter measurement
const float CURRENT_SENSOR_OFFSET = 2.502; // Volts

// Timing
const unsigned long CURRENT_MEASUREMENT_INTERVAL = 20; // 20ms
const unsigned long PRINT_INTERVAL = 500; // 500ms

// Controller settings
const float KP = 5.0;  // Lower gain

// Filter settings
const float FILTER_ALPHA = 0.01;  // Slow filter (lower value = slower)

// PWM ramp rate limit (max change per interval)
const int PWM_RAMP_LIMIT = 1;

// ===== GLOBAL VARIABLES =====
float filteredCurrent = 0.0;
float targetCurrent = 0.0;
int currentPWM = 0;
int previousPWM = 0;

unsigned long lastCurrentMeasurementTime = 0;
unsigned long lastPrintTime = 0;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  
  Serial.begin(115200);
  Serial.println("Battery Charger Controller Starting...");
  
  analogReference(DEFAULT);
}

void loop() {
  unsigned long currentMillis = millis();

  // Update current measurement
  if (currentMillis - lastCurrentMeasurementTime >= CURRENT_MEASUREMENT_INTERVAL) {
    int adcValue = analogRead(CURRENT_SENSOR_PIN);
    float voltage = (adcValue / 1023.0) * 5.0;
    float current = (voltage - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY;
    
    // Slow low-pass filter
    filteredCurrent = (FILTER_ALPHA * current) + ((1 - FILTER_ALPHA) * filteredCurrent);
    
    lastCurrentMeasurementTime = currentMillis;
  }

  // Read potentiometer and calculate target current
  int potValue = analogRead(POTENTIOMETER_PIN);
  targetCurrent = MIN_CURRENT_SETTING + (MAX_CURRENT_SETTING - MIN_CURRENT_SETTING) * (potValue / 1023.0);

  // P-controller with lower gain
  float error = targetCurrent - filteredCurrent;
  int pwmChange = error * KP;
  
  // Limit PWM change to ramp rate
  pwmChange = constrain(pwmChange, -PWM_RAMP_LIMIT, PWM_RAMP_LIMIT);
  currentPWM = constrain(previousPWM + pwmChange, 0, 255);
  previousPWM = currentPWM;
  
  analogWrite(PWM_PIN, currentPWM);

  // Print debug info
  if (currentMillis - lastPrintTime >= PRINT_INTERVAL) {
    int adcValue = analogRead(CURRENT_SENSOR_PIN);
    float voltage = (adcValue / 1023.0) * 5.0;
    
    Serial.print("Pot: ");
    Serial.print(potValue);
    Serial.print(" | Target: ");
    Serial.print(targetCurrent, 2);
    Serial.print("A | Actual: ");
    Serial.print(filteredCurrent, 2);
    Serial.print("A | Error: ");
    Serial.print(error, 3);
    Serial.print("A | PWM: ");
    Serial.print(currentPWM);
    Serial.print(" | Voltage: ");
    Serial.print(voltage, 4);
    Serial.println("V");
    
    lastPrintTime = currentMillis;
  }

  delay(10);
}