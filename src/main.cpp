#include <Arduino.h>

// ===== KONFIGURATIONSKONSTANTER =====
const float CURRENT_SENSOR_SENSITIVITY = 0.040;
const int CURRENT_SENSOR_PIN = A1;
const int PWM_PIN = 9;
const int POTENTIOMETER_PIN = A0;

// Strømgrænser
const float MIN_CURRENT_SETTING = 0.0;
const float MAX_CURRENT_SETTING = 1.5;
const float REVERSE_CURRENT_THRESHOLD = -0.5;

// Timing
const unsigned long CURRENT_MEASUREMENT_INTERVAL = 20;
const unsigned long PRINT_INTERVAL = 500;
const unsigned long RAMP_TIME_MS = 20000; // 20 sekunder ramp tid
const unsigned long RETRY_DELAY_MS = 60000;

// Filter indstillinger
const float FILTER_ALPHA = 0.01;

// Fast offset baseret på multimeter måling
const float CURRENT_SENSOR_OFFSET = 2.502; // Opdateret offset

// ===== GLOBALE VARIABLER =====
float filteredCurrent = 0.0;
float targetCurrent = 0.0;
float smoothedTargetCurrent = 0.0;
int currentPWM = 0;

unsigned long lastCurrentMeasurementTime = 0;
unsigned long lastPrintTime = 0;
unsigned long rampStartTime = 0;
unsigned long errorTime = 0;

enum SystemState {
  STATE_IDLE,
  STATE_RAMP_UP,
  STATE_CHARGING,
  STATE_ERROR
};

SystemState currentState = STATE_IDLE;

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);
  
  Serial.begin(115200);
  Serial.println("Battery Charger Controller Starting...");
  
  analogReference(DEFAULT);
  
  // Initialiser med den aktuelle target
  int potValue = analogRead(POTENTIOMETER_PIN);
  targetCurrent = MIN_CURRENT_SETTING + (MAX_CURRENT_SETTING - MIN_CURRENT_SETTING) * (potValue / 1023.0);
  smoothedTargetCurrent = 0; // Start fra 0
}

void loop() {
  unsigned long currentMillis = millis();

  // Opdater strømmåling med meget langsomt filter
  if (currentMillis - lastCurrentMeasurementTime >= CURRENT_MEASUREMENT_INTERVAL) {
    int adcValue = analogRead(CURRENT_SENSOR_PIN);
    float voltage = (adcValue / 1023.0) * 5.0;
    float current = (voltage - CURRENT_SENSOR_OFFSET) / CURRENT_SENSOR_SENSITIVITY;
    
    // Meget langsomt low-pass filter
    filteredCurrent = (FILTER_ALPHA * current) + ((1 - FILTER_ALPHA) * filteredCurrent);
    
    lastCurrentMeasurementTime = currentMillis;
  }

  // Læs potentiometer og beregn target strøm
  int potValue = analogRead(POTENTIOMETER_PIN);
  targetCurrent = MIN_CURRENT_SETTING + (MAX_CURRENT_SETTING - MIN_CURRENT_SETTING) * (potValue / 1023.0);

  // Tilstandsmaskine
  switch (currentState) {
    case STATE_IDLE:
      // Start ramp op
      currentState = STATE_RAMP_UP;
      rampStartTime = currentMillis;
      smoothedTargetCurrent = 0; // Start fra 0
      Serial.println("Starting ramp-up");
      break;
      
    case STATE_RAMP_UP:
      {
        // Beregn ramp fremskridt (0.0 til 1.0)
        float rampProgress = min(1.0, (float)(currentMillis - rampStartTime) / RAMP_TIME_MS);
        
        // Beregn glat target baseret på ramp fremskridt
        smoothedTargetCurrent = targetCurrent * rampProgress;
        
        // Hvis ramp er fuldført, gå til charging tilstand
        if (rampProgress >= 1.0) {
          currentState = STATE_CHARGING;
          Serial.println("Ramp complete, entering charging state");
        }
      }
      break;
      
    case STATE_CHARGING:
      // Brug den fulde target strøm
      smoothedTargetCurrent = targetCurrent;
      
      // Tjek for reverse strøm
      if (filteredCurrent < REVERSE_CURRENT_THRESHOLD) {
        Serial.println("Reverse current detected! Entering error state.");
        currentState = STATE_ERROR;
        errorTime = currentMillis;
        analogWrite(PWM_PIN, 0);
        currentPWM = 0;
      }
      break;
      
    case STATE_ERROR:
      // Vent i 60 sekunder før genstart
      if (currentMillis - errorTime >= RETRY_DELAY_MS) {
        Serial.println("Retrying...");
        currentState = STATE_IDLE;
      }
      break;
  }

  // Regulering (kun hvis ikke i fejltilstand)
  if (currentState != STATE_ERROR) {
    // Simpel P-regulator
    float error = smoothedTargetCurrent - filteredCurrent;
    
    // Regulator med højere forstærkning
    int pwmChange = error * 10.0; // Forøget forstærkning
    
    // Begræns PWM ændring
    pwmChange = constrain(pwmChange, -5, 5);
    currentPWM = constrain(currentPWM + pwmChange, 0, 255);
    
    analogWrite(PWM_PIN, currentPWM);
  }

  // Debug output
  if (currentMillis - lastPrintTime >= PRINT_INTERVAL) {
    Serial.print("State: ");
    switch (currentState) {
      case STATE_IDLE: Serial.print("IDLE"); break;
      case STATE_RAMP_UP: Serial.print("RAMP_UP"); break;
      case STATE_CHARGING: Serial.print("CHARGING"); break;
      case STATE_ERROR: Serial.print("ERROR"); break;
    }
    
    // Tilføj ramp fremskridt til output
    float rampProgress = 0.0;
    if (currentState == STATE_RAMP_UP) {
      rampProgress = min(1.0, (float)(currentMillis - rampStartTime) / RAMP_TIME_MS);
    }
    
    Serial.print(" | Ramp: ");
    Serial.print(rampProgress * 100, 1);
    Serial.print("%");
    Serial.print(" | Pot: ");
    Serial.print(potValue);
    Serial.print(" | Target: ");
    Serial.print(targetCurrent, 2);
    Serial.print("A | SmoothedTarget: ");
    Serial.print(smoothedTargetCurrent, 2);
    Serial.print("A | Actual: ");
    Serial.print(filteredCurrent, 2);
    Serial.print("A | PWM: ");
    Serial.println(currentPWM);
    
    lastPrintTime = currentMillis;
  }

  delay(1);
}