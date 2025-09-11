#include <Arduino.h>
#include <EEPROM.h>

// ------------------- Pin constants -------------------
const int ledPin       = LED_BUILTIN;
const int currentPin   = A1; // ACS758
const int carBatPin    = A2;
const int liefpoBatPin = A3;
const int pwmPin       = 9;

// ------------------- ADC / sensor constants -------------------
constexpr float Vref = 5.0;
constexpr float scaleDirect = Vref / 1023.0;

// Voltage dividers for batteries (carBat & LiFePO4)
constexpr float R1 = 8200.0;
constexpr float R2 = 2200.0;
constexpr float scaleDivider = (Vref / 1023.0) * ((R1 + R2) / R2);

// ACS758 parameters
constexpr float acsOffset = 2.50; // 0A
constexpr float acsSens   = 0.040; // 40 mV per 1A

// ------------------- Safety thresholds -------------------
constexpr float BAT_DIFF_MAX   = 0.2;   // V, carBat - LiFePO4
constexpr float LIFEPo_MAX     = 14.0;  // V
constexpr float LIFEPo_RECOVER = 13.5;  // V, resume charging
constexpr float ACS_MIN        = -1.0;  // A, stop if current goes negative

// ------------------- PWM control constants -------------------
constexpr float SETPOINT_A     = 20.0;  // target current
constexpr float PWM_STEP_FAST  = 0.05;  // 5% PWM step
constexpr float PWM_STEP_SLOW  = 0.02;  // 2% PWM step
constexpr int   PWM_MAX        = 255;
constexpr int   PWM_MIN        = 0;

// ------------------- Timing -------------------
unsigned long lastAdcTime   = 0;
unsigned long lastPrintTime = 0;
constexpr unsigned long adcInterval   = 100;   // 10 Hz
constexpr unsigned long printInterval = 1000;  // 1 Hz

// ------------------- Filtered ADC -------------------
uint16_t filtCurrent   = 0;
uint16_t filtCarBat    = 0;
uint16_t filtLiFePO4   = 0;

// ------------------- PWM state -------------------
int pwmOut = 0;

// ------------------- Charge permission flag -------------------
bool doCharge = true;   // true = charging allowed

// EEPROM address to store flag
constexpr int EEPROM_ADDR_FLAG = 0;

// ------------------- Function prototypes -------------------
void sampleAdc();
uint16_t filteredUpdate(uint16_t oldVal, int newVal);
void controlPWM(float measuredAmp);
void printStatus(float measuredAmp);
void batterySafetyCheck(float carVolt, float lifepoVolt, float measuredAmp);
bool readEepromFlag();
void writeEepromFlag(bool flag);

// ------------------- Setup -------------------
void setup() {
    pinMode(ledPin, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    analogWrite(pwmPin, pwmOut);
    Serial.begin(115200);

    // Read flag from EEPROM (0xFF = allow charging)
    doCharge = readEepromFlag();
}

// ------------------- Main loop -------------------
void loop() {
    unsigned long now = millis();

    // ADC sampling 10 Hz
    if (now - lastAdcTime >= adcInterval) {
        lastAdcTime = now;
        sampleAdc();
    }

    // Control + safety check + print 1 Hz
    if (now - lastPrintTime >= printInterval) {
        lastPrintTime = now;

        // Convert ADCs to voltage / current
        float carVolt     = filtCarBat * scaleDivider;
        float lifepoVolt  = filtLiFePO4 * scaleDivider;
        float currentVolt = filtCurrent * scaleDirect;
        float measuredAmp = (currentVolt - acsOffset) / acsSens;

        // Safety check and update doCharge
        batterySafetyCheck(carVolt, lifepoVolt, measuredAmp);

        // Only control PWM if allowed
        if (doCharge) controlPWM(measuredAmp);

        // Print status
        printStatus(measuredAmp);
        digitalWrite(ledPin, !digitalRead(ledPin));
    }
}

// ------------------- Functions -------------------

// Sample all ADC channels and apply simple filter
void sampleAdc() {
    filtCurrent   = filteredUpdate(filtCurrent,   analogRead(currentPin));
    filtCarBat    = filteredUpdate(filtCarBat,    analogRead(carBatPin));
    filtLiFePO4   = filteredUpdate(filtLiFePO4,   analogRead(liefpoBatPin));
}

uint16_t filteredUpdate(uint16_t oldVal, int newVal) {
    return (uint16_t)(oldVal * 0.9 + newVal * 0.1);
}

// Simple incremental PWM control
void controlPWM(float measuredAmp) {
    int step = 0;
    if (measuredAmp < SETPOINT_A * 0.8) {
        step = (int)(PWM_MAX * PWM_STEP_FAST); // +5%
    } else if (measuredAmp < SETPOINT_A) {
        step = (int)(PWM_MAX * PWM_STEP_SLOW); // +2%
    } else {
        step = -(int)(PWM_MAX * PWM_STEP_SLOW); // -2%
    }

    pwmOut += step;
    if (pwmOut > PWM_MAX) pwmOut = PWM_MAX;
    if (pwmOut < PWM_MIN) pwmOut = PWM_MIN;

    analogWrite(pwmPin, pwmOut);
}

// Print current & PWM
void printStatus(float measuredAmp) {
    Serial.print("Current: ");
    Serial.print(measuredAmp, 2);
    Serial.print(" A   PWM: 0x");
    Serial.print(pwmOut, HEX);
    Serial.print(" (");
    Serial.print(pwmOut);
    Serial.println(")");
    Serial.print("Charge allowed: ");
    Serial.println(doCharge ? "YES" : "NO");
}

// ------------------- Battery safety check -------------------
void batterySafetyCheck(float carVolt, float lifepoVolt, float measuredAmp) {
    bool prevFlag = doCharge;

    if ((carVolt - lifepoVolt) < BAT_DIFF_MAX) {
        doCharge = false;  // stop charging
    } else if (lifepoVolt > LIFEPo_MAX) {
        doCharge = false;  // stop charging
    } else if (!doCharge && lifepoVolt < LIFEPo_RECOVER) {
        doCharge = true;   // resume charging
    }

    // Stop if current goes negative beyond threshold
    if (measuredAmp < ACS_MIN) doCharge = false;

    // Write flag to EEPROM if changed
    if (doCharge != prevFlag) writeEepromFlag(doCharge);
}

// ------------------- EEPROM helpers -------------------
bool readEepromFlag() {
    byte val = EEPROM.read(EEPROM_ADDR_FLAG);
    return (val == 0xFF);
}

void writeEepromFlag(bool flag) {
    EEPROM.update(EEPROM_ADDR_FLAG, flag ? 0xFF : 0x00);
}
