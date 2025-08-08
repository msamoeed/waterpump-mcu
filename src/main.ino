/*
 * ESP32 Main Receiver - WITH INTEGRATED RELAY CONTROL AND ENHANCED FEATURES
 * PIN CONNECTIONS FOR ESP32 WROOM 32 DevKit V1:
 * - D4  → NRF24L01 CE
 * - D5  → NRF24L01 CSN
 * - D18 → NRF24L01 SCK
 * - D19 → NRF24L01 MISO
 * - D23 → NRF24L01 MOSI
 * - D21 → OLED SDA
 * - D22 → OLED SCL
 * - D32 → Rotary Encoder CLK (interrupt capable)
 * - D25 → Rotary Encoder DT
 * - D15 → Rotary Encoder SW
 * - D26 → Buzzer
 * - D2  → Status LED (built-in) - kept for backup
 * - D13 → RGB LED Red
 * - D14 → RGB LED Green  
 * - D27 → RGB LED Blue
 * 
 * NEW PINS FOR INTEGRATED RELAY:
 * - D33 → Relay Control (HIGH = Pump ON)
 * - D35 → Current Sensor (ACS712) - Optional for power monitoring
 * - D34 → AC Voltage Detection (Optional) - for safety
 * 
 * RGB LED STATUS CODES:
 * 🟢 Solid Green: Both transmitters OK, system normal
 * 🔴 Solid Red: Both transmitters offline
 * 🔵 Solid Blue: Pump protection active (voltage/current issue)
 * 🟡 Pulsing Yellow: One transmitter offline
 * 🟣 Pulsing Purple: Sensor defective/unplugged
 * 🌈 Rotating Colors: Water supply active (Red→Green→Blue cycle)
 * ⚫ Off: System initializing
 */

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <RF24.h>
#include <EEPROM.h>
#include <Update.h>
#include <WebServer.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsClient.h>

// ========================================
// CONFIGURATION
// ========================================
// WiFi Configuration - Update with your network details
const char WIFI_SSID[] = "Ahad 2.4Ghz";
const char WIFI_PASS[] = "Artifact";

// Backend Configuration - Update with your backend server details
const char BACKEND_HOST[] = "205.189.160.7";  // Updated to your server IP
const int BACKEND_PORT = 3002;  // Updated to match your server port
const char DEVICE_ID[] = "esp32_controller_001";
const char API_BASE_URL[] = "/api/v1";

// WebSocket Configuration
const char WS_HOST[] = "205.189.160.7";
const int WS_PORT = 3002;
const char WS_URL[] = "/";

// OTA Update Configuration
const char GITHUB_REPO[] = "msamoeed/waterpump-backend";
const char GITHUB_TOKEN[] = ""; // Optional: Add GitHub token for private repos

// ========================================
// PIN DEFINITIONS
// ========================================
#define CE_PIN 4
#define CSN_PIN 5
#define ENCODER_CLK 32
#define ENCODER_DT 25      
#define ENCODER_SW 15      
#define BUZZER_PIN 26
#define STATUS_LED_PIN 2

// RGB LED pins
#define RGB_RED_PIN 13
#define RGB_GREEN_PIN 14
#define RGB_BLUE_PIN 27

// NEW: Integrated Pump Control Pins
#define RELAY_PIN 33              // Relay control pin
#define CURRENT_SENSOR_PIN -1     // ACS712 current sensor (disabled for now)
#define VOLTAGE_DETECT_PIN 34     // AC voltage detection (optional)

// ========================================
// PUMP CONTROL CONSTANTS
// ========================================
#define MAX_PUMP_RUNTIME_HOURS 1        // Maximum continuous runtime
#define PUMP_PROTECTION_DELAY 300000    // 5 min delay after protection trip
#define CURRENT_SAMPLE_COUNT 10         // Samples for current averaging
#define NORMAL_PUMP_CURRENT_MIN 2.0     // Minimum normal operating current (A)
#define NORMAL_PUMP_CURRENT_MAX 8.0     // Maximum normal operating current (A)
#define OVERCURRENT_THRESHOLD 10.0      // Overcurrent protection threshold (A)

// ========================================
// RGB LED DEFINITIONS (Updated)
// ========================================
enum RGBState {
  RGB_OFF,
  RGB_SOLID_GREEN,      // Both transmitters OK
  RGB_SOLID_RED,        // Both transmitters offline
  RGB_SOLID_BLUE,       // Pump protection active
  RGB_PULSE_YELLOW,     // One transmitter offline
  RGB_PULSE_PURPLE,     // Sensor defective
  RGB_ROTATING_SUPPLY   // Water supply active
};

struct RGBColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

const RGBColor COLOR_OFF = {0, 0, 0};
const RGBColor COLOR_RED = {255, 0, 0};
const RGBColor COLOR_GREEN = {0, 255, 0};
const RGBColor COLOR_BLUE = {0, 0, 255};
const RGBColor COLOR_YELLOW = {255, 255, 0};
const RGBColor COLOR_PURPLE = {255, 0, 255};

// ========================================
// HARDWARE OBJECTS
// ========================================
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
RF24 radio(CE_PIN, CSN_PIN);
HTTPClient http;
WebSocketsClient webSocket;

// ========================================
// RADIO ADDRESSES
// ========================================
const uint64_t rxAddr1 = 0xF0F0F0F0E1LL;  // Ground tank
const uint64_t rxAddr2 = 0xF0F0F0F0E2LL;  // Roof tank

// ========================================
// DATA STRUCTURES
// ========================================
struct WaterLevelData {
  float levelPercent;
  float levelInches;
  bool alarmActive;
  bool waterSupplyOn;
  uint8_t tankID;
};

// NEW: Integrated Pump Status Structure
struct IntegratedPumpStatus {
  bool pumpRunning;
  bool manualOverride;
  float currentAmps;
  float powerWatts;
  float dailyConsumption;
  float hourlyConsumption;
  unsigned long runTimeMinutes;
  unsigned long totalRunTimeHours;
  bool protectionActive;
  bool overCurrentProtection;
  bool overTimeProtection;
  unsigned long protectionStartTime;
};

// ========================================
// ENHANCED MENU SYSTEM
// ========================================
enum PumpMenuLevel {
  PUMP_STATUS_VIEW,     // Level 0: Show pump status
  PUMP_MODE_SELECT,     // Level 1: Auto/Manual mode selection
  PUMP_MANUAL_CONTROL   // Level 2: Enhanced manual pump control options
};

// Menu constants
#define MANUAL_CONTROL_OPTIONS 6  // ON, OFF, 10", 20", 35", BACK
#define MODE_SELECT_OPTIONS 3     // AUTO, MANUAL, BACK TO MAIN

// ========================================
// EEPROM CONFIGURATION
// ========================================
#define EEPROM_SIZE 512
#define EEPROM_TARGET_STATE_ADDR 0
#define EEPROM_SYSTEM_STATE_ADDR 100
#define EEPROM_MAGIC_NUMBER 0xABCD1234

// Persistent Data Structures
struct PersistentTargetState {
  uint32_t magicNumber;           
  bool wasActive;                 
  float targetLevel;              
  char description[20];           
  unsigned long pausedTime;       
  float levelWhenPaused;         
  bool autoResumeEnabled;        
  bool wasPowerFailure;          
  uint32_t checksum;             
};

struct PersistentSystemState {
  bool pumpWasRunning;           
  bool autoModeWasEnabled;       
  float lastGroundLevel;         
  float lastRoofLevel;          
  unsigned long lastUpdateTime;  
  uint32_t bootCount;           
  uint32_t checksum;            
};

// ========================================
// GLOBAL VARIABLES
// ========================================
WaterLevelData groundTankData, roofTankData;
IntegratedPumpStatus pumpStatus;

bool groundTankConnected = false;
bool roofTankConnected = false;
bool groundSensorWorking = false;
bool roofSensorWorking = false;
bool groundAlarmActive = false;
bool roofAlarmActive = false;
bool waterSupplyOn = false;
bool forceDisplayUpdate = false;
bool supplyFromGroundTank = false, supplyFromRoofTank = false;

// Menu variables
PumpMenuLevel pumpMenuLevel = PUMP_STATUS_VIEW;
int pumpMenuSelection = 0;

// Pump control variables
bool pumpState = false;
bool autoControlEnabled = true;
bool manualPumpControl = false;
bool pumpProtectionActive = false;

// Target-based pump control variables
bool targetModeActive = false;
float currentTargetLevel = 0.0;
String targetDescription = "";
unsigned long targetStartTime = 0;

// Recovery variables
PersistentTargetState persistentTarget;
PersistentSystemState persistentSystem;
bool needsPowerRecovery = false;
unsigned long powerRecoveryStartTime = 0;
unsigned long lastEEPROMSave = 0;

// Recovery timing constants
#define POWER_RECOVERY_DELAY 10000      
#define EEPROM_SAVE_INTERVAL 30000      
#define SIGNIFICANT_LEVEL_CHANGE 5.0    
#define MAJOR_LEVEL_CHANGE 10.0

// Connection tracking
unsigned long lastGroundTankTime = 0;
unsigned long lastRoofTankTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastLedToggle = 0;
unsigned long lastCloudUpdate = 0;

// Pump monitoring variables
unsigned long pumpStartTime = 0;
unsigned long lastCurrentReading = 0;
unsigned long lastPowerCalculation = 0;
unsigned long dailyResetTime = 0;
unsigned long hourlyResetTime = 0;
float currentReadings[CURRENT_SAMPLE_COUNT];
int currentReadingIndex = 0;
bool currentReadingsValid = false;

// RGB LED variables
RGBState currentRGBState = RGB_OFF;
unsigned long lastRGBUpdate = 0;
unsigned long rgbStateStartTime = 0;
uint8_t currentBrightness = 255;

// Encoder variables
volatile int encoderPos = 0;
volatile bool lastClkState = HIGH;
int lastEncoderPos = 0;
int selectedMode = 0;
int menuScrollOffset = 0;
int currentView = -1;
bool encoderWorking = false;

bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
bool buttonPressed = false;

// Audio System
enum AudioState { AUDIO_IDLE, AUDIO_BEEP, AUDIO_ALERT };
AudioState audioState = AUDIO_IDLE;
unsigned long audioStartTime = 0;
unsigned long audioStepTime = 0;
uint8_t audioStep = 0;
uint8_t audioDuration = 0;
uint8_t audioRepeat = 0;
bool buzzerMuted = false;

// Message System
bool messageActive = false;
unsigned long messageStartTime = 0;
char currentMessage[32] = "";
bool returnToMenuAfterMessage = false;

// Water Level Thresholds
#define GROUND_TANK_LOWER_THRESHOLD 15.0
#define GROUND_TANK_UPPER_THRESHOLD 30.0
#define ROOF_TANK_LOWER_THRESHOLD 20.0
#define ROOF_TANK_UPPER_THRESHOLD 80.0

// Timing constants
#define TOTAL_MODES 8
#define MENU_ITEMS_VISIBLE 5
#define CONNECTION_TIMEOUT 15000
#define DISPLAY_UPDATE_INTERVAL 50
#define CLOUD_UPDATE_INTERVAL 30000
#define DEBOUNCE_DELAY 50
#define MESSAGE_DISPLAY_TIME 2000
#define RGB_UPDATE_INTERVAL 50
#define RGB_PULSE_INTERVAL 1000
#define RGB_ROTATION_INTERVAL 800
#define CURRENT_READING_INTERVAL 1000
#define POWER_CALC_INTERVAL 5000

// Backend communication
bool needsBackendUpdate = false;
unsigned long lastBackendUpdate = 0;
const unsigned long BACKEND_UPDATE_INTERVAL = 30000;
bool backendConnected = false;
unsigned long lastBackendResponse = 0;
int backendErrorCount = 0;

// Command checking variables
unsigned long lastCommandCheck = 0;
const unsigned long COMMAND_CHECK_INTERVAL = 5000;  // Check every 5 seconds
unsigned long lastManualCommandTime = 0;
const unsigned long MANUAL_COMMAND_OVERRIDE_DURATION = 30000;  // 30 seconds override

// ========================================
// OTA UPDATE VARIABLES
// ========================================
bool otaUpdateInProgress = false;
String otaDownloadUrl = "";
String otaVersion = "";
unsigned long otaStartTime = 0;
unsigned long lastOtaProgress = 0;
int otaProgress = 0;
String otaStatus = "";
bool otaUpdateAvailable = false;

// ========================================
// ENCODER INTERRUPT
// ========================================
void IRAM_ATTR updateEncoder() {
  bool clkState = digitalRead(ENCODER_CLK);
  bool dtState = digitalRead(ENCODER_DT);
  
  if (clkState != lastClkState) {
    if (dtState != clkState) {
      encoderPos++;
    } else {
      encoderPos--;
    }
    lastClkState = clkState;
  }
}

// ========================================
// EEPROM UTILITY FUNCTIONS
// ========================================
uint32_t calculateChecksum(void* data, size_t length) {
  uint32_t checksum = 0;
  uint8_t* bytes = (uint8_t*)data;
  for (size_t i = 0; i < length - sizeof(uint32_t); i++) {
    checksum += bytes[i];
  }
  return checksum;
}

void saveTargetStateToEEPROM(bool isPowerFailure = false) {
  if (!targetModeActive && !isPowerFailure) {
    return;
  }
  
  persistentTarget.magicNumber = EEPROM_MAGIC_NUMBER;
  persistentTarget.wasActive = targetModeActive;
  persistentTarget.targetLevel = currentTargetLevel;
  strncpy(persistentTarget.description, targetDescription.c_str(), 19);
  persistentTarget.description[19] = '\0';
  persistentTarget.pausedTime = millis();
  persistentTarget.levelWhenPaused = roofTankConnected ? roofTankData.levelInches : 0.0;
  persistentTarget.autoResumeEnabled = true;
  persistentTarget.wasPowerFailure = isPowerFailure;
  persistentTarget.checksum = calculateChecksum(&persistentTarget, sizeof(persistentTarget));
  
  EEPROM.put(EEPROM_TARGET_STATE_ADDR, persistentTarget);
  EEPROM.commit();
  
  Serial.println("Target state saved to EEPROM");
}

void saveSystemStateToEEPROM() {
  persistentSystem.pumpWasRunning = pumpState;
  persistentSystem.autoModeWasEnabled = autoControlEnabled;
  persistentSystem.lastGroundLevel = groundTankConnected ? groundTankData.levelPercent : 0.0;
  persistentSystem.lastRoofLevel = roofTankConnected ? roofTankData.levelInches : 0.0;
  persistentSystem.lastUpdateTime = millis();
  persistentSystem.bootCount++;
  persistentSystem.checksum = calculateChecksum(&persistentSystem, sizeof(persistentSystem));
  
  EEPROM.put(EEPROM_SYSTEM_STATE_ADDR, persistentSystem);
  EEPROM.commit();
  
  lastEEPROMSave = millis();
}

bool loadStateFromEEPROM() {
  EEPROM.get(EEPROM_TARGET_STATE_ADDR, persistentTarget);
  EEPROM.get(EEPROM_SYSTEM_STATE_ADDR, persistentSystem);
  
  if (persistentTarget.magicNumber != EEPROM_MAGIC_NUMBER) {
    Serial.println("No valid target state in EEPROM");
    return false;
  }
  
  uint32_t expectedTargetChecksum = persistentTarget.checksum;
  uint32_t actualTargetChecksum = calculateChecksum(&persistentTarget, sizeof(persistentTarget));
  
  if (expectedTargetChecksum != actualTargetChecksum) {
    Serial.println("Target state EEPROM data corrupted");
    return false;
  }
  
  Serial.println("Valid EEPROM target state found");
  return true;
}

void clearEEPROMState() {
  persistentTarget.magicNumber = 0;
  persistentTarget.wasActive = false;
  EEPROM.put(EEPROM_TARGET_STATE_ADDR, persistentTarget);
  EEPROM.commit();
  Serial.println("EEPROM target state cleared");
}

void clearTargetState() {
  targetModeActive = false;
  currentTargetLevel = 0.0;
  targetDescription = "";
  Serial.println("Target state cleared");
}

// ========================================
// POWER RECOVERY FUNCTIONS
// ========================================
bool canRecoverFromPower() {
  if (!persistentTarget.wasActive || !persistentTarget.autoResumeEnabled) {
    Serial.println("Cannot recover: Target was not active or auto-resume disabled");
    return false;
  }
  
  if (!roofTankConnected || !roofSensorWorking || 
      !groundTankConnected || !groundSensorWorking) {
    Serial.println("Cannot recover: Sensors not ready");
    return false;
  }
  
  if (pumpStatus.protectionActive) {
    Serial.println("Cannot recover: Protection system active");
    return false;
  }
  
  if (groundTankData.levelPercent < GROUND_TANK_LOWER_THRESHOLD) {
    Serial.println("Cannot recover: Ground tank too low");
    return false;
  }
  
  Serial.println("Power recovery validation passed - safe to recover");
  return true;
}

bool attemptPowerRecovery() {
  if (!canRecoverFromPower()) {
    clearEEPROMState();
    return false;
  }
  
  float currentLevel = roofTankData.levelInches;
  float targetLevel = persistentTarget.targetLevel;
  float previousLevel = persistentTarget.levelWhenPaused;
  
  if (currentLevel >= targetLevel) {
    Serial.println("Target already achieved during power outage");
    showMessageNonBlocking("TARGET COMPLETED!");
    playBeepNonBlocking(200);
    clearEEPROMState();
    return false;
  }
  
  float levelChange = currentLevel - previousLevel;
  
  if (abs(levelChange) >= MAJOR_LEVEL_CHANGE) {
    char message[50];
    if (levelChange > 0) {
      sprintf(message, "LEVEL ROSE +%.1f\" DURING OUTAGE", levelChange);
    } else {
      sprintf(message, "LEVEL DROPPED %.1f\" DURING OUTAGE", abs(levelChange));
    }
    showMessageNonBlocking(message);
    delay(3000);
  }
  
  float remainingToTarget = targetLevel - currentLevel;
  if (remainingToTarget < 1.0) {
    Serial.println("Less than 1 inch remaining - target nearly complete");
    showMessageNonBlocking("TARGET NEARLY DONE!");
    clearEEPROMState();
    return false;
  }
  
  targetModeActive = true;
  currentTargetLevel = persistentTarget.targetLevel;
  targetDescription = String(persistentTarget.description);
  manualPumpControl = true;
  autoControlEnabled = false;
  
  char reason[100];
  sprintf(reason, "Power recovery: %s (%.1f\" → %.1f\")", 
          persistentTarget.description, 
          currentLevel,
          targetLevel);
  
  setPumpState(true, reason);
  
  char message[50];
  sprintf(message, "RECOVERED: %s", persistentTarget.description);
  showMessageNonBlocking(message);
  playBeepNonBlocking(200);
  
  Serial.println("Power recovery successful!");
  return true;
}

void handlePowerRecovery() {
  if (!needsPowerRecovery) return;
  
  unsigned long currentTime = millis();
  
  if (currentTime - powerRecoveryStartTime < POWER_RECOVERY_DELAY) {
    static unsigned long lastCountdown = 0;
    if (currentTime - lastCountdown > 1000) {
      int remaining = (POWER_RECOVERY_DELAY - (currentTime - powerRecoveryStartTime)) / 1000;
      char message[30];
      sprintf(message, "STABILIZING... %ds", remaining);
      showMessageNonBlocking(message);
      lastCountdown = currentTime;
    }
    return;
  }
  
  needsPowerRecovery = false;
  
  if (!attemptPowerRecovery()) {
    Serial.println("Power recovery failed or not needed");
    showMessageNonBlocking("RECOVERY FAILED");
    playBeepNonBlocking(50);
  }
}

// ========================================
// INTEGRATED PUMP CONTROL FUNCTIONS
// ========================================
void setupPumpControl() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Ensure pump starts OFF
  
  // Initialize pump status
  pumpStatus = {false, false, 0.0, 0.0, 0.0, 0.0, 0, 0, false, false, false, 0};
  
  // Setup analog pins for monitoring (if available)
  #ifdef CURRENT_SENSOR_PIN
    pinMode(CURRENT_SENSOR_PIN, INPUT);
  #endif
  
  #ifdef VOLTAGE_DETECT_PIN
    pinMode(VOLTAGE_DETECT_PIN, INPUT);
  #endif
  
  // Initialize timing
  dailyResetTime = millis();
  hourlyResetTime = millis();
  
  Serial.println("Integrated pump control initialized");
}

void startTargetPumpOperation(float targetInches, const char* description) {
  if (pumpStatus.protectionActive) {
    showMessageWithAutoReturn("PROTECTION ACTIVE");
    return;
  }
  
  if (!roofTankConnected || !roofSensorWorking) {
    showMessageWithAutoReturn("ROOF SENSOR ERROR");
    return;
  }
  
  if (!groundTankConnected || !groundSensorWorking) {
    showMessageWithAutoReturn("GROUND SENSOR ERROR");
    return;
  }
  
  if (roofTankData.levelInches >= targetInches) {
    showMessageWithAutoReturn("TARGET ALREADY MET");
    return;
  }
  
  if (groundTankData.levelPercent < GROUND_TANK_LOWER_THRESHOLD) {
    showMessageWithAutoReturn("GROUND TANK TOO LOW");
    return;
  }
  
  clearEEPROMState();
  
  targetModeActive = true;
  currentTargetLevel = targetInches;
  targetDescription = String(description);
  
  char reason[50];
  sprintf(reason, "Target mode: %s", description);
  setPumpState(true, reason);
  
  char message[30];
  sprintf(message, "TARGET: %s", description);
  showMessageWithAutoReturn(message);
}

float readPumpCurrent() {
  // Check if current sensor pin is actually connected and working
  if (CURRENT_SENSOR_PIN != -1) {
    // ACS712 30A: 2.5V = 0A, 66mV per Amp
    float voltage = (analogRead(CURRENT_SENSOR_PIN) * 3.3) / 4095.0;
    float current = (voltage - 1.65) / 0.066;  // ACS712-30A formula
    
    // Add some validation to prevent false readings
    if (current < 0 || current > 50) {  // Invalid reading
      return pumpState ? 5.0 : 0.0;  // Fallback to estimated current
    }
    
    return abs(current);
  } else {
    // If no current sensor, return estimated current based on pump state
    return pumpState ? 5.0 : 0.0;  // Estimate 5A when running
  }
}

void updateCurrentReading() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastCurrentReading >= CURRENT_READING_INTERVAL) {
    float newCurrent = readPumpCurrent();
    
    // Add to circular buffer
    currentReadings[currentReadingIndex] = newCurrent;
    currentReadingIndex = (currentReadingIndex + 1) % CURRENT_SAMPLE_COUNT;
    
    // Calculate average
    float sum = 0;
    for (int i = 0; i < CURRENT_SAMPLE_COUNT; i++) {
      sum += currentReadings[i];
    }
    pumpStatus.currentAmps = sum / CURRENT_SAMPLE_COUNT;
    
    // Mark readings as valid after first full cycle
    if (currentReadingIndex == 0) currentReadingsValid = true;
    
    lastCurrentReading = currentTime;
  }
}

void updatePowerConsumption() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastPowerCalculation >= POWER_CALC_INTERVAL) {
    // Estimate power (P = V * I, assuming 230V AC)
    pumpStatus.powerWatts = pumpStatus.currentAmps * 230.0;
    
    // Update consumption counters
    if (pumpState && currentReadingsValid) {
      float hours = POWER_CALC_INTERVAL / 3600000.0;  // Convert ms to hours
      float kwh = (pumpStatus.powerWatts / 1000.0) * hours;
      
      pumpStatus.dailyConsumption += kwh;
      pumpStatus.hourlyConsumption += kwh;
    }
    
    // Reset hourly counter every hour
    if (currentTime - hourlyResetTime >= 3600000) {
      pumpStatus.hourlyConsumption = 0;
      hourlyResetTime = currentTime;
    }
    
    // Reset daily counter every 24 hours
    if (currentTime - dailyResetTime >= 86400000) {
      pumpStatus.dailyConsumption = 0;
      dailyResetTime = currentTime;
    }
    
    lastPowerCalculation = currentTime;
  }
}

void checkPumpProtection() {
  unsigned long currentTime = millis();
  
  // Check overcurrent protection
  if (currentReadingsValid && pumpStatus.currentAmps > OVERCURRENT_THRESHOLD) {
    if (!pumpStatus.overCurrentProtection) {
      pumpStatus.overCurrentProtection = true;
      pumpStatus.protectionActive = true;
      pumpStatus.protectionStartTime = currentTime;
      setPumpState(false, "Overcurrent protection");
      playAlertSoundNonBlocking();
      Serial.println("OVERCURRENT PROTECTION ACTIVATED");
    }
  }
  
  // Check runtime protection
  if (pumpState && (currentTime - pumpStartTime) > (MAX_PUMP_RUNTIME_HOURS * 3600000UL)) {
    if (!pumpStatus.overTimeProtection) {
      pumpStatus.overTimeProtection = true;
      pumpStatus.protectionActive = true;
      pumpStatus.protectionStartTime = currentTime;
      setPumpState(false, "Maximum runtime exceeded");
      playAlertSoundNonBlocking();
      Serial.println("RUNTIME PROTECTION ACTIVATED");
    }
  }
  
  // Check if protection can be reset (after delay)
  if (pumpStatus.protectionActive && 
      (currentTime - pumpStatus.protectionStartTime) > PUMP_PROTECTION_DELAY) {
    
    // Reset overcurrent protection if current is now normal
    if (pumpStatus.overCurrentProtection && 
        pumpStatus.currentAmps < (OVERCURRENT_THRESHOLD * 0.8)) {
      pumpStatus.overCurrentProtection = false;
      Serial.println("Overcurrent protection reset");
    }
    
    // Reset runtime protection
    if (pumpStatus.overTimeProtection) {
      pumpStatus.overTimeProtection = false;
      Serial.println("Runtime protection reset");
    }
    
    // Clear protection active flag if all protections are clear
    if (!pumpStatus.overCurrentProtection && !pumpStatus.overTimeProtection) {
      pumpStatus.protectionActive = false;
      playBeepNonBlocking(100);
      Serial.println("All protections cleared");
    }
  }
}

void setPumpState(bool newState, const char* reason) {
  Serial.print("setPumpState called - newState: ");
  Serial.print(newState ? "TRUE" : "FALSE");
  Serial.print(", current pumpState: ");
  Serial.print(pumpState ? "TRUE" : "FALSE");
  Serial.print(", reason: ");
  Serial.println(reason);
  
  if (pumpStatus.protectionActive && newState) {
    Serial.println("Pump start blocked by protection system");
    return;
  }
  
  if (pumpState != newState) {
    Serial.println("Pump state change detected - updating...");
    pumpState = newState;
    pumpStatus.pumpRunning = newState;
    digitalWrite(RELAY_PIN, newState ? HIGH : LOW);
    
    Serial.print("Relay pin set to: ");
    Serial.println(newState ? "HIGH" : "LOW");
    
    if (newState) {
      pumpStartTime = millis();
      if (targetModeActive) {
        targetStartTime = millis();
        saveTargetStateToEEPROM(false);
        Serial.print("Pump STARTED - Target Mode: ");
        Serial.print(targetDescription);
        Serial.print(" (");
        Serial.print(currentTargetLevel);
        Serial.println(" inches)");
      } else {
        Serial.print("Pump STARTED - Reason: ");
        Serial.println(reason);
      }
    } else {
      unsigned long runtime = (millis() - pumpStartTime) / 60000;
      pumpStatus.runTimeMinutes += runtime;
      pumpStatus.totalRunTimeHours = pumpStatus.runTimeMinutes / 60;
      
      if (targetModeActive) {
        Serial.print("Pump STOPPED - Target Mode: ");
        Serial.println(reason);
        
        if (strstr(reason, "failure") != NULL || 
            strstr(reason, "Sensor") != NULL ||
            strstr(reason, "protection") != NULL) {
          saveTargetStateToEEPROM(false);
        } else {
          clearEEPROMState();
        }
        
        targetModeActive = false;
        currentTargetLevel = 0.0;
        targetDescription = "";
      } else {
        Serial.print("Pump STOPPED - Reason: ");
        Serial.println(reason);
      }
    }
    
    sendPumpEventToBackend(newState ? "pump_start" : "pump_stop", reason);
   forceDisplayUpdate = true;
    
    // Track manual commands to prevent auto override
    if (strstr(reason, "API command") != NULL || 
        strstr(reason, "Manual") != NULL ||
        strstr(reason, "mobile app") != NULL) {
      lastManualCommandTime = millis();
      Serial.println("Manual command detected - setting override timer");
    }
    
    Serial.println("Pump state change completed");
  } else {
    Serial.println("No pump state change needed - already in requested state");
  }
}

void resetDailyUsage() {
  pumpStatus.dailyConsumption = 0;
  dailyResetTime = millis();
  Serial.println("Daily usage reset");
  sendPumpEventToBackend("usage_reset", "Manual daily reset");
}

void clearProtectionSystem() {
  pumpStatus.protectionActive = false;
  pumpStatus.overCurrentProtection = false;
  pumpStatus.overTimeProtection = false;
  Serial.println("Protection system manually cleared");
}

// ========================================
// RGB LED FUNCTIONS
// ========================================
void setupRGBLED() {
  pinMode(RGB_RED_PIN, OUTPUT);
  pinMode(RGB_GREEN_PIN, OUTPUT);
  pinMode(RGB_BLUE_PIN, OUTPUT);
  setRGBColor(COLOR_OFF);
}

void setRGBColor(const RGBColor &color) {
  analogWrite(RGB_RED_PIN, color.red);
  analogWrite(RGB_GREEN_PIN, color.green);
  analogWrite(RGB_BLUE_PIN, color.blue);
}

void setRGBColorBrightness(const RGBColor &color, uint8_t brightness) {
  RGBColor dimmedColor;
  dimmedColor.red = (color.red * brightness) / 255;
  dimmedColor.green = (color.green * brightness) / 255;
  dimmedColor.blue = (color.blue * brightness) / 255;
  setRGBColor(dimmedColor);
}

RGBState determineRGBState() {
  if (waterSupplyOn) return RGB_ROTATING_SUPPLY;
  if (pumpStatus.protectionActive) return RGB_SOLID_BLUE;  // Protection active
  if (!groundTankConnected && !roofTankConnected) return RGB_SOLID_RED;
  if ((groundTankConnected && !groundSensorWorking) || 
      (roofTankConnected && !roofSensorWorking)) return RGB_PULSE_PURPLE;
  if (!groundTankConnected || !roofTankConnected) return RGB_PULSE_YELLOW;
  if (groundTankConnected && roofTankConnected && 
      groundSensorWorking && roofSensorWorking) return RGB_SOLID_GREEN;
  return RGB_OFF;
}

void updateRGBStatus() {
  RGBState newState = determineRGBState();
  if (newState != currentRGBState) {
    currentRGBState = newState;
    rgbStateStartTime = millis();
  }
}

void handleRGBStateMachine() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastRGBUpdate >= RGB_UPDATE_INTERVAL) {
    switch (currentRGBState) {
      case RGB_OFF: 
        setRGBColor(COLOR_OFF); 
        break;
      case RGB_SOLID_GREEN: 
        setRGBColor(COLOR_GREEN); 
        break;
      case RGB_SOLID_RED: 
        setRGBColor(COLOR_RED); 
        break;
      case RGB_SOLID_BLUE: 
        setRGBColor(COLOR_BLUE); 
        break;
      
      case RGB_PULSE_YELLOW: {
        unsigned long pulseTime = (currentTime - rgbStateStartTime) % RGB_PULSE_INTERVAL;
        float pulseRatio = sin((pulseTime * 2.0 * PI) / RGB_PULSE_INTERVAL);
        currentBrightness = 150 + (105 * pulseRatio);
        setRGBColorBrightness(COLOR_YELLOW, currentBrightness);
        break;
      }
      
      case RGB_PULSE_PURPLE: {
        unsigned long pulseTime = (currentTime - rgbStateStartTime) % (RGB_PULSE_INTERVAL / 2);
        float pulseRatio = sin((pulseTime * 2.0 * PI) / (RGB_PULSE_INTERVAL / 2));
        currentBrightness = 150 + (105 * pulseRatio);
        setRGBColorBrightness(COLOR_PURPLE, currentBrightness);
        break;
      }
      
      case RGB_ROTATING_SUPPLY: {
        unsigned long rotationTime = (currentTime - rgbStateStartTime) % (RGB_ROTATION_INTERVAL * 3);
        if (rotationTime < RGB_ROTATION_INTERVAL) {
          setRGBColor(COLOR_RED);
        } else if (rotationTime < RGB_ROTATION_INTERVAL * 2) {
          setRGBColor(COLOR_GREEN);
        } else {
          setRGBColor(COLOR_BLUE);
        }
        break;
      }
    }
    lastRGBUpdate = currentTime;
  }
}

// ========================================
// ENCODER HANDLING
// ========================================
void checkEncoderNanoStyle() {
  if (encoderPos != lastEncoderPos) {
    int delta = encoderPos - lastEncoderPos;
    
    if (abs(delta) >= 2) {
      encoderWorking = true;
      int singleSteps = delta / 2;
      
      if (currentView == -1) {
        selectedMode = (selectedMode + singleSteps + TOTAL_MODES) % TOTAL_MODES;
        
        if (selectedMode < menuScrollOffset) {
          menuScrollOffset = selectedMode;
        } else if (selectedMode >= menuScrollOffset + MENU_ITEMS_VISIBLE) {
          menuScrollOffset = selectedMode - MENU_ITEMS_VISIBLE + 1;
        }
        
        playBeepNonBlocking(20);
        forceDisplayUpdate = true;
      } else if (currentView == 4) {
        switch (pumpMenuLevel) {
          case PUMP_MODE_SELECT:
            pumpMenuSelection = (pumpMenuSelection + singleSteps + MODE_SELECT_OPTIONS) % MODE_SELECT_OPTIONS;
            break;
            
          case PUMP_MANUAL_CONTROL:
            pumpMenuSelection = (pumpMenuSelection + singleSteps + MANUAL_CONTROL_OPTIONS) % MANUAL_CONTROL_OPTIONS;
            break;
            
          default:
            currentView = -1;
            pumpMenuLevel = PUMP_STATUS_VIEW;
            break;
        }
        playBeepNonBlocking(20);
        forceDisplayUpdate = true;
      } else {
        currentView = -1;
        pumpMenuLevel = PUMP_STATUS_VIEW;
        playBeepNonBlocking(30);
        forceDisplayUpdate = true;
      }
      
      lastEncoderPos = encoderPos;
    }
  }
}

void checkButtonWithDebounce() {
  unsigned long currentTime = millis();
  int reading = digitalRead(ENCODER_SW);
  
  if (reading != lastButtonState) {
    lastDebounceTime = currentTime;
  }
  
  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (reading != buttonPressed) {
      buttonPressed = reading;
      
      if (buttonPressed == LOW) {
        handleButtonPress();
        playBeepNonBlocking(50);
        forceDisplayUpdate = true;
      }
    }
  }
  
  lastButtonState = reading;
}

void handleButtonPress() {
  if (currentView == -1) {
    currentView = selectedMode;
    
    if (currentView == 3) {
      showMessageNonBlocking("RESETTING...");
      currentView = -1;
    }
  } else if (currentView == 4) {
    if (pumpStatus.protectionActive && pumpMenuLevel != PUMP_STATUS_VIEW) {
      showMessageWithAutoReturn("PROTECTION ACTIVE");
      pumpMenuLevel = PUMP_STATUS_VIEW;
      currentView = -1;
      return;
    }
    
    switch (pumpMenuLevel) {
      case PUMP_STATUS_VIEW:
        pumpMenuLevel = PUMP_MODE_SELECT;
        pumpMenuSelection = autoControlEnabled ? 0 : 1;
        forceDisplayUpdate = true;
        break;
        
      case PUMP_MODE_SELECT:
        if (pumpMenuSelection == 0) {
          // Auto Mode selected
          autoControlEnabled = true;
          manualPumpControl = false;
          if (targetModeActive) {
            setPumpState(false, "Switched to auto mode");
          }
          showMessageWithAutoReturn("AUTO MODE ENABLED");
          pumpMenuLevel = PUMP_STATUS_VIEW;
          currentView = -1;
        } else if (pumpMenuSelection == 1) {
          // Manual Mode selected
          autoControlEnabled = false;
          manualPumpControl = true;
          pumpMenuLevel = PUMP_MANUAL_CONTROL;
          pumpMenuSelection = 0;
          forceDisplayUpdate = true;
        } else if (pumpMenuSelection == 2) {
          // Back to Main Menu selected
          pumpMenuLevel = PUMP_STATUS_VIEW;
          currentView = -1;  // Go back to main menu
          playBeepNonBlocking(50);
          forceDisplayUpdate = true;
        }
        break;
        
      case PUMP_MANUAL_CONTROL:
        switch (pumpMenuSelection) {
          case 0:
            if (targetModeActive) {
              setPumpState(false, "Stopping target mode");
              delay(100);
            }
            setPumpState(true, "Manual control - CONTINUOUS ON");
            showMessageWithAutoReturn("PUMP ON");
            pumpMenuLevel = PUMP_STATUS_VIEW;
            currentView = -1;
            break;
            
          case 1:
            if (targetModeActive) {
              setPumpState(false, "Manual control - Target mode stopped");
            } else {
              setPumpState(false, "Manual control - OFF");
            }
            showMessageWithAutoReturn("PUMP OFF");
            pumpMenuLevel = PUMP_STATUS_VIEW;
            currentView = -1;
            break;
            
          case 2:
            startTargetPumpOperation(10.0, "10 INCH");
            pumpMenuLevel = PUMP_STATUS_VIEW;
            currentView = -1;
            break;
            
          case 3:
            startTargetPumpOperation(20.0, "20 INCH");
            pumpMenuLevel = PUMP_STATUS_VIEW;
            currentView = -1;
            break;
            
          case 4:
            startTargetPumpOperation(35.0, "35 INCH");
            pumpMenuLevel = PUMP_STATUS_VIEW;
            currentView = -1;
            break;
            
          case 5:
            pumpMenuLevel = PUMP_MODE_SELECT;
            pumpMenuSelection = autoControlEnabled ? 0 : 1;
            forceDisplayUpdate = true;
            break;
        }
        break;
    }
  } else if (currentView == 5) {
    resetDailyUsage();
    showMessageWithAutoReturn("USAGE RESET");
    currentView = -1;
  } else if (currentView == 6) {
    buzzerMuted = !buzzerMuted;
    if (!buzzerMuted) {
      playBeepNonBlocking(100);
      showMessageWithAutoReturn("BUZZER ENABLED");
    } else {
      showMessageWithAutoReturn("BUZZER MUTED");
    }
    currentView = -1;
  } else {
    currentView = -1;
  }
}

// ========================================
// AUDIO FUNCTIONS
// ========================================
void playBeepNonBlocking(uint8_t duration) {
  if (audioState == AUDIO_IDLE && !buzzerMuted) {
    audioState = AUDIO_BEEP;
    audioStartTime = millis();
    audioDuration = duration;
    digitalWrite(BUZZER_PIN, HIGH);
  }
}

void playAlertSoundNonBlocking() {
  if (audioState == AUDIO_IDLE && !buzzerMuted) {
    audioState = AUDIO_ALERT;
    audioStartTime = millis();
    audioStepTime = millis();
    audioStep = 0;
    audioRepeat = 0;
    digitalWrite(BUZZER_PIN, HIGH);
  }
}

void handleNonBlockingAudio() {
  unsigned long currentTime = millis();
  
  switch (audioState) {
    case AUDIO_BEEP:
      if (currentTime - audioStartTime >= audioDuration) {
        digitalWrite(BUZZER_PIN, LOW);
        audioState = AUDIO_IDLE;
      }
      break;
      
    case AUDIO_ALERT:
      if (audioStep == 0) {
        if (currentTime - audioStepTime >= 100) {
          digitalWrite(BUZZER_PIN, LOW);
          audioStep = 1;
          audioStepTime = currentTime;
        }
      } else {
        if (currentTime - audioStepTime >= 50) {
          audioRepeat++;
          if (audioRepeat < 3) {
            digitalWrite(BUZZER_PIN, HIGH);
            audioStep = 0;
            audioStepTime = currentTime;
          } else {
            audioState = AUDIO_IDLE;
          }
        }
      }
      break;
      
    default:
      audioState = AUDIO_IDLE;
      break;
  }
}

// ========================================
// MESSAGE FUNCTIONS
// ========================================
void showMessageNonBlocking(const char* message) {
  strncpy(currentMessage, message, 31);
  currentMessage[31] = '\0';
  messageActive = true;
  messageStartTime = millis();
  returnToMenuAfterMessage = false;
  forceDisplayUpdate = true;
}

void showMessageWithAutoReturn(const char* message) {
  strncpy(currentMessage, message, 31);
  currentMessage[31] = '\0';
  messageActive = true;
  messageStartTime = millis();
  returnToMenuAfterMessage = true;
  forceDisplayUpdate = true;
}

// ========================================
// DISPLAY FUNCTIONS
// ========================================
void updateDisplayAlways() {
  unsigned long currentTime = millis();
  
  if (forceDisplayUpdate || (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL)) {
    
    if (messageActive) {
      if (currentTime - messageStartTime >= MESSAGE_DISPLAY_TIME) {
        messageActive = false;
        if (returnToMenuAfterMessage) {
          currentView = -1;
          returnToMenuAfterMessage = false;
          
          if (selectedMode < menuScrollOffset) {
            menuScrollOffset = selectedMode;
          } else if (selectedMode >= menuScrollOffset + MENU_ITEMS_VISIBLE) {
            menuScrollOffset = selectedMode - MENU_ITEMS_VISIBLE + 1;
          }
        }
        forceDisplayUpdate = true;
      } else {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_8x13_tf);
        int width = u8g2.getStrWidth(currentMessage);
        int x = (128 - width) / 2;
        u8g2.drawStr(x, 35, currentMessage);
        u8g2.drawFrame(x - 5, 20, width + 10, 20);
        u8g2.sendBuffer();
        
        forceDisplayUpdate = false;
        lastDisplayUpdate = currentTime;
        return;
      }
    }
    
    switch (currentView) {
      case -1: 
        displayMainMenu(); 
        break;
      case 0: 
        displayDualTanksNanoStyle(); 
        break;
      case 1: 
        displaySingleTank(groundTankData, "GROUND TANK", groundTankConnected, groundSensorWorking); 
        break;
      case 2: 
        displaySingleTank(roofTankData, "ROOF TANK", roofTankConnected, roofSensorWorking); 
        break;
      case 4: 
        displayPumpControl(); 
        break;
      case 5: 
        displayPowerConsumption(); 
        break;
      case 6: 
        displayBuzzerSettings(); 
        break;
      case 7: 
        displaySystemInfo(); 
        break;
    }
    
    forceDisplayUpdate = false;
    lastDisplayUpdate = currentTime;
  }
}

void displayMainMenu() {
  u8g2.clearBuffer();
  
  u8g2.setFont(u8g2_font_6x12_tf);
  const char* title = "MAIN MENU";
  int titleWidth = u8g2.getStrWidth(title);
  u8g2.drawStr((128 - titleWidth) / 2, 14, title);
  
  const char* options[] = {
    "1. Dual Tank View",
    "2. Ground Tank",
    "3. Roof Tank", 
    "4. Reset System",
    "5. Pump Control",
    "6. Power Usage",
    "7. Buzzer Settings",
    "8. System Info"
  };
  
  int startIdx = menuScrollOffset;
  int endIdx = min(startIdx + MENU_ITEMS_VISIBLE, TOTAL_MODES);
  
  for (int i = startIdx; i < endIdx; i++) {
    int displayIndex = i - startIdx;
    int lineY = 26 + displayIndex * 8;
    
    if (i == selectedMode) {
      u8g2.drawBox(4, lineY - 9, 120, 10);
      u8g2.setDrawColor(0);
      u8g2.drawStr(8, lineY, options[i]);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(8, lineY, options[i]);
    }
  }
  
  u8g2.setFont(u8g2_font_5x8_tf);
  if (menuScrollOffset > 0) u8g2.drawStr(120, 22, "^");
  if (menuScrollOffset + MENU_ITEMS_VISIBLE < TOTAL_MODES) u8g2.drawStr(120, 62, "v");
  
  u8g2.drawStr(4, 78, "Rotate: Navigate  Press: Select");
  
  u8g2.sendBuffer();
}

void displayDualTanksNanoStyle() {
  char buffer[12];
  
  const int TITLE_Y = 10;
  const int TANK_NAME_Y = 22;
  const int TANK_WIDTH = 22;
  const int TANK_HEIGHT = 32;
  const int TANK_Y = 26;
  const int TANK_SPACING = 20;
  const int GROUND_TANK_X = 28;
  const int ROOF_TANK_X = GROUND_TANK_X + TANK_WIDTH + TANK_SPACING;
  const int GROUND_READING_X = 2;
  const int ROOF_READING_X = 100;
  const int READING_Y_START = 40;
  
  u8g2.clearBuffer();
  
  u8g2.setFont(u8g2_font_7x14_tf);
  const char* title = "WATER LEVEL SYSTEM";
  int titleWidth = u8g2.getStrWidth(title);
  int titleX = (128 - titleWidth) / 2;
  u8g2.drawStr(titleX, TITLE_Y, title);
  
  u8g2.setFont(u8g2_font_7x14_tf);
  if (waterSupplyOn) {
    int supplyWidth = u8g2.getStrWidth("SUPPLY ON");
    int supplyX = (128 - supplyWidth) / 2;
    u8g2.drawStr(supplyX, TANK_NAME_Y, "SUPPLY ON");
  } else {
    int groundNameWidth = u8g2.getStrWidth("GROUND");
    int roofNameWidth = u8g2.getStrWidth("ROOF");
    int groundNameX = GROUND_TANK_X + (TANK_WIDTH - groundNameWidth) / 2;
    int roofNameX = ROOF_TANK_X + (TANK_WIDTH - roofNameWidth) / 2;
    u8g2.drawStr(groundNameX, TANK_NAME_Y, "GROUND");
    u8g2.drawStr(roofNameX, TANK_NAME_Y, "ROOF");
  }
  
  u8g2.drawFrame(GROUND_TANK_X, TANK_Y, TANK_WIDTH, TANK_HEIGHT);
  u8g2.drawFrame(ROOF_TANK_X, TANK_Y, TANK_WIDTH, TANK_HEIGHT);
  
  // Ground tank content
  if (groundTankConnected && groundSensorWorking) {
    int waterHeight = map(constrain((int)groundTankData.levelPercent, 0, 100), 0, 100, 0, TANK_HEIGHT-2);
    if (waterHeight > 0) {
      u8g2.drawBox(GROUND_TANK_X+1, TANK_Y+TANK_HEIGHT-waterHeight-1, TANK_WIDTH-2, waterHeight);
    }
    
    u8g2.setFont(u8g2_font_7x14_tf);
    sprintf(buffer, "%d\"", (int)groundTankData.levelInches);
    u8g2.drawStr(GROUND_READING_X, READING_Y_START, buffer);
    
    u8g2.setFont(u8g2_font_6x12_tf);
    sprintf(buffer, "%d%%", (int)groundTankData.levelPercent);
    u8g2.drawStr(GROUND_READING_X, READING_Y_START + 14, buffer);
    
    if (groundAlarmActive) {
      u8g2.drawStr(GROUND_READING_X, READING_Y_START + 22, "ALARM");
    }
  } else if (groundTankConnected && !groundSensorWorking) {
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(GROUND_READING_X, READING_Y_START, "NO");  
    u8g2.drawStr(GROUND_READING_X, READING_Y_START + 10, "SENSOR");
  } else {
    u8g2.drawLine(GROUND_TANK_X, TANK_Y, GROUND_TANK_X+TANK_WIDTH, TANK_Y+TANK_HEIGHT);
    u8g2.drawLine(GROUND_TANK_X+TANK_WIDTH, TANK_Y, GROUND_TANK_X, TANK_Y+TANK_HEIGHT);
  }
  
  // Roof tank content
  if (roofTankConnected && roofSensorWorking) {
    int waterHeight = map(constrain((int)roofTankData.levelPercent, 0, 100), 0, 100, 0, TANK_HEIGHT-2);
    if (waterHeight > 0) {
      u8g2.drawBox(ROOF_TANK_X+1, TANK_Y+TANK_HEIGHT-waterHeight-1, TANK_WIDTH-2, waterHeight);
    }
    
    u8g2.setFont(u8g2_font_7x14_tf);
    sprintf(buffer, "%d\"", (int)roofTankData.levelInches);
    u8g2.drawStr(ROOF_READING_X, READING_Y_START, buffer);
    
    u8g2.setFont(u8g2_font_6x12_tf);
    sprintf(buffer, "%d%%", (int)roofTankData.levelPercent);
    u8g2.drawStr(ROOF_READING_X, READING_Y_START + 14, buffer);
    
    if (roofAlarmActive) {
      u8g2.drawStr(ROOF_READING_X-5, READING_Y_START + 22, "ALARM");
    }
  } else if (roofTankConnected && !roofSensorWorking) {
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(ROOF_READING_X, READING_Y_START, "NO");
    u8g2.drawStr(ROOF_READING_X-5, READING_Y_START + 10, "SENSOR");
  } else {
    u8g2.drawLine(ROOF_TANK_X, TANK_Y, ROOF_TANK_X+TANK_WIDTH, TANK_Y+TANK_HEIGHT);
    u8g2.drawLine(ROOF_TANK_X+TANK_WIDTH, TANK_Y, ROOF_TANK_X, TANK_Y+TANK_HEIGHT);
  }
  
  // Show pump status
  if (pumpState) {
    u8g2.setFont(u8g2_font_5x8_tf);
    if (pumpStatus.protectionActive) {
      u8g2.drawStr(40, 70, "PUMP PROTECTED");
    } else {
      u8g2.drawStr(45, 70, "PUMP ON");
    }
  }
  
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(5, 78, "Press button for menu");
  
  u8g2.sendBuffer();
}

void displaySingleTank(WaterLevelData &tankData, const char* tankName, bool connected, bool sensorWorking) {
  u8g2.clearBuffer();
  
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(5, 12, tankName);
  
  if (!connected) {
    u8g2.drawFrame(30, 25, 70, 16);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(50, 37, "OFFLINE");
  } else if (!sensorWorking) {
    u8g2.drawFrame(30, 25, 70, 16);
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr(45, 37, "NO SENSOR");
  } else {
    u8g2.drawFrame(10, 20, 30, 40);
    
    int waterHeight = map(constrain((int)tankData.levelPercent, 0, 100), 0, 100, 0, 38);
    if (waterHeight > 0) {
      u8g2.drawBox(11, 59 - waterHeight, 28, waterHeight);
    }
    
    char buffer[16];
    u8g2.setFont(u8g2_font_6x12_tf);
    sprintf(buffer, "Level: %d\"", (int)tankData.levelInches);
    u8g2.drawStr(45, 30, buffer);
    
    sprintf(buffer, "Full: %d%%", (int)tankData.levelPercent);
    u8g2.drawStr(45, 45, buffer);
    
    if (tankData.alarmActive) {
      u8g2.drawStr(45, 60, "ALARM!");
    }
  }
  
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(10, 78, "Press for menu");
  u8g2.sendBuffer();
}

void displayPumpControl() {
  u8g2.clearBuffer();
  
  switch (pumpMenuLevel) {
    case PUMP_STATUS_VIEW:
      u8g2.setFont(u8g2_font_7x14_tf);
      u8g2.drawStr(25, 12, "PUMP CONTROL");
      
      char buffer[30];
      u8g2.setFont(u8g2_font_6x12_tf);
      
      sprintf(buffer, "Mode: %s", autoControlEnabled ? "AUTO" : "MANUAL");
      u8g2.drawStr(5, 25, buffer);
      
      if (targetModeActive) {
        sprintf(buffer, "Target: %s", targetDescription.c_str());
        u8g2.drawStr(5, 35, buffer);
        
        sprintf(buffer, "Current: %.1f\"", roofTankData.levelInches);
        u8g2.drawStr(5, 45, buffer);
        
        sprintf(buffer, "Remaining: %.1f\"", currentTargetLevel - roofTankData.levelInches);
        u8g2.drawStr(5, 55, buffer);
      } else {
        sprintf(buffer, "Pump: %s", pumpState ? "RUNNING" : "STOPPED");
        u8g2.drawStr(5, 35, buffer);
        
        if (pumpStatus.protectionActive) {
          u8g2.drawStr(5, 45, "PROTECTION ACTIVE");
        } else {
          sprintf(buffer, "Current: %.1fA", pumpStatus.currentAmps);
          u8g2.drawStr(5, 45, buffer);
          
          sprintf(buffer, "Power: %.0fW", pumpStatus.powerWatts);
          u8g2.drawStr(5, 55, buffer);
        }
      }
      
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(5, 78, "Press: Mode Selection");
      break;
      
    case PUMP_MODE_SELECT:
      u8g2.setFont(u8g2_font_7x14_tf);
      u8g2.drawStr(20, 12, "SELECT MODE");
      
      u8g2.setFont(u8g2_font_6x12_tf);
      
      // Auto Mode option
      if (pumpMenuSelection == 0) {
        u8g2.drawBox(10, 22, 108, 12);
        u8g2.setDrawColor(0);
        u8g2.drawStr(15, 32, "AUTO MODE");
        u8g2.setDrawColor(1);
      } else {
        u8g2.drawStr(15, 32, "AUTO MODE");
      }
      
      // Manual Mode option  
      if (pumpMenuSelection == 1) {
        u8g2.drawBox(10, 37, 108, 12);
        u8g2.setDrawColor(0);
        u8g2.drawStr(15, 47, "MANUAL MODE");
        u8g2.setDrawColor(1);
      } else {
        u8g2.drawStr(15, 47, "MANUAL MODE");
      }
      
      // Back to Main Menu option
      if (pumpMenuSelection == 2) {
        u8g2.drawBox(10, 52, 108, 12);
        u8g2.setDrawColor(0);
        u8g2.drawStr(15, 62, "BACK TO MAIN MENU");
        u8g2.setDrawColor(1);
      } else {
        u8g2.drawStr(15, 62, "BACK TO MAIN MENU");
      }
      
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(5, 78, "Rotate: Select  Press: Confirm");
      break;
      
    case PUMP_MANUAL_CONTROL:
      u8g2.setFont(u8g2_font_7x14_tf);
      u8g2.drawStr(15, 12, "MANUAL CONTROL");
      
      u8g2.setFont(u8g2_font_6x12_tf);
      
      const char* manualOptions[] = {
        "1. PUMP ON",
        "2. PUMP OFF", 
        "3. TARGET 10\"",
        "4. TARGET 20\"",
        "5. TARGET 35\"",
        "6. BACK TO MODE"
      };
      
      int startOption = 0;
      int visibleOptions = 5;
      
      if (pumpMenuSelection >= visibleOptions) {
        startOption = pumpMenuSelection - visibleOptions + 1;
      }
      
      for (int i = 0; i < visibleOptions && (startOption + i) < MANUAL_CONTROL_OPTIONS; i++) {
        int optionIndex = startOption + i;
        int yPos = 25 + (i * 10);
        
        if (optionIndex == pumpMenuSelection) {
          u8g2.drawBox(5, yPos - 8, 118, 10);
          u8g2.setDrawColor(0);
          u8g2.drawStr(8, yPos, manualOptions[optionIndex]);
          u8g2.setDrawColor(1);
        } else {
          u8g2.drawStr(8, yPos, manualOptions[optionIndex]);
        }
      }
      
      u8g2.setFont(u8g2_font_5x8_tf);
      sprintf(buffer, "Current: %s", pumpState ? "RUNNING" : "STOPPED");
      u8g2.drawStr(5, 78, buffer);
      break;
  }
  
  u8g2.sendBuffer();
}

void displayPowerConsumption() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(25, 12, "POWER USAGE");
  
  char buffer[25];
  u8g2.setFont(u8g2_font_6x12_tf);
  
  sprintf(buffer, "Today: %.3f kWh", pumpStatus.dailyConsumption);
  u8g2.drawStr(5, 30, buffer);
  
  sprintf(buffer, "This Hour: %.3f kWh", pumpStatus.hourlyConsumption);
  u8g2.drawStr(5, 42, buffer);
  
  sprintf(buffer, "Total Runtime: %luh", pumpStatus.totalRunTimeHours);
  u8g2.drawStr(5, 54, buffer);
  
  if (pumpState) {
    unsigned long currentRuntime = (millis() - pumpStartTime) / 60000;
    sprintf(buffer, "Current Run: %lum", currentRuntime);
    u8g2.drawStr(5, 66, buffer);
  }
  
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(5, 78, "Press: Reset Daily");
  u8g2.sendBuffer();
}

void displayBuzzerSettings() {
  u8g2.clearBuffer();
  
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(15, 12, "BUZZER SETTINGS");
  
  u8g2.setFont(u8g2_font_6x12_tf);
  u8g2.drawStr(5, 30, "Current Status:");
  
  if (buzzerMuted) {
    u8g2.drawStr(25, 45, "MUTED");
    u8g2.drawStr(5, 60, "Press: ENABLE & Exit");
  } else {
    u8g2.drawStr(25, 45, "ENABLED");
    u8g2.drawStr(5, 60, "Press: MUTE & Exit");
  }
  
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(5, 78, "Press: Toggle & Return to Menu");
  
  u8g2.sendBuffer();
}

void displaySystemInfo() {
  u8g2.clearBuffer();
  
  u8g2.setFont(u8g2_font_7x14_tf);
  u8g2.drawStr(25, 12, "SYSTEM INFO");
  
  char buffer[25];
  u8g2.setFont(u8g2_font_6x12_tf);
  
  sprintf(buffer, "Ground: %s", groundTankConnected ? "OK" : "OFF");
  u8g2.drawStr(5, 25, buffer);
  
  sprintf(buffer, "Roof: %s", roofTankConnected ? "OK" : "OFF");
  u8g2.drawStr(5, 35, buffer);
  
  sprintf(buffer, "Pump: %s", pumpStatus.protectionActive ? "PROTECTED" : "OK");
  u8g2.drawStr(5, 45, buffer);
  
  sprintf(buffer, "WiFi: %s", WiFi.status() == WL_CONNECTED ? "OK" : "OFF");
  u8g2.drawStr(5, 55, buffer);
  
  sprintf(buffer, "Cloud: %s", backendConnected ? "OK" : "OFF");
  u8g2.drawStr(5, 65, buffer);
  
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(5, 78, "Press for menu");
  u8g2.sendBuffer();
}

// ========================================
// BACKEND COMMUNICATION
// ========================================
void sendStatusToBackend() {
  if (WiFi.status() != WL_CONNECTED) {
    backendConnected = false;
    Serial.println("WiFi not connected, skipping backend update");
    return;
  }
  
  DynamicJsonDocument doc(2048);
  
  doc["device_id"] = DEVICE_ID;
  doc["timestamp"] = millis();
  
  // Ground tank data
  JsonObject groundTank = doc.createNestedObject("ground_tank");
  groundTank["level_percent"] = groundTankData.levelPercent;
  groundTank["level_inches"] = groundTankData.levelInches;
  groundTank["alarm_active"] = groundAlarmActive;
  groundTank["connected"] = groundTankConnected;
  groundTank["sensor_working"] = groundSensorWorking;
  groundTank["water_supply_on"] = supplyFromGroundTank;
  
  // Roof tank data
  JsonObject roofTank = doc.createNestedObject("roof_tank");
  roofTank["level_percent"] = roofTankData.levelPercent;
  roofTank["level_inches"] = roofTankData.levelInches;
  roofTank["alarm_active"] = roofAlarmActive;
  roofTank["connected"] = roofTankConnected;
  roofTank["sensor_working"] = roofSensorWorking;
  roofTank["water_supply_on"] = supplyFromRoofTank;
  
  // Integrated pump data
  JsonObject pump = doc.createNestedObject("pump");
  pump["running"] = pumpStatus.pumpRunning;
  pump["manual_override"] = manualPumpControl;
  pump["current_amps"] = pumpStatus.currentAmps;
  pump["power_watts"] = pumpStatus.powerWatts;
  pump["daily_consumption"] = pumpStatus.dailyConsumption;
  pump["hourly_consumption"] = pumpStatus.hourlyConsumption;
  pump["runtime_minutes"] = pumpStatus.runTimeMinutes;
  pump["total_runtime_hours"] = pumpStatus.totalRunTimeHours;
  pump["protection_active"] = pumpStatus.protectionActive;
  pump["overcurrent_protection"] = pumpStatus.overCurrentProtection;
  pump["overtime_protection"] = pumpStatus.overTimeProtection;
  
  // System state
  JsonObject system = doc.createNestedObject("system");
  system["auto_mode_enabled"] = autoControlEnabled;
  system["manual_pump_control"] = manualPumpControl;
  system["water_supply_active"] = waterSupplyOn;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  String url = String("http://") + BACKEND_HOST + ":" + BACKEND_PORT + API_BASE_URL + "/devices/status/update";
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    backendConnected = true;
    lastBackendResponse = millis();
    backendErrorCount = 0;
  } else {
    backendConnected = false;
    backendErrorCount++;
  }
  
  http.end();
}

void sendPumpEventToBackend(const char* eventType, const char* reason) {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  DynamicJsonDocument doc(512);
  doc["event_type"] = eventType;
  doc["pump_on"] = pumpState;
  doc["auto_mode"] = autoControlEnabled;
  doc["trigger_reason"] = reason;
  doc["ground_tank_level"] = groundTankData.levelPercent;
  doc["roof_tank_level"] = roofTankData.levelPercent;
  doc["pump_current"] = pumpStatus.currentAmps;
  doc["pump_power"] = pumpStatus.powerWatts;
  doc["protection_active"] = pumpStatus.protectionActive;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  String url = String("http://") + BACKEND_HOST + ":" + BACKEND_PORT + API_BASE_URL + "/devices/events/pump";
  
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  
  int httpResponseCode = http.POST(jsonString);
  http.end();
}

// ========================================
// RADIO COMMUNICATION
// ========================================
void handleRadioReceive() {
  if (radio.available()) {
    uint8_t pipeNum;
    if (radio.available(&pipeNum)) {
      unsigned long currentTime = millis();
      
      if (pipeNum == 1 || pipeNum == 2) {
        WaterLevelData tempData;
        radio.read(&tempData, sizeof(tempData));
        
        if (tempData.tankID == 1) {
          groundTankData = tempData;
          groundTankConnected = true;
          lastGroundTankTime = currentTime;
          groundSensorWorking = (tempData.levelInches > 0.1 || tempData.levelPercent > 0.1);
          supplyFromGroundTank = tempData.waterSupplyOn;
          groundAlarmActive = tempData.alarmActive;
        } else if (tempData.tankID == 2) {
          roofTankData = tempData;
          roofTankConnected = true;
          lastRoofTankTime = currentTime;
          roofSensorWorking = (tempData.levelInches > 0.1 || tempData.levelPercent > 0.1);
          supplyFromRoofTank = tempData.waterSupplyOn;
          roofAlarmActive = tempData.alarmActive;
        }
        
        waterSupplyOn = supplyFromGroundTank || supplyFromRoofTank;
      }
    }
  }
}

// ========================================
// SYSTEM FUNCTIONS
// ========================================
void checkConnectionTimeouts() {
  unsigned long currentTime = millis();
  
  if (groundTankConnected && (currentTime - lastGroundTankTime >= CONNECTION_TIMEOUT)) {
    groundTankConnected = false;
    groundSensorWorking = false;
    forceDisplayUpdate = true;
  }
  
  if (roofTankConnected && (currentTime - lastRoofTankTime >= CONNECTION_TIMEOUT)) {
    roofTankConnected = false;
    roofSensorWorking = false;
    forceDisplayUpdate = true;
  }
}

void handleAutoPumpControl() {
  // Check if we're in manual command override period
  unsigned long currentTime = millis();
  bool inManualOverride = (currentTime - lastManualCommandTime) < MANUAL_COMMAND_OVERRIDE_DURATION;
  
  if (inManualOverride) {
    Serial.println("Auto control disabled - manual command override active");
    return;
  }
  
  // Handle target mode monitoring (works in manual mode)
  if (targetModeActive && pumpState) {
    bool shouldStop = false;
    const char* stopReason = "";
    
    if (roofTankConnected && roofSensorWorking && 
        roofTankData.levelInches >= currentTargetLevel) {
      shouldStop = true;
      stopReason = "Target level reached";
    }
    else if (groundTankConnected && groundSensorWorking && 
             groundTankData.levelPercent < GROUND_TANK_LOWER_THRESHOLD) {
      shouldStop = true;
      stopReason = "Ground tank protection";
    }
    else if (!roofTankConnected || !roofSensorWorking || 
             !groundTankConnected || !groundSensorWorking) {
      shouldStop = true;
      stopReason = "Sensor failure protection";
    }
    
    if (shouldStop) {
      setPumpState(false, stopReason);
      
      if (strstr(stopReason, "Target") != NULL) {
        playBeepNonBlocking(200);
        showMessageNonBlocking("TARGET COMPLETE!");
      } else {
        playAlertSoundNonBlocking();
        showMessageNonBlocking("TARGET PAUSED");
      }
      
      return;
    }
  }
  
  // Normal auto control (only when auto mode enabled and no target mode active)
  if (!autoControlEnabled || pumpStatus.protectionActive || targetModeActive || manualPumpControl) {
    if (!autoControlEnabled) Serial.println("Auto control skipped - auto mode disabled");
    if (pumpStatus.protectionActive) Serial.println("Auto control skipped - protection active");
    if (targetModeActive) Serial.println("Auto control skipped - target mode active");
    if (manualPumpControl) Serial.println("Auto control skipped - manual mode active");
    return;
  }
  
  Serial.println("Auto control active - checking conditions...");
  
  Serial.print("System mode - autoControlEnabled: ");
  Serial.print(autoControlEnabled ? "TRUE" : "FALSE");
  Serial.print(", manualPumpControl: ");
  Serial.print(manualPumpControl ? "TRUE" : "FALSE");
  Serial.print(", targetModeActive: ");
  Serial.println(targetModeActive ? "TRUE" : "FALSE");
  
  if (groundTankConnected && groundSensorWorking && 
      roofTankConnected && roofSensorWorking) {
    
    Serial.print("Ground tank: ");
    Serial.print(groundTankData.levelPercent);
    Serial.print("% (threshold: ");
    Serial.print(GROUND_TANK_UPPER_THRESHOLD);
    Serial.print("), Roof tank: ");
    Serial.print(roofTankData.levelPercent);
    Serial.print("% (threshold: ");
    Serial.print(ROOF_TANK_LOWER_THRESHOLD);
    Serial.println(")");
    
    if (!pumpState && 
        groundTankData.levelPercent >= GROUND_TANK_UPPER_THRESHOLD && 
        roofTankData.levelPercent <= ROOF_TANK_LOWER_THRESHOLD) {
      
      Serial.println("Auto control: Starting pump");
      setPumpState(true, "Auto start: Ground tank full, roof tank low");
      playBeepNonBlocking(200);
    }
    else if (pumpState && 
            (groundTankData.levelPercent < GROUND_TANK_LOWER_THRESHOLD || 
             roofTankData.levelPercent >= ROOF_TANK_UPPER_THRESHOLD)) {
      
      const char* reason = (groundTankData.levelPercent < GROUND_TANK_LOWER_THRESHOLD) ? 
                          "Auto stop: Ground tank empty protection" : 
                          "Auto stop: Roof tank full protection";
      Serial.println("Auto control: Stopping pump");
      setPumpState(false, reason);
      playBeepNonBlocking(100);
    } else {
      Serial.println("Auto control: No action needed");
    }
  }
  else if (pumpState) {
    Serial.println("Auto control: Safety stop due to sensor issues");
    setPumpState(false, "Safety stop: Sensor failure or connection lost");
    playAlertSoundNonBlocking();
  }
}

void updateBackendVariablesBackground() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastBackendUpdate >= BACKEND_UPDATE_INTERVAL) {
    sendStatusToBackend();
    lastBackendUpdate = currentTime;
  }
}

// ========================================
// SETUP FUNCTION
// ========================================
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Water Level Receiver - Enhanced Pump Control");
  
  // WiFi setup
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart) < 30000) {
    delay(1000);
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected! IP: ");
    Serial.println(WiFi.localIP());
  }
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("EEPROM initialized");
  
  // Configure pins
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Setup integrated pump control
  setupPumpControl();
  
  // Setup RGB LED
  setupRGBLED();
  
  // Setup encoder interrupt
  lastClkState = digitalRead(ENCODER_CLK);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), updateEncoder, CHANGE);
  Serial.println("Encoder interrupt attached");
  
  // Initialize I2C and OLED
  Wire.begin(21, 22);
  u8g2.begin();
  
  showMessageNonBlocking("STARTING...");
  
  // Initialize SPI and NRF24L01
  SPI.begin(18, 19, 23, 5);
  
  if (!radio.begin()) {
    showMessageNonBlocking("RADIO ERROR!");
  } else {
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(76);
    
    // Only setup receiving pipes for tank transmitters
    radio.openReadingPipe(1, rxAddr1);  // Ground tank
    radio.openReadingPipe(2, rxAddr2);  // Roof tank
    radio.startListening();
    Serial.println("Radio initialized for tank monitoring");
  }
  
  // Initialize data structures
  groundTankData = {0.0, 0.0, false, false, 1};
  roofTankData = {0.0, 0.0, false, false, 2};
  
  // Check for power-fail recovery AFTER hardware is ready
  if (loadStateFromEEPROM()) {
    if (persistentTarget.wasActive) {
      Serial.println("POWER RECOVERY MODE ACTIVATED");
      
      showMessageNonBlocking("POWER RECOVERY");
      playBeepNonBlocking(150);
      
      needsPowerRecovery = true;
      powerRecoveryStartTime = millis();
      
      currentRGBState = RGB_PULSE_YELLOW;
      rgbStateStartTime = millis();
    }
  }
  
  playBeepNonBlocking(100);
  showMessageNonBlocking("READY");
  
  currentView = 0;
  selectedMode = 0;
  menuScrollOffset = 0;
  currentRGBState = RGB_OFF;
  rgbStateStartTime = millis();
  
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Clear any protection system issues for testing
  clearProtectionSystem();
  
  // Initialize WebSocket connection
  webSocket.begin(WS_HOST, WS_PORT, WS_URL);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
  
  Serial.println("Setup complete - Enhanced pump control ready!");
}

// ========================================
// MAIN LOOP
// ========================================
void loop() {
  // Input handling
  checkEncoderNanoStyle();
  checkButtonWithDebounce();
  updateDisplayAlways();
  
  // Handle power recovery
  handlePowerRecovery();
  
  // System operations
  updateRGBStatus();
  handleRGBStateMachine();
  handleRadioReceive();
  handleNonBlockingAudio();
  
  // Integrated pump control
  updateCurrentReading();
  updatePowerConsumption();
  checkPumpProtection();
  
  // System control and monitoring
  checkConnectionTimeouts();
  handleAutoPumpControl();
  
  // Background operations
  updateBackendVariablesBackground();
  
  // Check for pump commands
  checkForPumpCommands();
  
  // Handle WebSocket events
  webSocket.loop();
  
  // Handle OTA updates
  if (otaUpdateAvailable && !otaUpdateInProgress) {
    // Check if user wants to start OTA update (you can add a button or menu option)
    // For now, we'll auto-start after a delay
    static unsigned long otaAutoStartTime = 0;
    if (otaAutoStartTime == 0) {
      otaAutoStartTime = millis();
    } else if (millis() - otaAutoStartTime > 10000) { // 10 second delay
      startOTAUpdate();
      otaAutoStartTime = 0;
    }
  }
  
  // Periodic EEPROM saves during active operations
  unsigned long currentTime = millis();
  if (currentTime - lastEEPROMSave > EEPROM_SAVE_INTERVAL) {
    if (targetModeActive || pumpState) {
      saveSystemStateToEEPROM();
      
      if (targetModeActive) {
        saveTargetStateToEEPROM(false);
      }
    }
  }
  
  yield();
}

// ========================================
// COMMAND PROCESSING FUNCTIONS
// ========================================
void checkForPumpCommands() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastCommandCheck >= COMMAND_CHECK_INTERVAL) {
    if (WiFi.status() == WL_CONNECTED) {
      // Check for commands in Redis via HTTP endpoint
      String url = String("http://") + BACKEND_HOST + ":" + BACKEND_PORT + API_BASE_URL + "/devices/pump/command/" + DEVICE_ID;
      
      Serial.print("Checking for pump commands at: ");
      Serial.println(url);
      
      http.begin(url);
      http.addHeader("Content-Type", "application/json");
      
      int httpResponseCode = http.GET();
      
      Serial.print("HTTP Response Code: ");
      Serial.println(httpResponseCode);
      
      if (httpResponseCode == 200) {
        String payload = http.getString();
        Serial.print("Received payload: ");
        Serial.println(payload);
        
        if (payload.length() > 0 && payload != "null") {
          Serial.println("Processing pump command...");
          processPumpCommand(payload);
        } else {
          Serial.println("No command found (empty or null payload)");
        }
      } else {
        Serial.print("HTTP request failed with code: ");
        Serial.println(httpResponseCode);
      }
      
      http.end();
    } else {
      Serial.println("WiFi not connected, skipping command check");
    }
    
    lastCommandCheck = currentTime;
  }
}

void processPumpCommand(const String& commandJson) {
  Serial.println("=== PROCESSING PUMP COMMAND ===");
  Serial.print("Raw JSON: ");
  Serial.println(commandJson);
  
  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, commandJson);
  
  if (error) {
    Serial.print("Failed to parse pump command JSON: ");
    Serial.println(error.c_str());
    return;
  }
  
  const char* action = doc["action"];
  const char* reason = doc["reason"];
  float targetLevel = doc["target_level"] | -1.0;
  
  Serial.print("Parsed action: ");
  Serial.println(action ? action : "NULL");
  Serial.print("Parsed reason: ");
  Serial.println(reason ? reason : "NULL");
  Serial.print("Parsed target_level: ");
  Serial.println(targetLevel);
  
  if (!action) {
    Serial.println("No action in pump command");
    return;
  }
  
  Serial.print("Processing pump command: ");
  Serial.println(action);
  
  if (strcmp(action, "start") == 0) {
    Serial.println("Executing START command");
    // Manual continuous start
    if (pumpStatus.protectionActive) {
      showMessageWithAutoReturn("PROTECTION ACTIVE");
      return;
    }
    
    setPumpState(true, reason ? reason : "API command - continuous start");
    showMessageWithAutoReturn("API: PUMP ON");
    
  } else if (strcmp(action, "stop") == 0) {
    Serial.println("Executing STOP command");
    // Manual stop
    setPumpState(false, reason ? reason : "API command - stop");
    showMessageWithAutoReturn("API: PUMP OFF");
    
  } else if (strcmp(action, "target") == 0) {
    Serial.println("Executing TARGET command");
    // Target mode
    if (targetLevel > 0) {
      char description[20];
      sprintf(description, "API TARGET %.1f\"", targetLevel);
      startTargetPumpOperation(targetLevel, description);
    } else {
      showMessageWithAutoReturn("API: INVALID TARGET");
    }
    
  } else if (strcmp(action, "auto") == 0) {
    Serial.println("Executing AUTO command");
    // Enable auto mode
    autoControlEnabled = true;
    manualPumpControl = false;
    Serial.println("Mode changed: AUTO mode enabled, manual control disabled");
    if (targetModeActive) {
      setPumpState(false, "API command - switched to auto mode");
    }
    showMessageWithAutoReturn("API: AUTO MODE");
    
  } else if (strcmp(action, "manual") == 0) {
    Serial.println("Executing MANUAL command");
    // Enable manual mode
    autoControlEnabled = false;
    manualPumpControl = true;
    Serial.println("Mode changed: MANUAL mode enabled, auto control disabled");
    showMessageWithAutoReturn("API: MANUAL MODE");
  } else if (strcmp(action, "reset_usage") == 0) {
    Serial.println("Executing RESET_USAGE command");
    // Reset daily usage
    resetDailyUsage();
    showMessageWithAutoReturn("API: USAGE RESET");
  } else {
    Serial.print("Unknown action: ");
    Serial.println(action);
  }
  
  Serial.println("=== COMMAND PROCESSING COMPLETE ===");
}

// ========================================
// WEBSOCKET EVENT HANDLERS
// ========================================
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket disconnected");
      break;
      
    case WStype_CONNECTED:
      Serial.println("WebSocket connected");
      // Subscribe to device updates
      webSocket.sendTXT("{\"event\":\"subscribe_device\",\"data\":\"" + String(DEVICE_ID) + "\"}");
      break;
      
    case WStype_TEXT:
      handleWebSocketMessage(payload, length);
      break;
      
    case WStype_ERROR:
      Serial.println("WebSocket error");
      break;
  }
}

void handleWebSocketMessage(uint8_t * payload, size_t length) {
  String message = String((char*)payload);
  Serial.print("WebSocket message: ");
  Serial.println(message);
  
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.println("Failed to parse WebSocket message");
    return;
  }
  
  const char* event = doc["event"];
  if (!event) return;
  
  if (strcmp(event, "ota_update_available") == 0) {
    handleOTAUpdateAvailable(doc);
  } else if (strcmp(event, "ota_progress_update") == 0) {
    handleOTAProgressUpdate(doc);
  } else if (strcmp(event, "ota_update_complete") == 0) {
    handleOTAUpdateComplete(doc);
  }
}

void handleOTAUpdateAvailable(JsonDocument& doc) {
  otaUpdateAvailable = true;
  otaVersion = doc["version"] | "";
  otaDownloadUrl = doc["download_url"] | "";
  
  Serial.print("OTA update available: ");
  Serial.println(otaVersion);
  
  showMessageNonBlocking("OTA UPDATE AVAILABLE");
  playBeepNonBlocking(200);
}

void handleOTAProgressUpdate(JsonDocument& doc) {
  otaProgress = doc["progress"] | 0;
  otaStatus = doc["status"] | "";
  
  Serial.print("OTA progress: ");
  Serial.print(otaProgress);
  Serial.print("% - ");
  Serial.println(otaStatus);
}

void handleOTAUpdateComplete(JsonDocument& doc) {
  bool success = doc["success"] | false;
  String version = doc["version"] | "";
  
  if (success) {
    Serial.print("OTA update completed successfully: ");
    Serial.println(version);
    showMessageNonBlocking("OTA UPDATE COMPLETE");
    playBeepNonBlocking(300);
  } else {
    String error = doc["error"] | "";
    Serial.print("OTA update failed: ");
    Serial.println(error);
    showMessageNonBlocking("OTA UPDATE FAILED");
    playAlertSoundNonBlocking();
  }
  
  otaUpdateInProgress = false;
  otaUpdateAvailable = false;
}

// ========================================
// OTA UPDATE FUNCTIONS
// ========================================
void startOTAUpdate() {
  if (otaUpdateInProgress || otaDownloadUrl.isEmpty()) {
    Serial.println("OTA update already in progress or no URL available");
    return;
  }
  
  Serial.println("Starting OTA update...");
  otaUpdateInProgress = true;
  otaStartTime = millis();
  otaProgress = 0;
  otaStatus = "Starting download";
  
  showMessageNonBlocking("STARTING OTA UPDATE");
  
  // Send progress update
  sendOTAProgress(0, "Starting download");
  
  // Start the update process
  performOTAUpdate();
}

void performOTAUpdate() {
  if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
    Serial.println("Not enough space to begin OTA");
    otaStatus = "Not enough space";
    sendOTAProgress(0, "Not enough space");
    otaUpdateInProgress = false;
    return;
  }
  
  otaStatus = "Downloading firmware";
  sendOTAProgress(10, "Downloading firmware");
  
  HTTPClient http;
  http.begin(otaDownloadUrl);
  
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
    otaStatus = "Download failed";
    sendOTAProgress(0, "Download failed");
    otaUpdateInProgress = false;
    http.end();
    return;
  }
  
  int contentLength = http.getSize();
  Serial.printf("Content-Length: %d\n", contentLength);
  
  if (contentLength <= 0) {
    Serial.println("Content-Length not defined");
    otaStatus = "Invalid content length";
    sendOTAProgress(0, "Invalid content length");
    otaUpdateInProgress = false;
    http.end();
    return;
  }
  
  WiFiClient * stream = http.getStreamPtr();
  size_t written = 0;
  size_t totalBytes = 0;
  
  otaStatus = "Installing firmware";
  sendOTAProgress(20, "Installing firmware");
  
  while (http.connected() && (totalBytes < contentLength)) {
    size_t sizeAvailable = stream->available();
    if (sizeAvailable) {
      uint8_t buff[128] = { 0 };
      size_t bytesToRead = ((sizeAvailable > sizeof(buff)) ? sizeof(buff) : sizeAvailable);
      size_t bytesRead = stream->readBytes(buff, bytesToRead);
      
      written = Update.write(buff, bytesRead);
      if (written != bytesRead) {
        Serial.println("Written size doesn't match");
        otaStatus = "Write failed";
        sendOTAProgress(0, "Write failed");
        otaUpdateInProgress = false;
        http.end();
        return;
      }
      
      totalBytes += bytesRead;
      
      // Update progress
      int progress = map(totalBytes, 0, contentLength, 20, 90);
      if (progress != otaProgress) {
        otaProgress = progress;
        sendOTAProgress(progress, "Installing firmware");
      }
    }
    delay(1);
  }
  
  http.end();
  
  otaStatus = "Finalizing update";
  sendOTAProgress(90, "Finalizing update");
  
  if (Update.end()) {
    Serial.println("OTA update completed successfully");
    otaStatus = "Update completed";
    sendOTAProgress(100, "Update completed");
    
    // Send completion notification
    sendOTAComplete(true, otaVersion, "");
    
    // Restart after a short delay
    showMessageNonBlocking("UPDATE COMPLETE - RESTARTING");
    delay(2000);
    ESP.restart();
  } else {
    Serial.println("OTA update failed");
    otaStatus = "Update failed";
    sendOTAProgress(0, "Update failed");
    sendOTAComplete(false, "", "Update verification failed");
    otaUpdateInProgress = false;
  }
}

void sendOTAProgress(int progress, const char* status) {
  if (webSocket.isConnected()) {
    String message = "{\"event\":\"ota_progress\",\"data\":{\"device_id\":\"" + String(DEVICE_ID) + 
                    "\",\"progress\":" + String(progress) + 
                    ",\"status\":\"" + String(status) + "\"}}";
    webSocket.sendTXT(message);
  }
}

void sendOTAComplete(bool success, String version, String error) {
  if (webSocket.isConnected()) {
    String message = "{\"event\":\"ota_complete\",\"data\":{\"device_id\":\"" + String(DEVICE_ID) + 
                    "\",\"success\":" + String(success ? "true" : "false") + 
                    ",\"version\":\"" + version + 
                    "\",\"error\":\"" + error + "\"}}";
    webSocket.sendTXT(message);
  }
}