/*
 * ESP32 Main Receiver - WITH INTEGRATED RELAY CONTROL
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

 #include <WiFi.h>
 #include <Wire.h>
 #include <U8g2lib.h>
 #include <HTTPClient.h>
 #include <ArduinoJson.h>
 #include <SPI.h>
 #include <RF24.h>
 
 // ========================================
 // CONFIGURATION
 // ========================================
 // WiFi Configuration - Update with your network details
 const char WIFI_SSID[] = "Ahad 2.4Ghz";
 const char WIFI_PASS[] = "Artifact";
 
 // Backend Configuration - Update with your backend server details
 // If running locally: use "localhost" or "127.0.0.1"
 // If running on another device: use the IP address of your backend server
 const char BACKEND_HOST[] = "205.189.160.7";  // Updated to your server IP
 const int BACKEND_PORT = 3002;  // Updated to match your server port
 const char DEVICE_ID[] = "esp32_controller_001";
 const char API_BASE_URL[] = "/api/v1";
 
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
 #define MAX_PUMP_RUNTIME_HOURS 4        // Maximum continuous runtime
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
 
 // ========================================
 // RADIO ADDRESSES (Motor controller addresses removed)
 // ========================================
 const uint64_t rxAddr1 = 0xF0F0F0F0E1LL;  // Ground tank
 const uint64_t rxAddr2 = 0xF0F0F0F0E2LL;  // Roof tank
 
 // ========================================
 // DATA STRUCTURES (Simplified)
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
 // GLOBAL VARIABLES (Updated)
 // ========================================
 WaterLevelData groundTankData, roofTankData;
 IntegratedPumpStatus pumpStatus;  // Replaces motorControllerStatus
 
 bool groundTankConnected = false;
 bool roofTankConnected = false;
 bool groundSensorWorking = false;
 bool roofSensorWorking = false;
 bool groundAlarmActive = false;
 bool roofAlarmActive = false;
 bool waterSupplyOn = false;
 bool forceDisplayUpdate = false;
 bool supplyFromGroundTank = false, supplyFromRoofTank = false;
 
 // Pump control variables (updated)
 bool pumpState = false;
 bool autoControlEnabled = true;
 bool manualPumpControl = false;
 bool pumpProtectionActive = false;
 
 // Connection tracking (motor controller removed)
 unsigned long lastGroundTankTime = 0;
 unsigned long lastRoofTankTime = 0;
 unsigned long lastDisplayUpdate = 0;
 unsigned long lastLedToggle = 0;
 unsigned long lastCloudUpdate = 0;
 
 // NEW: Pump monitoring variables
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
 
 // ========================================
 // ENCODER VARIABLES (Same as original)
 // ========================================
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
 
 // Audio System (Same as original)
 enum AudioState { AUDIO_IDLE, AUDIO_BEEP, AUDIO_ALERT };
 AudioState audioState = AUDIO_IDLE;
 unsigned long audioStartTime = 0;
 unsigned long audioStepTime = 0;
 uint8_t audioStep = 0;
 uint8_t audioDuration = 0;
 uint8_t audioRepeat = 0;
 bool buzzerMuted = false;
 
 // Message System (Same as original)
 bool messageActive = false;
 unsigned long messageStartTime = 0;
 char currentMessage[32] = "";
 bool returnToMenuAfterMessage = false;
 
 // Water Level Thresholds (Same as original)
 #define GROUND_TANK_LOWER_THRESHOLD 15.0
 #define GROUND_TANK_UPPER_THRESHOLD 30.0
 #define ROOF_TANK_LOWER_THRESHOLD 20.0
 #define ROOF_TANK_UPPER_THRESHOLD 80.0
 
 // Timing constants (Updated)
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
 
 // ========================================
 // NANO-STYLE ENCODER INTERRUPT (Same as original)
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
 // NEW: INTEGRATED PUMP CONTROL FUNCTIONS
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
     
     // Debug output every 10 readings
     static int debugCounter = 0;
     debugCounter++;
     if (debugCounter >= 10) {
       Serial.print("Current reading: ");
       Serial.print(newCurrent);
       Serial.print("A, Average: ");
       Serial.print(pumpStatus.currentAmps);
       Serial.println("A");
       debugCounter = 0;
     }
     
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
   if (pumpStatus.protectionActive && newState) {
     Serial.println("Pump start blocked by protection system");
     return;
   }
   
   if (pumpState != newState) {
     pumpState = newState;
     pumpStatus.pumpRunning = newState;
     digitalWrite(RELAY_PIN, newState ? HIGH : LOW);
     
     if (newState) {
       pumpStartTime = millis();
       Serial.print("Pump STARTED - Reason: ");
     } else {
       unsigned long runtime = (millis() - pumpStartTime) / 60000;  // minutes
       pumpStatus.runTimeMinutes += runtime;
       pumpStatus.totalRunTimeHours = pumpStatus.runTimeMinutes / 60;
       Serial.print("Pump STOPPED - Reason: ");
     }
     Serial.println(reason);
     
     // Send event to backend
     sendPumpEventToBackend(newState ? "pump_start" : "pump_stop", reason);
     
     forceDisplayUpdate = true;
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
 // RGB LED FUNCTIONS (Updated for new status)
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
 // ENCODER AND BUTTON HANDLING (Updated for new menus)
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
       } else {
         currentView = -1;
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
       // Reset option
       showMessageNonBlocking("RESETTING...");
       // Add reset logic here
       currentView = -1;
     }
   } else if (currentView == 4) {
     // Motor Control screen (now Pump Control)
     if (pumpStatus.protectionActive) {
       showMessageWithAutoReturn("PROTECTION ACTIVE");
       currentView = -1;
     } else if (autoControlEnabled) {
       autoControlEnabled = false;
       showMessageWithAutoReturn("MANUAL MODE");
     } else {
       setPumpState(!pumpState, "Manual control");
     }
   } else if (currentView == 5) {
     // Power Consumption screen - reset daily usage
     resetDailyUsage();
     showMessageWithAutoReturn("USAGE RESET");
     currentView = -1;
   } else if (currentView == 6) {
     // Buzzer Settings screen
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
 // AUDIO FUNCTIONS (Same as original)
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
 // MESSAGE FUNCTIONS (Same as original)
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
 // DISPLAY FUNCTIONS (Updated for integrated pump)
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
     "5. Pump Control",     // Updated from Motor Control
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
   
   u8g2.setFont(u8g2_font_7x14_tf);
   u8g2.drawStr(25, 12, "PUMP CONTROL");
   
   char buffer[25];
   u8g2.setFont(u8g2_font_6x12_tf);
   
   sprintf(buffer, "Pump: %s", pumpState ? "RUNNING" : "STOPPED");
   u8g2.drawStr(5, 25, buffer);
   
   sprintf(buffer, "Mode: %s", autoControlEnabled ? "AUTO" : "MANUAL");
   u8g2.drawStr(5, 35, buffer);
   
   if (pumpStatus.protectionActive) {
     u8g2.drawStr(5, 45, "PROTECTION ACTIVE");
     if (pumpStatus.overCurrentProtection) {
       u8g2.drawStr(5, 55, "Overcurrent");
     }
     if (pumpStatus.overTimeProtection) {
       u8g2.drawStr(5, 65, "Runtime exceeded");
     }
   } else {
     sprintf(buffer, "Current: %.1fA", pumpStatus.currentAmps);
     u8g2.drawStr(5, 45, buffer);
     
     sprintf(buffer, "Power: %.0fW", pumpStatus.powerWatts);
     u8g2.drawStr(5, 55, buffer);
   }
   
   u8g2.setFont(u8g2_font_5x8_tf);
   if (pumpStatus.protectionActive) {
     u8g2.drawStr(5, 78, "Press: View Only");
   } else {
     u8g2.drawStr(5, 78, "Press: Toggle Mode/State");
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
 // BACKEND COMMUNICATION (Updated)
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
   
   // Integrated pump data (replaces motor data)
   JsonObject pump = doc.createNestedObject("pump");
   pump["running"] = pumpStatus.pumpRunning;
   pump["manual_override"] = manualPumpControl;  // Use the correct variable
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
   system["manual_pump_control"] = manualPumpControl;  // Fixed variable name
   system["water_supply_active"] = waterSupplyOn;
   
   String jsonString;
   serializeJson(doc, jsonString);
   
   // Debug: Print the JSON being sent (first 200 characters)
   Serial.print("JSON payload: ");
   Serial.println(jsonString.substring(0, 200));
   
   String url = String("http://") + BACKEND_HOST + ":" + BACKEND_PORT + API_BASE_URL + "/devices/status/update";
   Serial.print("Sending status to: ");
   Serial.println(url);
   
   http.begin(url);
   http.addHeader("Content-Type", "application/json");
   
   int httpResponseCode = http.POST(jsonString);
   Serial.print("HTTP Response Code: ");
   Serial.println(httpResponseCode);
   
   if (httpResponseCode > 0) {
     backendConnected = true;
     lastBackendResponse = millis();
     backendErrorCount = 0;
     Serial.println("Backend update successful");
   } else {
     backendConnected = false;
     backendErrorCount++;
     Serial.print("Backend update failed, error count: ");
     Serial.println(backendErrorCount);
   }
   
   http.end();
 }
 
 void sendPumpEventToBackend(const char* eventType, const char* reason) {
   if (WiFi.status() != WL_CONNECTED) {
     Serial.println("WiFi not connected, skipping pump event");
     return;
   }
   
   DynamicJsonDocument doc(512);
   doc["device_id"] = DEVICE_ID;
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
   
   // Debug: Print the JSON being sent
   Serial.print("Pump event JSON: ");
   Serial.println(jsonString);
   
   String url = String("http://") + BACKEND_HOST + ":" + BACKEND_PORT + API_BASE_URL + "/devices/events/pump";
   Serial.print("Sending pump event to: ");
   Serial.println(url);
   
   http.begin(url);
   http.addHeader("Content-Type", "application/json");
   
   int httpResponseCode = http.POST(jsonString);
   Serial.print("Pump event HTTP Response Code: ");
   Serial.println(httpResponseCode);
   http.end();
 }
 
 // ========================================
 // RADIO COMMUNICATION (Simplified - no motor controller)
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
 // SYSTEM FUNCTIONS (Updated)
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
   if (!autoControlEnabled || pumpStatus.protectionActive) return;
   
   if (groundTankConnected && groundSensorWorking && 
       roofTankConnected && roofSensorWorking) {
     
     if (!pumpState && 
         groundTankData.levelPercent >= GROUND_TANK_UPPER_THRESHOLD && 
         roofTankData.levelPercent <= ROOF_TANK_LOWER_THRESHOLD) {
       
       setPumpState(true, "Auto start: Ground tank full, roof tank low");
       playBeepNonBlocking(200);
     }
     else if (pumpState && 
             (groundTankData.levelPercent < GROUND_TANK_LOWER_THRESHOLD || 
              roofTankData.levelPercent >= ROOF_TANK_UPPER_THRESHOLD)) {
       
       const char* reason = (groundTankData.levelPercent < GROUND_TANK_LOWER_THRESHOLD) ? 
                           "Auto stop: Ground tank empty protection" : 
                           "Auto stop: Roof tank full protection";
       setPumpState(false, reason);
       playBeepNonBlocking(100);
     }
   }
   else if (pumpState) {
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
 // SETUP FUNCTION (Updated)
 // ========================================
 void setup() {
   Serial.begin(115200);
   Serial.println("ESP32 Water Level Receiver - Integrated Pump Control");
   
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
   
   Serial.println("Setup complete - Integrated pump control ready!");
 }
 
 // ========================================
 // MAIN LOOP (Updated)
 // ========================================
 void loop() {
   // Input handling
   checkEncoderNanoStyle();
   checkButtonWithDebounce();
   updateDisplayAlways();
   
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
   
   yield();
 }