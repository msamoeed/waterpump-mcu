/*
 * ESP32 Water Pump Controller - Refactored Multi-Core Architecture
 * 
 * This is a complete refactoring of the original monolithic code into a 
 * professional, industry-standard multi-core architecture using:
 * 
 * - FreeRTOS task management with core assignment
 * - Hardware Abstraction Layer (HAL)
 * - Modular communication management
 * - Inter-task communication via queues
 * - Proper synchronization with mutexes
 * - Separation of concerns
 * 
 * Core 0: Communication tasks (WiFi, HTTP, WebSocket, OTA, Logging)
 * Core 1: Control & Display tasks (Pump Control, Sensors, Display, User Input)
 */

 #include <Arduino.h>
 #include <EEPROM.h>
 
 // Core modules
 #include "config.h"
 #include "types.h"
 #include "core/TaskManager.h"
 #include "hardware/HardwareAbstraction.h"
 #include "communication/CommunicationManager.h"
 
 // ========================================
 // GLOBAL MANAGERS (SINGLETONS)
 // ========================================
 TaskManager& taskManager = TaskManager::getInstance();
 HardwareManager& hardwareManager = HardwareManager::getInstance();
 CommunicationManager& commManager = CommunicationManager::getInstance();
 
 // ========================================
 // SHARED STATE (PROTECTED BY MUTEXES)
 // ========================================
 SystemState g_systemState;
 PumpStatus g_pumpStatus;
 WaterLevelData g_groundTankData;
 WaterLevelData g_roofTankData;
 OTAStatus g_otaStatus;
 
 // ========================================
 // FORWARD DECLARATIONS
 // ========================================
 void initializeEEPROM();
 void handleOTAEvent(const String& event, const JsonVariant& data);
 void logMessage(LogLevel level, const String& tag, const String& message);
 
 // ========================================
 // SETUP FUNCTION
 // ========================================
 void setup() {
     Serial.begin(115200);
     delay(1000);
     
         Serial.println("\n==================================================");
    Serial.println("ESP32 Water Pump Controller v" + String(FIRMWARE_VERSION));
    Serial.println("Multi-Core Architecture");
    Serial.println("==================================================");
     
     // Initialize EEPROM first
     initializeEEPROM();
     
     // Initialize core systems
     if (!taskManager.initialize()) {
         Serial.println("[SETUP] TaskManager initialization failed!");
         while (true) delay(1000);
     }
     
     if (!hardwareManager.initializeAll()) {
         Serial.println("[SETUP] Hardware initialization failed!");
         while (true) delay(1000);
     }
     
     if (!commManager.initialize()) {
         Serial.println("[SETUP] Communication initialization failed!");
         // Continue without communication (local operation only)
     }
     
     // Set up OTA event handling
     commManager.setOTAEventCallback(handleOTAEvent);
     
     // Initialize system state
     g_systemState = SystemState();
     g_pumpStatus = PumpStatus();
     g_groundTankData = WaterLevelData();
     g_roofTankData = WaterLevelData();
     g_otaStatus = OTAStatus();
     
         // Display startup message
    if (hardwareManager.getDisplay()) {
        hardwareManager.getDisplay()->clearBuffer();
        hardwareManager.getDisplay()->drawStr(20, 30, "INITIALIZING...");
        hardwareManager.getDisplay()->sendBuffer();
    }
     
     // Play startup sound
     if (hardwareManager.getAudio()) {
         hardwareManager.getAudio()->playBeep(200);
     }
     
     // Start all tasks (this will create and start the FreeRTOS tasks)
     taskManager.startAllTasks();
     
     logMessage(LogLevel::INFO, "STARTUP", "System initialization complete");
     
     Serial.println("[SETUP] System ready! Tasks started on both cores.");
     Serial.println("[SETUP] Core 0: Communication, OTA, Logging");
     Serial.println("[SETUP] Core 1: Control, Sensors, Display");
     
     // The main loop will now be handled by FreeRTOS tasks
     // This setup() function completes and Arduino's loop() becomes minimal
 }
 
 // ========================================
 // MAIN LOOP (MINIMAL - RTOS HANDLES EVERYTHING)
 // ========================================
 void loop() {
     // In the new architecture, most work is done by FreeRTOS tasks
     // The main loop just handles system monitoring and emergency stops
     
     static unsigned long lastSystemCheck = 0;
     unsigned long currentTime = millis();
     
     if (currentTime - lastSystemCheck >= 5000) { // Every 5 seconds
         // System health check
         if (taskManager.getFreeHeap() < 10000) { // Less than 10KB free
             Serial.println("[MAIN] WARNING: Low memory!");
             logMessage(LogLevel::WARN, "MAIN", "Low memory: " + String(taskManager.getFreeHeap()) + " bytes");
         }
         
         // Emergency pump stop if critical error
         if (taskManager.getMinimumFreeHeap() < 5000) { // Critical memory
             if (hardwareManager.getPump() && hardwareManager.getPump()->isRunning()) {
                 hardwareManager.getPump()->turnOff();
                 logMessage(LogLevel::ERROR, "MAIN", "Emergency pump stop - Critical memory");
             }
         }
         
         lastSystemCheck = currentTime;
     }
     
     // Small delay to prevent watchdog timeout
     delay(100);
 }
 
 // ========================================
 // INITIALIZATION FUNCTIONS
 // ========================================
 void initializeEEPROM() {
     if (!EEPROM.begin(EEPROM_SIZE)) {
         Serial.println("[EEPROM] Initialization failed!");
         return;
     }
     Serial.println("[EEPROM] Initialized (" + String(EEPROM_SIZE) + " bytes)");
 }
 
 // ========================================
 // EVENT HANDLERS
 // ========================================
 void handleOTAEvent(const String& event, const JsonVariant& data) {
     if (taskManager.takeMutex(taskManager.getSystemMutex(), pdMS_TO_TICKS(100))) {
         if (event == "ota_update_available") {
             g_otaStatus.updateAvailable = true;
             g_otaStatus.version = data["version"].as<String>();
             g_otaStatus.downloadUrl = data["download_url"].as<String>();
             
             logMessage(LogLevel::INFO, "OTA", "Update available: " + g_otaStatus.version);
             
             // Queue display update
             DisplayUpdate displayUpdate;
             displayUpdate.showMessage = true;
             displayUpdate.message = "OTA UPDATE AVAILABLE";
             displayUpdate.forceUpdate = true;
             xQueueSend(taskManager.getDisplayUpdateQueue(), &displayUpdate, 0);
             
         } else if (event == "ota_progress_update") {
             g_otaStatus.progress = data["progress"] | 0;
             g_otaStatus.status = data["status"].as<String>();
             
         } else if (event == "ota_update_complete") {
             bool success = data["success"] | false;
             g_otaStatus.updateInProgress = false;
             g_otaStatus.updateAvailable = false;
             
             if (success) {
                 logMessage(LogLevel::INFO, "OTA", "Update completed successfully");
                 // System will restart automatically
             } else {
                 String error = data["error"].as<String>();
                 logMessage(LogLevel::ERROR, "OTA", "Update failed: " + error);
             }
         }
         
         taskManager.giveMutex(taskManager.getSystemMutex());
     }
 }
 
 // ========================================
 // UTILITY FUNCTIONS
 // ========================================
 void logMessage(LogLevel level, const String& tag, const String& message) {
     // Send to log queue for background processing
     LogEntry logEntry(level, message, tag);
     xQueueSend(taskManager.getLogQueue(), &logEntry, 0);
     
     // Also print to serial for debugging
     String levelStr;
     switch (level) {
         case LogLevel::DEBUG: levelStr = "DEBUG"; break;
         case LogLevel::INFO:  levelStr = "INFO";  break;
         case LogLevel::WARN:  levelStr = "WARN";  break;
         case LogLevel::ERROR: levelStr = "ERROR"; break;
     }
     
     Serial.printf("[%s] %s: %s\n", levelStr.c_str(), tag.c_str(), message.c_str());
 }
 
 // ========================================
 // TASK IMPLEMENTATIONS (SEPARATED BY CORE)
 // ========================================
 
 // ========================================
 // CORE 0 TASKS (COMMUNICATION)
 // ========================================
 extern "C" void communicationTask(void* parameters) {
     Serial.println("[CommunicationTask] Started on core " + String(xPortGetCoreID()));
     
     TickType_t lastWakeTime = xTaskGetTickCount();
     const TickType_t frequency = pdMS_TO_TICKS(100); // 100ms
     
     while (true) {
         // Update communication components
         commManager.update();
         
         // Check for commands periodically
         static unsigned long lastCommandCheck = 0;
         if (millis() - lastCommandCheck >= UPDATE_INTERVAL_COMMANDS) {
             CommandMessage command;
             if (commManager.checkForCommands(command)) {
                 // Send command to pump control task
                 xQueueSend(taskManager.getCommandQueue(), &command, 0);
             }
             lastCommandCheck = millis();
         }
         
         // Send status updates periodically
         static unsigned long lastStatusUpdate = 0;
         if (millis() - lastStatusUpdate >= UPDATE_INTERVAL_BACKEND) {
             if (taskManager.takeMutex(taskManager.getSystemMutex(), pdMS_TO_TICKS(10))) {
                 commManager.sendStatusUpdate(g_systemState, g_pumpStatus, 
                                            g_groundTankData, g_roofTankData);
                 taskManager.giveMutex(taskManager.getSystemMutex());
             }
             lastStatusUpdate = millis();
         }
         
         taskManager.feedWatchdog();
         vTaskDelayUntil(&lastWakeTime, frequency);
     }
 }
 
 extern "C" void loggerTask(void* parameters) {
     Serial.println("[LoggerTask] Started on core " + String(xPortGetCoreID()));
     
     LogEntry logBuffer[LOG_BATCH_MAX];
     size_t logCount = 0;
     
     TickType_t lastWakeTime = xTaskGetTickCount();
     const TickType_t frequency = pdMS_TO_TICKS(500); // 500ms
     
     while (true) {
         // Collect logs from queue
         LogEntry logEntry;
         while (logCount < LOG_BATCH_MAX && 
                xQueueReceive(taskManager.getLogQueue(), &logEntry, 0) == pdTRUE) {
             logBuffer[logCount++] = logEntry;
         }
         
         // Send logs if we have any
         if (logCount > 0) {
             commManager.sendLogs(logBuffer, logCount);
             logCount = 0;
         }
         
         taskManager.feedWatchdog();
         vTaskDelayUntil(&lastWakeTime, frequency);
     }
 }
 
 // ========================================
 // CORE 1 TASKS (CONTROL & DISPLAY)  
 // ========================================
 extern "C" void sensorTask(void* parameters) {
     Serial.println("[SensorTask] Started on core " + String(xPortGetCoreID()));
     
     TickType_t lastWakeTime = xTaskGetTickCount();
     const TickType_t frequency = pdMS_TO_TICKS(UPDATE_INTERVAL_SENSORS);
     
     while (true) {
         // Read radio data
         if (hardwareManager.getRadio() && hardwareManager.getRadio()->isAvailable()) {
             WaterLevelData receivedData;
             if (hardwareManager.getRadio()->read(&receivedData, sizeof(receivedData))) {
                 
                 if (taskManager.takeMutex(taskManager.getSystemMutex(), pdMS_TO_TICKS(10))) {
                     if (receivedData.tankID == 1) {
                         g_groundTankData = receivedData;
                         g_groundTankData.connected = true;
                         g_groundTankData.lastUpdate = millis();
                     } else if (receivedData.tankID == 2) {
                         g_roofTankData = receivedData;
                         g_roofTankData.connected = true;
                         g_roofTankData.lastUpdate = millis();
                     }
                     taskManager.giveMutex(taskManager.getSystemMutex());
                 }
                 
                 // Send sensor reading to display task
                 SensorReading reading(receivedData.tankID, receivedData);
                 xQueueSend(taskManager.getSensorDataQueue(), &reading, 0);
             }
         }
         
         // Check connection timeouts
         if (taskManager.takeMutex(taskManager.getSystemMutex(), pdMS_TO_TICKS(10))) {
             unsigned long currentTime = millis();
             
             if (g_groundTankData.connected && 
                 (currentTime - g_groundTankData.lastUpdate) > CONNECTION_TIMEOUT) {
                 g_groundTankData.connected = false;
                 g_groundTankData.sensorWorking = false;
             }
             
             if (g_roofTankData.connected && 
                 (currentTime - g_roofTankData.lastUpdate) > CONNECTION_TIMEOUT) {
                 g_roofTankData.connected = false;
                 g_roofTankData.sensorWorking = false;
             }
             
             taskManager.giveMutex(taskManager.getSystemMutex());
         }
         
         taskManager.feedWatchdog();
         vTaskDelayUntil(&lastWakeTime, frequency);
     }
 }
 
 extern "C" void pumpControlTask(void* parameters) {
     Serial.println("[PumpControlTask] Started on core " + String(xPortGetCoreID()));
     
     TickType_t lastWakeTime = xTaskGetTickCount();
     const TickType_t frequency = pdMS_TO_TICKS(UPDATE_INTERVAL_PUMP);
     
     while (true) {
         // Process commands
         CommandMessage command;
         if (xQueueReceive(taskManager.getCommandQueue(), &command, 0) == pdTRUE) {
             // Process pump command
             String reason = command.reason.isEmpty() ? "Remote command" : command.reason;
             
             switch (command.command) {
                 case PumpCommand::START:
                     if (hardwareManager.getPump()) {
                         hardwareManager.getPump()->turnOn();
                         commManager.sendPumpEvent("pump_start", reason);
                     }
                     break;
                     
                 case PumpCommand::STOP:
                     if (hardwareManager.getPump()) {
                         hardwareManager.getPump()->turnOff();
                         commManager.sendPumpEvent("pump_stop", reason);
                     }
                     break;
                     
                 case PumpCommand::AUTO_MODE:
                     if (taskManager.takeMutex(taskManager.getSystemMutex(), pdMS_TO_TICKS(10))) {
                         g_systemState.mode = SystemMode::AUTO;
                         g_systemState.autoControlEnabled = true;
                         g_systemState.manualPumpControl = false;
                         taskManager.giveMutex(taskManager.getSystemMutex());
                     }
                     break;
                     
                 case PumpCommand::MANUAL_MODE:
                     if (taskManager.takeMutex(taskManager.getSystemMutex(), pdMS_TO_TICKS(10))) {
                         g_systemState.mode = SystemMode::MANUAL;
                         g_systemState.autoControlEnabled = false;
                         g_systemState.manualPumpControl = true;
                         taskManager.giveMutex(taskManager.getSystemMutex());
                     }
                     break;
                     
                 default:
                     break;
             }
         }
         
         // Update pump status
         if (hardwareManager.getPump() && 
             taskManager.takeMutex(taskManager.getSystemMutex(), pdMS_TO_TICKS(10))) {
             
             g_pumpStatus.running = hardwareManager.getPump()->isRunning();
             g_pumpStatus.currentAmps = hardwareManager.getPump()->readCurrent();
             g_pumpStatus.powerWatts = g_pumpStatus.currentAmps * 230.0f; // Estimate
             
             taskManager.giveMutex(taskManager.getSystemMutex());
         }
         
         // Auto control logic (simplified)
         if (taskManager.takeMutex(taskManager.getSystemMutex(), pdMS_TO_TICKS(10))) {
             if (g_systemState.autoControlEnabled && 
                 g_groundTankData.connected && g_roofTankData.connected) {
                 
                 bool shouldStart = !g_pumpStatus.running &&
                                  g_groundTankData.levelPercent >= GROUND_TANK_UPPER_THRESHOLD &&
                                  g_roofTankData.levelPercent <= ROOF_TANK_LOWER_THRESHOLD;
                 
                 bool shouldStop = g_pumpStatus.running &&
                                 (g_groundTankData.levelPercent < GROUND_TANK_LOWER_THRESHOLD ||
                                  g_roofTankData.levelPercent >= ROOF_TANK_UPPER_THRESHOLD);
                 
                 if (shouldStart && hardwareManager.getPump()) {
                     hardwareManager.getPump()->turnOn();
                     commManager.sendPumpEvent("pump_start", "Auto control");
                 } else if (shouldStop && hardwareManager.getPump()) {
                     hardwareManager.getPump()->turnOff();
                     commManager.sendPumpEvent("pump_stop", "Auto control");
                 }
             }
             taskManager.giveMutex(taskManager.getSystemMutex());
         }
         
         taskManager.feedWatchdog();
         vTaskDelayUntil(&lastWakeTime, frequency);
     }
 }
 
 extern "C" void displayTask(void* parameters) {
     Serial.println("[DisplayTask] Started on core " + String(xPortGetCoreID()));
     
     TickType_t lastWakeTime = xTaskGetTickCount();
     const TickType_t frequency = pdMS_TO_TICKS(UPDATE_INTERVAL_DISPLAY);
     
     while (true) {
         // Update non-blocking hardware (audio, RGB)
         hardwareManager.updateNonBlockingHardware();
         
         // Process display updates
         DisplayUpdate displayUpdate;
                 if (xQueueReceive(taskManager.getDisplayUpdateQueue(), &displayUpdate, 0) == pdTRUE) {
            if (displayUpdate.showMessage && hardwareManager.getDisplay()) {
                hardwareManager.getDisplay()->clearBuffer();
                hardwareManager.getDisplay()->drawStr(10, 30, displayUpdate.message.c_str());
                hardwareManager.getDisplay()->sendBuffer();
            }
        }
         
         // Regular display update
         if (hardwareManager.getDisplay() && 
             taskManager.takeMutex(taskManager.getSystemMutex(), pdMS_TO_TICKS(10))) {
             
                         // Simple status display
            hardwareManager.getDisplay()->clearBuffer();
            
            char buffer[32];
             sprintf(buffer, "Ground: %.1f%%", g_groundTankData.levelPercent);
             hardwareManager.getDisplay()->drawStr(5, 15, buffer);
             
             sprintf(buffer, "Roof: %.1f%%", g_roofTankData.levelPercent);
             hardwareManager.getDisplay()->drawStr(5, 30, buffer);
             
             sprintf(buffer, "Pump: %s", g_pumpStatus.running ? "ON" : "OFF");
             hardwareManager.getDisplay()->drawStr(5, 45, buffer);
             
             sprintf(buffer, "Mode: %s", g_systemState.autoControlEnabled ? "AUTO" : "MANUAL");
             hardwareManager.getDisplay()->drawStr(5, 60, buffer);
             
             hardwareManager.getDisplay()->sendBuffer();
             taskManager.giveMutex(taskManager.getSystemMutex());
         }
         
         taskManager.feedWatchdog();
         vTaskDelayUntil(&lastWakeTime, frequency);
     }
 }
 
 extern "C" void otaTask(void* parameters) {
     Serial.println("[OTATask] Started on core " + String(xPortGetCoreID()));
     
     while (true) {
         // OTA handling logic would go here
         // This task sleeps most of the time and only activates during OTA
         
         taskManager.delayMs(1000);
         taskManager.feedWatchdog();
     }
 }
 
 extern "C" void systemMonitorTask(void* parameters) {
     Serial.println("[SystemMonitorTask] Started on core " + String(xPortGetCoreID()));
     
     while (true) {
         // Print system statistics every 30 seconds
         static unsigned long lastStatsTime = 0;
         if (millis() - lastStatsTime >= 30000) {
             taskManager.printSystemStats();
             lastStatsTime = millis();
         }
         
         taskManager.delayMs(10000); // Check every 10 seconds
         taskManager.feedWatchdog();
     }
 } 