#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <IPAddress.h>

// ========================================
// ENUMS
// ========================================
enum class LogLevel : uint8_t {
    DEBUG = 0,
    INFO = 1,
    WARN = 2,
    ERROR = 3
};

enum class PumpCommand : uint8_t {
    START,
    STOP,
    TARGET,
    AUTO_MODE,
    MANUAL_MODE,
    RESET_USAGE
};

enum class RGBState : uint8_t {
    OFF,
    SOLID_GREEN,      // Both transmitters OK
    SOLID_RED,        // Both transmitters offline
    SOLID_BLUE,       // Pump protection active
    PULSE_YELLOW,     // One transmitter offline
    PULSE_PURPLE,     // Sensor defective
    ROTATING_SUPPLY   // Water supply active
};

enum class AudioState : uint8_t {
    IDLE,
    BEEP,
    ALERT
};

enum class SystemMode : uint8_t {
    AUTO,
    MANUAL,
    TARGET
};

enum class TaskState : uint8_t {
    READY,
    RUNNING,
    SUSPENDED,
    ERROR
};

// ========================================
// DATA STRUCTURES
// ========================================
struct RGBColor {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    
    RGBColor(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0) : red(r), green(g), blue(b) {}
};

struct WaterLevelData {
    float levelPercent;
    float levelInches;
    bool alarmActive;
    bool waterSupplyOn;
    bool connected;
    bool sensorWorking;
    uint8_t tankID;
    unsigned long lastUpdate;
    
    WaterLevelData() : levelPercent(0), levelInches(0), alarmActive(false), 
                      waterSupplyOn(false), connected(false), sensorWorking(false), 
                      tankID(0), lastUpdate(0) {}
};

struct PumpStatus {
    bool running;
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
    unsigned long startTime;
    
    PumpStatus() : running(false), manualOverride(false), currentAmps(0), powerWatts(0),
                  dailyConsumption(0), hourlyConsumption(0), runTimeMinutes(0), 
                  totalRunTimeHours(0), protectionActive(false), overCurrentProtection(false),
                  overTimeProtection(false), protectionStartTime(0), startTime(0) {}
};

struct SystemState {
    SystemMode mode;
    bool autoControlEnabled;
    bool manualPumpControl;
    bool waterSupplyActive;
    bool targetModeActive;
    float currentTargetLevel;
    String targetDescription;
    unsigned long lastManualCommandTime;
    
    SystemState() : mode(SystemMode::AUTO), autoControlEnabled(true), manualPumpControl(false),
                   waterSupplyActive(false), targetModeActive(false), currentTargetLevel(0),
                   targetDescription(""), lastManualCommandTime(0) {}
};

struct WiFiStatus {
    bool connected;
    String ssid;
    int32_t rssi;
    IPAddress localIP;
    unsigned long lastConnectAttempt;
    uint32_t reconnectCount;
    
    WiFiStatus() : connected(false), ssid(""), rssi(0), localIP(0,0,0,0), 
                  lastConnectAttempt(0), reconnectCount(0) {}
};

struct BackendStatus {
    bool connected;
    unsigned long lastResponse;
    uint32_t errorCount;
    unsigned long lastUpdateSent;
    
    BackendStatus() : connected(false), lastResponse(0), errorCount(0), lastUpdateSent(0) {}
};

struct OTAStatus {
    bool updateAvailable;
    bool updateInProgress;
    String version;
    String downloadUrl;
    int progress;
    String status;
    unsigned long startTime;
    
    OTAStatus() : updateAvailable(false), updateInProgress(false), version(""), 
                 downloadUrl(""), progress(0), status(""), startTime(0) {}
};

struct LogEntry {
    LogLevel level;
    String message;
    String tag;
    unsigned long timestamp;
    
    LogEntry() : level(LogLevel::INFO), message(""), tag(""), timestamp(0) {}
    LogEntry(LogLevel l, const String& msg, const String& t = "") 
        : level(l), message(msg), tag(t), timestamp(millis()) {}
};

struct CommandMessage {
    PumpCommand command;
    String reason;
    float targetLevel;
    unsigned long timestamp;
    
    CommandMessage() : command(PumpCommand::STOP), reason(""), targetLevel(0), timestamp(0) {}
    CommandMessage(PumpCommand cmd, const String& r = "", float target = 0) 
        : command(cmd), reason(r), targetLevel(target), timestamp(millis()) {}
};

struct SensorReading {
    uint8_t tankId;
    WaterLevelData data;
    unsigned long timestamp;
    
    SensorReading() : tankId(0), data(), timestamp(0) {}
    SensorReading(uint8_t id, const WaterLevelData& d) 
        : tankId(id), data(d), timestamp(millis()) {}
};

struct DisplayUpdate {
    bool forceUpdate;
    String message;
    bool showMessage;
    unsigned long messageStartTime;
    
    DisplayUpdate() : forceUpdate(false), message(""), showMessage(false), messageStartTime(0) {}
};

// ========================================
// PERSISTENT STORAGE STRUCTURES
// ========================================
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
// TASK INFORMATION STRUCTURE
// ========================================
struct TaskInfo {
    TaskHandle_t handle;
    const char* name;
    TaskState state;
    uint32_t stackHighWaterMark;
    unsigned long lastRunTime;
    
    TaskInfo() : handle(nullptr), name(""), state(TaskState::READY), 
                stackHighWaterMark(0), lastRunTime(0) {}
};

// ========================================
// CONSTANTS
// ========================================
const RGBColor COLOR_OFF(0, 0, 0);
const RGBColor COLOR_RED(255, 0, 0);
const RGBColor COLOR_GREEN(0, 255, 0);
const RGBColor COLOR_BLUE(0, 0, 255);
const RGBColor COLOR_YELLOW(255, 255, 0);
const RGBColor COLOR_PURPLE(255, 0, 255);
const RGBColor COLOR_WHITE(255, 255, 255); 