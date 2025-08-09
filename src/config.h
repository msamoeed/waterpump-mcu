#pragma once

// ========================================
// SYSTEM CONFIGURATION
// ========================================
#define FIRMWARE_VERSION "1.0.0"
#define DEVICE_ID "esp32_controller_001"

// ========================================
// FREERTOS CONFIGURATION
// ========================================
#define TASK_STACK_SIZE_SMALL     4096     // Increased from 2048
#define TASK_STACK_SIZE_MEDIUM    8192     // Increased from 4096  
#define TASK_STACK_SIZE_LARGE     16384    // Increased from 8192

// Core assignments
#define CORE_COMMUNICATION       0
#define CORE_CONTROL_DISPLAY      1

// Task priorities (0-24, higher = more priority)
#define PRIORITY_CRITICAL         20
#define PRIORITY_HIGH             15
#define PRIORITY_NORMAL           10
#define PRIORITY_LOW              5

// ========================================
// NETWORK CONFIGURATION
// ========================================
const char WIFI_SSID[] = "Ahad 2.4Ghz";
const char WIFI_PASS[] = "Artifact";

const char BACKEND_HOST[] = "205.189.160.7";
const int BACKEND_PORT = 3002;
const char API_BASE_URL[] = "/api/v1";

const char WS_HOST[] = "205.189.160.7";
const int WS_PORT = 3002;

// ========================================
// GITHUB CONFIGURATION
// ========================================
const char GITHUB_REPO[] = "msamoeed/waterpump-mcu";

// ========================================
// PIN DEFINITIONS
// ========================================
// RF24 Radio
#define CE_PIN              4
#define CSN_PIN             5

// User Interface
#define ENCODER_CLK         32
#define ENCODER_DT          25
#define ENCODER_SW          15
#define BUZZER_PIN          26

// RGB LED
#define RGB_RED_PIN         13
#define RGB_GREEN_PIN       14
#define RGB_BLUE_PIN        27

// Pump Control
#define RELAY_PIN           33
#define CURRENT_SENSOR_PIN  -1      // Disabled for now
#define VOLTAGE_DETECT_PIN  34

// Status LED
#define STATUS_LED_PIN      2

// I2C (OLED Display)
#define SDA_PIN             21
#define SCL_PIN             22

// SPI (RF24)
#define SCK_PIN             18
#define MISO_PIN            19
#define MOSI_PIN            23

// ========================================
// TIMING CONSTANTS
// ========================================
#define UPDATE_INTERVAL_DISPLAY     50      // ms
#define UPDATE_INTERVAL_SENSORS     100     // ms
#define UPDATE_INTERVAL_PUMP        1000    // ms
#define UPDATE_INTERVAL_BACKEND     30000   // ms
#define UPDATE_INTERVAL_COMMANDS    5000    // ms

#define CONNECTION_TIMEOUT          15000   // ms
#define BACKEND_TIMEOUT             10000   // ms
#define OTA_TIMEOUT                 30000   // ms

// ========================================
// PUMP CONTROL CONSTANTS
// ========================================
#define MAX_PUMP_RUNTIME_HOURS          1
#define PUMP_PROTECTION_DELAY           300000  // 5 minutes
#define MANUAL_COMMAND_OVERRIDE_DURATION 30000  // 30 seconds

#define NORMAL_PUMP_CURRENT_MIN         2.0f
#define NORMAL_PUMP_CURRENT_MAX         8.0f
#define OVERCURRENT_THRESHOLD           10.0f

// ========================================
// WATER LEVEL THRESHOLDS
// ========================================
#define GROUND_TANK_LOWER_THRESHOLD     15.0f
#define GROUND_TANK_UPPER_THRESHOLD     30.0f
#define ROOF_TANK_LOWER_THRESHOLD       20.0f
#define ROOF_TANK_UPPER_THRESHOLD       80.0f

// ========================================
// EEPROM CONFIGURATION
// ========================================
#define EEPROM_SIZE                     512
#define EEPROM_TARGET_STATE_ADDR        0
#define EEPROM_SYSTEM_STATE_ADDR        100
#define EEPROM_MAGIC_NUMBER             0xABCD1234

// ========================================
// RADIO CONFIGURATION
// ========================================
const uint64_t RX_ADDR_GROUND_TANK = 0xF0F0F0F0E1LL;
const uint64_t RX_ADDR_ROOF_TANK   = 0xF0F0F0F0E2LL;

// ========================================
// LOGGING CONFIGURATION
// ========================================
#define REMOTE_LOGGING_ENABLED          1
#define REMOTE_LOG_MIN_LEVEL            1  // 0=debug,1=info,2=warn,3=error
#define LOG_BUFFER_CAPACITY             32
#define LOG_FLUSH_INTERVAL_MS           1000
#define LOG_BATCH_MAX                   10

// ========================================
// QUEUE SIZES
// ========================================
#define QUEUE_SIZE_COMMANDS             10
#define QUEUE_SIZE_SENSOR_DATA          20
#define QUEUE_SIZE_LOGS                 50
#define QUEUE_SIZE_DISPLAY_UPDATES      10 