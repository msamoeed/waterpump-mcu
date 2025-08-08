#include "TaskManager.h"
#include "esp_system.h"

TaskManager* TaskManager::instance = nullptr;

TaskManager::TaskManager() : taskCount(0), initialized(false), systemStartTime(0) {
    systemMutex = nullptr;
    displayMutex = nullptr;
    communicationMutex = nullptr;
    commandQueue = nullptr;
    sensorDataQueue = nullptr;
    logQueue = nullptr;
    displayUpdateQueue = nullptr;
}

TaskManager& TaskManager::getInstance() {
    if (instance == nullptr) {
        instance = new TaskManager();
    }
    return *instance;
}

bool TaskManager::initialize() {
    if (initialized) return true;
    
    Serial.println("[TaskManager] Initializing...");
    
    // Create mutexes
    systemMutex = xSemaphoreCreateMutex();
    displayMutex = xSemaphoreCreateMutex();
    communicationMutex = xSemaphoreCreateMutex();
    
    if (!systemMutex || !displayMutex || !communicationMutex) {
        Serial.println("[TaskManager] Failed to create mutexes");
        return false;
    }
    
    // Create queues
    commandQueue = xQueueCreate(QUEUE_SIZE_COMMANDS, sizeof(CommandMessage));
    sensorDataQueue = xQueueCreate(QUEUE_SIZE_SENSOR_DATA, sizeof(SensorReading));
    logQueue = xQueueCreate(QUEUE_SIZE_LOGS, sizeof(LogEntry));
    displayUpdateQueue = xQueueCreate(QUEUE_SIZE_DISPLAY_UPDATES, sizeof(DisplayUpdate));
    
    if (!commandQueue || !sensorDataQueue || !logQueue || !displayUpdateQueue) {
        Serial.println("[TaskManager] Failed to create queues");
        return false;
    }
    
    systemStartTime = millis();
    initialized = true;
    
    Serial.println("[TaskManager] Initialization complete");
    return true;
}

bool TaskManager::createTask(const char* name, TaskFunction_t function, uint32_t stackSize, 
                           UBaseType_t priority, BaseType_t core, void* parameters) {
    if (taskCount >= 8) {
        Serial.println("[TaskManager] Maximum tasks reached");
        return false;
    }
    
    TaskHandle_t handle;
    BaseType_t result;
    
    if (core >= 0) {
        result = xTaskCreatePinnedToCore(function, name, stackSize, parameters, 
                                       priority, &handle, core);
    } else {
        result = xTaskCreate(function, name, stackSize, parameters, priority, &handle);
    }
    
    if (result != pdPASS) {
        Serial.printf("[TaskManager] Failed to create task: %s\n", name);
        return false;
    }
    
    // Store task info
    tasks[taskCount].handle = handle;
    tasks[taskCount].name = name;
    tasks[taskCount].state = TaskState::READY;
    tasks[taskCount].lastRunTime = millis();
    taskCount++;
    
    Serial.printf("[TaskManager] Created task: %s (Core: %d, Priority: %d)\n", 
                 name, core, priority);
    return true;
}

void TaskManager::startAllTasks() {
    if (!initialized) {
        Serial.println("[TaskManager] Not initialized!");
        return;
    }
    
    Serial.println("[TaskManager] Starting all tasks...");
    
    // Create tasks with core assignment
    createTask("CommunicationTask", communicationTask, TASK_STACK_SIZE_LARGE, 
               PRIORITY_HIGH, CORE_COMMUNICATION);
    
    createTask("SensorTask", sensorTask, TASK_STACK_SIZE_MEDIUM, 
               PRIORITY_NORMAL, CORE_CONTROL_DISPLAY);
    
    createTask("PumpControlTask", pumpControlTask, TASK_STACK_SIZE_MEDIUM, 
               PRIORITY_HIGH, CORE_CONTROL_DISPLAY);
    
    createTask("DisplayTask", displayTask, TASK_STACK_SIZE_MEDIUM, 
               PRIORITY_NORMAL, CORE_CONTROL_DISPLAY);
    
    createTask("OTATask", otaTask, TASK_STACK_SIZE_LARGE, 
               PRIORITY_LOW, CORE_COMMUNICATION);
    
    createTask("LoggerTask", loggerTask, TASK_STACK_SIZE_MEDIUM, 
               PRIORITY_LOW, CORE_COMMUNICATION);
    
    createTask("SystemMonitorTask", systemMonitorTask, TASK_STACK_SIZE_SMALL, 
               PRIORITY_LOW, -1); // No core pinning for monitor
    
    Serial.printf("[TaskManager] Started %d tasks\n", taskCount);
}

void TaskManager::suspendTask(const char* name) {
    TaskInfo* task = getTaskInfo(name);
    if (task && task->handle) {
        vTaskSuspend(task->handle);
        task->state = TaskState::SUSPENDED;
        Serial.printf("[TaskManager] Suspended task: %s\n", name);
    }
}

void TaskManager::resumeTask(const char* name) {
    TaskInfo* task = getTaskInfo(name);
    if (task && task->handle) {
        vTaskResume(task->handle);
        task->state = TaskState::RUNNING;
        Serial.printf("[TaskManager] Resumed task: %s\n", name);
    }
}

TaskInfo* TaskManager::getTaskInfo(const char* name) {
    for (uint8_t i = 0; i < taskCount; i++) {
        if (strcmp(tasks[i].name, name) == 0) {
            return &tasks[i];
        }
    }
    return nullptr;
}

bool TaskManager::takeMutex(SemaphoreHandle_t mutex, TickType_t timeout) {
    return xSemaphoreTake(mutex, timeout) == pdTRUE;
}

void TaskManager::giveMutex(SemaphoreHandle_t mutex) {
    xSemaphoreGive(mutex);
}

void TaskManager::updateTaskStats() {
    for (uint8_t i = 0; i < taskCount; i++) {
        if (tasks[i].handle) {
            tasks[i].stackHighWaterMark = uxTaskGetStackHighWaterMark(tasks[i].handle);
        }
    }
}

void TaskManager::printSystemStats() {
    updateTaskStats();
    
    Serial.println("\n========== SYSTEM STATISTICS ==========");
    Serial.printf("Uptime: %lu ms\n", getUptime());
    Serial.printf("Free Heap: %u bytes\n", getFreeHeap());
    Serial.printf("Minimum Free Heap: %u bytes\n", getMinimumFreeHeap());
    Serial.printf("Tasks Running: %d\n", taskCount);
    
    Serial.println("\n---------- TASK INFORMATION ----------");
    for (uint8_t i = 0; i < taskCount; i++) {
        Serial.printf("Task: %-16s Stack Free: %4u bytes State: %d\n", 
                     tasks[i].name, 
                     tasks[i].stackHighWaterMark * 4, // Convert to bytes
                     (int)tasks[i].state);
    }
    
    Serial.println("======================================\n");
}

uint32_t TaskManager::getFreeHeap() {
    return esp_get_free_heap_size();
}

uint32_t TaskManager::getMinimumFreeHeap() {
    return esp_get_minimum_free_heap_size();
}

void TaskManager::delayMs(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void TaskManager::yield() {
    taskYIELD();
}

void TaskManager::feedWatchdog() {
    // Implementation depends on watchdog setup
    // For now, just yield to prevent task starvation
    taskYIELD();
}

TaskManager::~TaskManager() {
    // Clean up resources
    if (systemMutex) vSemaphoreDelete(systemMutex);
    if (displayMutex) vSemaphoreDelete(displayMutex);
    if (communicationMutex) vSemaphoreDelete(communicationMutex);
    
    if (commandQueue) vQueueDelete(commandQueue);
    if (sensorDataQueue) vQueueDelete(sensorDataQueue);
    if (logQueue) vQueueDelete(logQueue);
    if (displayUpdateQueue) vQueueDelete(displayUpdateQueue);
}

// Task implementations are in main.ino 