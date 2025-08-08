#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "../config.h"
#include "../types.h"

class TaskManager {
private:
    static TaskManager* instance;
    
    // Task handles
    TaskInfo tasks[8];
    uint8_t taskCount;
    
    // Synchronization
    SemaphoreHandle_t systemMutex;
    SemaphoreHandle_t displayMutex;
    SemaphoreHandle_t communicationMutex;
    
    // Inter-task queues
    QueueHandle_t commandQueue;
    QueueHandle_t sensorDataQueue;
    QueueHandle_t logQueue;
    QueueHandle_t displayUpdateQueue;
    
    // System state
    bool initialized;
    unsigned long systemStartTime;
    
    TaskManager();

public:
    static TaskManager& getInstance();
    
    // Initialization
    bool initialize();
    void startAllTasks();
    
    // Task management
    bool createTask(const char* name, TaskFunction_t function, uint32_t stackSize, 
                   UBaseType_t priority, BaseType_t core, void* parameters = nullptr);
    void suspendTask(const char* name);
    void resumeTask(const char* name);
    TaskInfo* getTaskInfo(const char* name);
    
    // Queue management
    QueueHandle_t getCommandQueue() { return commandQueue; }
    QueueHandle_t getSensorDataQueue() { return sensorDataQueue; }
    QueueHandle_t getLogQueue() { return logQueue; }
    QueueHandle_t getDisplayUpdateQueue() { return displayUpdateQueue; }
    
    // Synchronization
    bool takeMutex(SemaphoreHandle_t mutex, TickType_t timeout = portMAX_DELAY);
    void giveMutex(SemaphoreHandle_t mutex);
    SemaphoreHandle_t getSystemMutex() { return systemMutex; }
    SemaphoreHandle_t getDisplayMutex() { return displayMutex; }
    SemaphoreHandle_t getCommunicationMutex() { return communicationMutex; }
    
    // System monitoring
    void updateTaskStats();
    void printSystemStats();
    uint32_t getFreeHeap();
    uint32_t getMinimumFreeHeap();
    
    // Utility
    void delayMs(uint32_t ms);
    void yield();
    unsigned long getUptime() { return millis() - systemStartTime; }
    
    // Watchdog
    void feedWatchdog();
    
    ~TaskManager();
};

// Global task functions
extern "C" {
    void communicationTask(void* parameters);
    void sensorTask(void* parameters);  
    void pumpControlTask(void* parameters);
    void displayTask(void* parameters);
    void otaTask(void* parameters);
    void systemMonitorTask(void* parameters);
    void loggerTask(void* parameters);
} 