#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <SocketIOclient.h>
#include <ArduinoJson.h>
#include "../config.h"
#include "../types.h"

// ========================================
// COMMUNICATION INTERFACES
// ========================================

class IWiFiManager {
public:
    virtual bool initialize() = 0;
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() = 0;
    virtual WiFiStatus getStatus() = 0;
    virtual void update() = 0;
    virtual IPAddress getLocalIP() = 0;
    virtual int32_t getRSSI() = 0;
};

class IHTTPClient {
public:
    virtual bool sendStatusUpdate(const SystemState& system, const PumpStatus& pump, 
                                const WaterLevelData& ground, const WaterLevelData& roof) = 0;
    virtual bool sendPumpEvent(const String& eventType, const String& reason) = 0;
    virtual bool sendLogs(const LogEntry* logs, size_t count) = 0;
    virtual bool checkForCommands(CommandMessage& command) = 0;
    virtual bool getOTAInfo(OTAStatus& otaStatus) = 0;
    virtual BackendStatus getBackendStatus() = 0;
};

class IWebSocketClient {
public:
    virtual bool initialize() = 0;
    virtual bool connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() = 0;
    virtual void update() = 0;
    virtual bool sendOTAProgress(int progress, const String& status) = 0;
    virtual bool sendOTAComplete(bool success, const String& version, const String& error = "") = 0;
    virtual void setEventCallback(void (*callback)(const String& event, const JsonVariant& data)) = 0;
};

// ========================================
// CONCRETE IMPLEMENTATIONS
// ========================================

class WiFiManager : public IWiFiManager {
private:
    WiFiStatus status;
    bool initialized;
    unsigned long lastConnectionAttempt;
    const unsigned long RECONNECT_INTERVAL = 30000; // 30 seconds

public:
    WiFiManager();
    bool initialize() override;
    bool connect() override;
    void disconnect() override;
    bool isConnected() override;
    WiFiStatus getStatus() override;
    void update() override;
    IPAddress getLocalIP() override;
    int32_t getRSSI() override;
};

class BackendHTTPClient : public IHTTPClient {
private:
    ::HTTPClient http;
    BackendStatus status;
    
    String buildStatusPayload(const SystemState& system, const PumpStatus& pump, 
                            const WaterLevelData& ground, const WaterLevelData& roof);
    String buildEventPayload(const String& eventType, const String& reason);
    String buildLogsPayload(const LogEntry* logs, size_t count);
    bool sendRequest(const String& endpoint, const String& payload, String& response);

public:
    BackendHTTPClient();
    bool sendStatusUpdate(const SystemState& system, const PumpStatus& pump, 
                        const WaterLevelData& ground, const WaterLevelData& roof) override;
    bool sendPumpEvent(const String& eventType, const String& reason) override;
    bool sendLogs(const LogEntry* logs, size_t count) override;
    bool checkForCommands(CommandMessage& command) override;
    bool getOTAInfo(OTAStatus& otaStatus) override;
    BackendStatus getBackendStatus() override { return status; }
};

class WebSocketClient : public IWebSocketClient {
private:
    SocketIOclient socketIO;
    bool initialized;
    bool connected;
    void (*eventCallback)(const String& event, const JsonVariant& data);
    
    static void staticEventHandler(socketIOmessageType_t type, uint8_t* payload, size_t length);
    void handleSocketEvent(socketIOmessageType_t type, uint8_t* payload, size_t length);

public:
    WebSocketClient();
    bool initialize() override;
    bool connect() override;
    void disconnect() override;
    bool isConnected() override;
    void update() override;
    bool sendOTAProgress(int progress, const String& status) override;
    bool sendOTAComplete(bool success, const String& version, const String& error = "") override;
    void setEventCallback(void (*callback)(const String& event, const JsonVariant& data)) override;
};

// ========================================
// COMMUNICATION MANAGER
// ========================================

class CommunicationManager {
private:
    static CommunicationManager* instance;
    
    // Communication components
    IWiFiManager* wifiManager;
    IHTTPClient* httpClient;
    IWebSocketClient* webSocketClient;
    
    // State
    bool initialized;
    unsigned long lastStatusUpdate;
    unsigned long lastCommandCheck;
    
    // Event callback
    void (*otaEventCallback)(const String& event, const JsonVariant& data);
    
    CommunicationManager();
    
    // Internal methods
    void handleWebSocketEvent(const String& event, const JsonVariant& data);
    static void staticWebSocketEventHandler(const String& event, const JsonVariant& data);

public:
    static CommunicationManager& getInstance();
    
    bool initialize();
    void update();
    
    // WiFi management
    bool connectWiFi();
    bool isWiFiConnected();
    WiFiStatus getWiFiStatus();
    
    // Backend communication
    bool sendStatusUpdate(const SystemState& system, const PumpStatus& pump, 
                        const WaterLevelData& ground, const WaterLevelData& roof);
    bool sendPumpEvent(const String& eventType, const String& reason);
    bool sendLogs(const LogEntry* logs, size_t count);
    bool checkForCommands(CommandMessage& command);
    
    // OTA communication
    bool getOTAInfo(OTAStatus& otaStatus);
    bool sendOTAProgress(int progress, const String& status);
    bool sendOTAComplete(bool success, const String& version, const String& error = "");
    void setOTAEventCallback(void (*callback)(const String& event, const JsonVariant& data));
    
    // Status
    BackendStatus getBackendStatus();
    bool isConnected(); // Overall connectivity status
    
    ~CommunicationManager();
}; 