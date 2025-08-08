#include "CommunicationManager.h"

// ========================================
// WIFI MANAGER IMPLEMENTATION
// ========================================
WiFiManager::WiFiManager() : initialized(false), lastConnectionAttempt(0) {}

bool WiFiManager::initialize() {
    if (initialized) return true;
    
    WiFi.mode(WIFI_STA);
    initialized = true;
    
    Serial.println("[CommunicationManager] WiFi Manager initialized");
    return true;
}

bool WiFiManager::connect() {
    if (!initialized) return false;
    
    if (isConnected()) return true;
    
    unsigned long currentTime = millis();
    if (currentTime - lastConnectionAttempt < RECONNECT_INTERVAL) {
        return false; // Too soon to retry
    }
    
    lastConnectionAttempt = currentTime;
    status.lastConnectAttempt = currentTime;
    status.reconnectCount++;
    
    Serial.println("[WiFiManager] Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    // Non-blocking connection attempt
    return false; // Will be connected on next update
}

void WiFiManager::disconnect() {
    WiFi.disconnect();
    status.connected = false;
}

bool WiFiManager::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

WiFiStatus WiFiManager::getStatus() {
    status.connected = isConnected();
    if (status.connected) {
        status.ssid = WiFi.SSID();
        status.rssi = WiFi.RSSI();
        status.localIP = WiFi.localIP();
    }
    return status;
}

void WiFiManager::update() {
    if (!initialized) return;
    
    bool currentlyConnected = isConnected();
    
    if (!currentlyConnected && !status.connected) {
        // Try to connect if not connected
        connect();
    }
    
    status.connected = currentlyConnected;
}

IPAddress WiFiManager::getLocalIP() {
    return WiFi.localIP();
}

int32_t WiFiManager::getRSSI() {
    return WiFi.RSSI();
}

// ========================================
// BACKEND HTTP CLIENT IMPLEMENTATION
// ========================================
BackendHTTPClient::BackendHTTPClient() {}

bool BackendHTTPClient::sendStatusUpdate(const SystemState& system, const PumpStatus& pump, 
                                       const WaterLevelData& ground, const WaterLevelData& roof) {
    String payload = buildStatusPayload(system, pump, ground, roof);
    String response;
    return sendRequest("/devices/status/update", payload, response);
}

bool BackendHTTPClient::sendPumpEvent(const String& eventType, const String& reason) {
    String payload = buildEventPayload(eventType, reason);
    String response;
    return sendRequest("/devices/events/pump", payload, response);
}

bool BackendHTTPClient::sendLogs(const LogEntry* logs, size_t count) {
    String payload = buildLogsPayload(logs, count);
    String response;
    return sendRequest("/devices/" + String(DEVICE_ID) + "/logs", payload, response);
}

bool BackendHTTPClient::checkForCommands(CommandMessage& command) {
    String response;
    String endpoint = "/devices/pump/command/" + String(DEVICE_ID);
    
    if (sendRequest(endpoint, "", response)) {
        if (response.length() > 0 && response != "null") {
            // Parse command from JSON response
            DynamicJsonDocument doc(512);
            DeserializationError error = deserializeJson(doc, response);
            
            if (!error) {
                const char* action = doc["action"];
                if (action) {
                    if (strcmp(action, "start") == 0) command.command = PumpCommand::START;
                    else if (strcmp(action, "stop") == 0) command.command = PumpCommand::STOP;
                    else if (strcmp(action, "auto") == 0) command.command = PumpCommand::AUTO_MODE;
                    else if (strcmp(action, "manual") == 0) command.command = PumpCommand::MANUAL_MODE;
                    else if (strcmp(action, "reset_usage") == 0) command.command = PumpCommand::RESET_USAGE;
                    
                    command.reason = doc["reason"] | "";
                    command.targetLevel = doc["target_level"] | 0.0f;
                    command.timestamp = millis();
                    return true;
                }
            }
        }
    }
    return false;
}

bool BackendHTTPClient::getOTAInfo(OTAStatus& otaStatus) {
    String response;
    String endpoint = "/devices/" + String(DEVICE_ID) + "/ota/latest";
    
    if (sendRequest(endpoint, "", response)) {
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, response);
        
        if (!error) {
            otaStatus.updateAvailable = true;
            otaStatus.version = doc["version"] | "";
            otaStatus.downloadUrl = doc["firmware_url"] | "";
            return true;
        }
    }
    return false;
}

String BackendHTTPClient::buildStatusPayload(const SystemState& system, const PumpStatus& pump, 
                                           const WaterLevelData& ground, const WaterLevelData& roof) {
    DynamicJsonDocument doc(2048);
    
    doc["device_id"] = DEVICE_ID;
    doc["timestamp"] = millis();
    
    // Ground tank data
    JsonObject groundTank = doc.createNestedObject("ground_tank");
    groundTank["level_percent"] = ground.levelPercent;
    groundTank["level_inches"] = ground.levelInches;
    groundTank["alarm_active"] = ground.alarmActive;
    groundTank["connected"] = ground.connected;
    groundTank["sensor_working"] = ground.sensorWorking;
    groundTank["water_supply_on"] = ground.waterSupplyOn;
    
    // Roof tank data
    JsonObject roofTank = doc.createNestedObject("roof_tank");
    roofTank["level_percent"] = roof.levelPercent;
    roofTank["level_inches"] = roof.levelInches;
    roofTank["alarm_active"] = roof.alarmActive;
    roofTank["connected"] = roof.connected;
    roofTank["sensor_working"] = roof.sensorWorking;
    roofTank["water_supply_on"] = roof.waterSupplyOn;
    
    // Pump data
    JsonObject pumpObj = doc.createNestedObject("pump");
    pumpObj["running"] = pump.running;
    pumpObj["manual_override"] = pump.manualOverride;
    pumpObj["current_amps"] = pump.currentAmps;
    pumpObj["power_watts"] = pump.powerWatts;
    pumpObj["daily_consumption"] = pump.dailyConsumption;
    pumpObj["hourly_consumption"] = pump.hourlyConsumption;
    pumpObj["runtime_minutes"] = pump.runTimeMinutes;
    pumpObj["total_runtime_hours"] = pump.totalRunTimeHours;
    pumpObj["protection_active"] = pump.protectionActive;
    pumpObj["overcurrent_protection"] = pump.overCurrentProtection;
    pumpObj["overtime_protection"] = pump.overTimeProtection;
    
    // System state
    JsonObject systemObj = doc.createNestedObject("system");
    systemObj["auto_mode_enabled"] = system.autoControlEnabled;
    systemObj["manual_pump_control"] = system.manualPumpControl;
    systemObj["water_supply_active"] = system.waterSupplyActive;
    
    String result;
    serializeJson(doc, result);
    return result;
}

String BackendHTTPClient::buildEventPayload(const String& eventType, const String& reason) {
    DynamicJsonDocument doc(512);
    doc["event_type"] = eventType;
    doc["reason"] = reason;
    doc["timestamp"] = millis();
    
    String result;
    serializeJson(doc, result);
    return result;
}

String BackendHTTPClient::buildLogsPayload(const LogEntry* logs, size_t count) {
    DynamicJsonDocument doc(2048);
    JsonArray arr = doc.to<JsonArray>();
    
    for (size_t i = 0; i < count; i++) {
        JsonObject logObj = arr.createNestedObject();
        
        const char* levelStr = "info";
        switch (logs[i].level) {
            case LogLevel::DEBUG: levelStr = "debug"; break;
            case LogLevel::INFO:  levelStr = "info";  break;
            case LogLevel::WARN:  levelStr = "warn";  break;
            case LogLevel::ERROR: levelStr = "error"; break;
        }
        
        logObj["level"] = levelStr;
        logObj["message"] = logs[i].message;
        logObj["tag"] = logs[i].tag;
        logObj["timestamp"] = logs[i].timestamp;
    }
    
    String result;
    serializeJson(arr, result);
    return result;
}

bool BackendHTTPClient::sendRequest(const String& endpoint, const String& payload, String& response) {
    if (WiFi.status() != WL_CONNECTED) {
        status.connected = false;
        return false;
    }
    
    String url = String("http://") + BACKEND_HOST + ":" + BACKEND_PORT + API_BASE_URL + endpoint;
    
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(BACKEND_TIMEOUT);
    
    int httpResponseCode;
    if (payload.isEmpty()) {
        httpResponseCode = http.GET();
    } else {
        httpResponseCode = http.POST(payload);
    }
    
    if (httpResponseCode > 0) {
        response = http.getString();
        status.connected = true;
        status.lastResponse = millis();
        status.errorCount = 0;
        http.end();
        return true;
    } else {
        status.connected = false;
        status.errorCount++;
        http.end();
        return false;
    }
}

// ========================================
// WEBSOCKET CLIENT IMPLEMENTATION
// ========================================
WebSocketClient::WebSocketClient() : initialized(false), connected(false), eventCallback(nullptr) {}

bool WebSocketClient::initialize() {
    if (initialized) return true;
    
    socketIO.begin(WS_HOST, WS_PORT, "/socket.io/?EIO=4");
    socketIO.onEvent(staticEventHandler);
    
    initialized = true;
    Serial.println("[CommunicationManager] WebSocket Client initialized");
    return true;
}

bool WebSocketClient::connect() {
    if (!initialized) return false;
    // Connection is handled automatically by SocketIO client
    return true;
}

void WebSocketClient::disconnect() {
    if (initialized) {
        socketIO.disconnect();
        connected = false;
    }
}

bool WebSocketClient::isConnected() {
    return connected;
}

void WebSocketClient::update() {
    if (initialized) {
        socketIO.loop();
    }
}

bool WebSocketClient::sendOTAProgress(int progress, const String& status) {
    if (!connected) return false;
    
    DynamicJsonDocument doc(256);
    JsonArray array = doc.to<JsonArray>();
    array.add("ota_progress");
    
    JsonObject data = array.createNestedObject();
    data["device_id"] = DEVICE_ID;
    data["progress"] = progress;
    data["status"] = status;
    
    String message;
    serializeJson(doc, message);
    
    socketIO.sendEVENT(message);
    return true;
}

bool WebSocketClient::sendOTAComplete(bool success, const String& version, const String& error) {
    if (!connected) return false;
    
    DynamicJsonDocument doc(256);
    JsonArray array = doc.to<JsonArray>();
    array.add("ota_complete");
    
    JsonObject data = array.createNestedObject();
    data["device_id"] = DEVICE_ID;
    data["success"] = success;
    data["version"] = version;
    if (!error.isEmpty()) {
        data["error"] = error;
    }
    
    String message;
    serializeJson(doc, message);
    
    socketIO.sendEVENT(message);
    return true;
}

void WebSocketClient::setEventCallback(void (*callback)(const String& event, const JsonVariant& data)) {
    eventCallback = callback;
}

void WebSocketClient::staticEventHandler(socketIOmessageType_t type, uint8_t* payload, size_t length) {
    // This is a static method, so we need to get the instance
    // For now, we'll implement a simple global pointer approach
    // In a more sophisticated design, we'd use a registry pattern
}

void WebSocketClient::handleSocketEvent(socketIOmessageType_t type, uint8_t* payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            connected = false;
            Serial.println("[WebSocketClient] Disconnected");
            break;
            
        case sIOtype_CONNECT: {
            connected = true;
            Serial.println("[WebSocketClient] Connected");
            
            // Subscribe to device room
            DynamicJsonDocument doc(128);
            JsonArray array = doc.to<JsonArray>();
            array.add("subscribe_device");
            array.add(String(DEVICE_ID));
            String message;
            serializeJson(doc, message);
            socketIO.sendEVENT(message);
            break;
        }
            
        case sIOtype_EVENT:
            if (eventCallback) {
                DynamicJsonDocument doc(1024);
                DeserializationError err = deserializeJson(doc, payload, length);
                if (!err) {
                    String eventName = doc[0] | "";
                    JsonVariant data = doc[1];
                    eventCallback(eventName, data);
                }
            }
            break;
            
        default:
            break;
    }
}

// ========================================
// COMMUNICATION MANAGER IMPLEMENTATION
// ========================================
CommunicationManager* CommunicationManager::instance = nullptr;

CommunicationManager::CommunicationManager() : initialized(false), lastStatusUpdate(0), 
                                             lastCommandCheck(0), otaEventCallback(nullptr) {
    wifiManager = nullptr;
    httpClient = nullptr;
    webSocketClient = nullptr;
}

CommunicationManager& CommunicationManager::getInstance() {
    if (instance == nullptr) {
        instance = new CommunicationManager();
    }
    return *instance;
}

bool CommunicationManager::initialize() {
    if (initialized) return true;
    
    Serial.println("[CommunicationManager] Initializing...");
    
    // Create communication components
    wifiManager = new WiFiManager();
    httpClient = new BackendHTTPClient();
    webSocketClient = new WebSocketClient();
    
    // Initialize components
    bool success = true;
    success &= wifiManager->initialize();
    success &= webSocketClient->initialize();
    
    if (success) {
        initialized = true;
        Serial.println("[CommunicationManager] Initialization complete");
    } else {
        Serial.println("[CommunicationManager] Initialization failed");
    }
    
    return success;
}

void CommunicationManager::update() {
    if (!initialized) return;
    
    // Update WiFi connection
    if (wifiManager) {
        wifiManager->update();
    }
    
    // Update WebSocket connection
    if (webSocketClient) {
        webSocketClient->update();
        
        // Connect WebSocket if WiFi is connected but WebSocket isn't
        if (isWiFiConnected() && !webSocketClient->isConnected()) {
            webSocketClient->connect();
        }
    }
}

bool CommunicationManager::connectWiFi() {
    if (wifiManager) {
        return wifiManager->connect();
    }
    return false;
}

bool CommunicationManager::isWiFiConnected() {
    if (wifiManager) {
        return wifiManager->isConnected();
    }
    return false;
}

WiFiStatus CommunicationManager::getWiFiStatus() {
    if (wifiManager) {
        return wifiManager->getStatus();
    }
    return WiFiStatus();
}

bool CommunicationManager::sendStatusUpdate(const SystemState& system, const PumpStatus& pump, 
                                           const WaterLevelData& ground, const WaterLevelData& roof) {
    if (httpClient && isWiFiConnected()) {
        return httpClient->sendStatusUpdate(system, pump, ground, roof);
    }
    return false;
}

bool CommunicationManager::sendPumpEvent(const String& eventType, const String& reason) {
    if (httpClient && isWiFiConnected()) {
        return httpClient->sendPumpEvent(eventType, reason);
    }
    return false;
}

bool CommunicationManager::sendLogs(const LogEntry* logs, size_t count) {
    if (httpClient && isWiFiConnected()) {
        return httpClient->sendLogs(logs, count);
    }
    return false;
}

bool CommunicationManager::checkForCommands(CommandMessage& command) {
    if (httpClient && isWiFiConnected()) {
        return httpClient->checkForCommands(command);
    }
    return false;
}

bool CommunicationManager::getOTAInfo(OTAStatus& otaStatus) {
    if (httpClient && isWiFiConnected()) {
        return httpClient->getOTAInfo(otaStatus);
    }
    return false;
}

bool CommunicationManager::sendOTAProgress(int progress, const String& status) {
    if (webSocketClient && webSocketClient->isConnected()) {
        return webSocketClient->sendOTAProgress(progress, status);
    }
    return false;
}

bool CommunicationManager::sendOTAComplete(bool success, const String& version, const String& error) {
    if (webSocketClient && webSocketClient->isConnected()) {
        return webSocketClient->sendOTAComplete(success, version, error);
    }
    return false;
}

void CommunicationManager::setOTAEventCallback(void (*callback)(const String& event, const JsonVariant& data)) {
    otaEventCallback = callback;
    if (webSocketClient) {
        webSocketClient->setEventCallback(callback);
    }
}

BackendStatus CommunicationManager::getBackendStatus() {
    if (httpClient) {
        return httpClient->getBackendStatus();
    }
    return BackendStatus();
}

bool CommunicationManager::isConnected() {
    return isWiFiConnected(); // && (httpClient backend is responding)
}

void CommunicationManager::handleWebSocketEvent(const String& event, const JsonVariant& data) {
    if (otaEventCallback) {
        otaEventCallback(event, data);
    }
}

void CommunicationManager::staticWebSocketEventHandler(const String& event, const JsonVariant& data) {
    if (instance) {
        instance->handleWebSocketEvent(event, data);
    }
}

CommunicationManager::~CommunicationManager() {
    delete wifiManager;
    delete httpClient;
    delete webSocketClient;
} 