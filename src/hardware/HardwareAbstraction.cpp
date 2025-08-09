#include "HardwareAbstraction.h"

// ========================================
// OLED DISPLAY IMPLEMENTATION
// ========================================
OLEDDisplay::OLEDDisplay() : initialized(false) {
    display = new U8G2_SH1106_128X64_NONAME_F_HW_I2C(U8G2_R0, U8X8_PIN_NONE);
}

bool OLEDDisplay::initialize() {
    if (initialized) return true;
    
    // Check if display object was created successfully
    if (!display) {
        Serial.println("[HardwareAbstraction] OLED Display object creation failed");
        return false;
    }
    
    // Initialize I2C with error checking
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);  // Give I2C time to initialize
    
    // Initialize display with error handling
    if (!display->begin()) {
        Serial.println("[HardwareAbstraction] OLED Display begin() failed");
        return false;
    }
    
    // Clear display and test
    display->clearBuffer();
    display->setFont(u8g2_font_6x10_tf);
    display->drawStr(0, 10, "Init OK");
    display->sendBuffer();
    
    initialized = true;
    Serial.println("[HardwareAbstraction] OLED Display initialized");
    return true;
}

void OLEDDisplay::clearBuffer() {
    if (display && initialized) display->clearBuffer();
}

void OLEDDisplay::sendBuffer() {
    if (display && initialized) display->sendBuffer();
}

void OLEDDisplay::setFont(const uint8_t* font) {
    if (display && initialized) display->setFont(font);
}

void OLEDDisplay::drawStr(uint8_t x, uint8_t y, const char* str) {
    if (display && initialized) display->drawStr(x, y, str);
}

void OLEDDisplay::drawBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    if (display && initialized) display->drawBox(x, y, w, h);
}

void OLEDDisplay::drawFrame(uint8_t x, uint8_t y, uint8_t w, uint8_t h) {
    if (display && initialized) display->drawFrame(x, y, w, h);
}

void OLEDDisplay::drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    if (display && initialized) display->drawLine(x0, y0, x1, y1);
}

uint8_t OLEDDisplay::getStrWidth(const char* str) {
    if (display && initialized) return display->getStrWidth(str);
    return 0;
}

void OLEDDisplay::setDrawColor(uint8_t color) {
    if (display && initialized) display->setDrawColor(color);
}

OLEDDisplay::~OLEDDisplay() {
    delete display;
}

// ========================================
// RGB LED IMPLEMENTATION
// ========================================
RGBLed::RGBLed() : initialized(false), currentBrightness(255), currentColor(0, 0, 0) {}

bool RGBLed::initialize() {
    if (initialized) return true;
    
    pinMode(RGB_RED_PIN, OUTPUT);
    pinMode(RGB_GREEN_PIN, OUTPUT);
    pinMode(RGB_BLUE_PIN, OUTPUT);
    
    turnOff();
    initialized = true;
    
    Serial.println("[HardwareAbstraction] RGB LED initialized");
    return true;
}

void RGBLed::setColor(const RGBColor& color) {
    if (!initialized) return;
    
    currentColor = color;
    analogWrite(RGB_RED_PIN, color.red);
    analogWrite(RGB_GREEN_PIN, color.green);
    analogWrite(RGB_BLUE_PIN, color.blue);
}

void RGBLed::setColorWithBrightness(const RGBColor& color, uint8_t brightness) {
    if (!initialized) return;
    
    currentColor = color;
    currentBrightness = brightness;
    
    analogWrite(RGB_RED_PIN, (color.red * brightness) / 255);
    analogWrite(RGB_GREEN_PIN, (color.green * brightness) / 255);
    analogWrite(RGB_BLUE_PIN, (color.blue * brightness) / 255);
}

void RGBLed::setBrightness(uint8_t brightness) {
    currentBrightness = brightness;
    setColorWithBrightness(currentColor, brightness);
}

void RGBLed::turnOff() {
    setColor(RGBColor(0, 0, 0));
}

// ========================================
// BUZZER AUDIO IMPLEMENTATION
// ========================================
BuzzerAudio::BuzzerAudio() : initialized(false), muted(false), state(AudioState::IDLE),
                            startTime(0), stepTime(0), currentStep(0), duration(0), repeatCount(0) {}

bool BuzzerAudio::initialize() {
    if (initialized) return true;
    
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    initialized = true;
    
    Serial.println("[HardwareAbstraction] Buzzer Audio initialized");
    return true;
}

void BuzzerAudio::playBeep(uint16_t duration) {
    if (!initialized || muted) return;
    
    if (state == AudioState::IDLE) {
        state = AudioState::BEEP;
        startTime = millis();
        this->duration = duration;
        digitalWrite(BUZZER_PIN, HIGH);
    }
}

void BuzzerAudio::playAlert() {
    if (!initialized || muted) return;
    
    if (state == AudioState::IDLE) {
        state = AudioState::ALERT;
        startTime = millis();
        stepTime = millis();
        currentStep = 0;
        repeatCount = 0;
        digitalWrite(BUZZER_PIN, HIGH);
    }
}

void BuzzerAudio::setMuted(bool muted) {
    this->muted = muted;
    if (muted) {
        digitalWrite(BUZZER_PIN, LOW);
        state = AudioState::IDLE;
    }
}

bool BuzzerAudio::isMuted() {
    return muted;
}

void BuzzerAudio::update() {
    if (!initialized || muted) return;
    
    unsigned long currentTime = millis();
    
    switch (state) {
        case AudioState::BEEP:
            if (currentTime - startTime >= duration) {
                digitalWrite(BUZZER_PIN, LOW);
                state = AudioState::IDLE;
            }
            break;
            
        case AudioState::ALERT:
            if (currentStep == 0) {
                if (currentTime - stepTime >= 100) {
                    digitalWrite(BUZZER_PIN, LOW);
                    currentStep = 1;
                    stepTime = currentTime;
                }
            } else {
                if (currentTime - stepTime >= 50) {
                    repeatCount++;
                    if (repeatCount < 3) {
                        digitalWrite(BUZZER_PIN, HIGH);
                        currentStep = 0;
                        stepTime = currentTime;
                    } else {
                        state = AudioState::IDLE;
                    }
                }
            }
            break;
            
        default:
            state = AudioState::IDLE;
            break;
    }
}

// ========================================
// ROTARY ENCODER IMPLEMENTATION
// ========================================
RotaryEncoder::RotaryEncoder() : position(0), lastClkState(HIGH), lastReadPosition(0),
                                buttonState(false), lastButtonState(HIGH), lastDebounceTime(0),
                                interruptCallback(nullptr) {}

bool RotaryEncoder::initialize() {
    pinMode(ENCODER_CLK, INPUT_PULLUP);
    pinMode(ENCODER_DT, INPUT_PULLUP);
    pinMode(ENCODER_SW, INPUT_PULLUP);
    
    lastClkState = digitalRead(ENCODER_CLK);
    
    Serial.println("[HardwareAbstraction] Rotary Encoder initialized");
    return true;
}

int RotaryEncoder::getPosition() {
    int currentPos = position;
    lastReadPosition = currentPos;
    return currentPos;
}

void RotaryEncoder::resetPosition() {
    position = 0;
    lastReadPosition = 0;
}

bool RotaryEncoder::isButtonPressed() {
    unsigned long currentTime = millis();
    int reading = digitalRead(ENCODER_SW);
    
    if (reading != lastButtonState) {
        lastDebounceTime = currentTime;
    }
    
    if ((currentTime - lastDebounceTime) > 50) { // 50ms debounce
        if (reading != buttonState) {
            buttonState = reading;
            if (buttonState == LOW) { // Pressed
                lastButtonState = reading;
                return true;
            }
        }
    }
    
    lastButtonState = reading;
    return false;
}

void RotaryEncoder::setInterruptCallback(void (*callback)()) {
    interruptCallback = callback;
    attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), callback, CHANGE);
}

void RotaryEncoder::handleInterrupt() {
    bool clkState = digitalRead(ENCODER_CLK);
    bool dtState = digitalRead(ENCODER_DT);
    
    if (clkState != lastClkState) {
        if (dtState != clkState) {
            position++;
        } else {
            position--;
        }
        lastClkState = clkState;
    }
}

// ========================================
// NRF24 RADIO IMPLEMENTATION
// ========================================
NRF24Radio::NRF24Radio() : initialized(false) {
    radio = new RF24(CE_PIN, CSN_PIN);
}

bool NRF24Radio::initialize() {
    if (initialized) return true;
    
    SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CSN_PIN);
    
    if (!radio->begin()) {
        Serial.println("[HardwareAbstraction] NRF24 Radio initialization failed");
        return false;
    }
    
    radio->setPALevel(RF24_PA_HIGH);
    radio->setDataRate(RF24_250KBPS);
    radio->setChannel(76);
    
    radio->openReadingPipe(1, RX_ADDR_GROUND_TANK);
    radio->openReadingPipe(2, RX_ADDR_ROOF_TANK);
    radio->startListening();
    
    initialized = true;
    Serial.println("[HardwareAbstraction] NRF24 Radio initialized");
    return true;
}

bool NRF24Radio::isAvailable() {
    return initialized && radio && radio->available();
}

bool NRF24Radio::read(void* buffer, uint8_t length) {
    if (!initialized || !radio) return false;
    
    radio->read(buffer, length);
    return true;
}

void NRF24Radio::startListening() {
    if (initialized && radio) radio->startListening();
}

void NRF24Radio::stopListening() {
    if (initialized && radio) radio->stopListening();
}

void NRF24Radio::setPower(uint8_t level) {
    if (initialized && radio) radio->setPALevel(level);
}

void NRF24Radio::setChannel(uint8_t channel) {
    if (initialized && radio) radio->setChannel(channel);
}

NRF24Radio::~NRF24Radio() {
    delete radio;
}

// ========================================
// RELAY PUMP CONTROL IMPLEMENTATION
// ========================================
RelayPumpControl::RelayPumpControl() : initialized(false), running(false),
                                      hasCurrentSensorConnected(false), hasVoltageSensorConnected(false) {}

bool RelayPumpControl::initialize() {
    if (initialized) return true;
    
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // Start OFF
    
    // Check if sensors are connected
    if (CURRENT_SENSOR_PIN != -1) {
        pinMode(CURRENT_SENSOR_PIN, INPUT);
        hasCurrentSensorConnected = true;
    }
    
    if (VOLTAGE_DETECT_PIN != -1) {
        pinMode(VOLTAGE_DETECT_PIN, INPUT);
        hasVoltageSensorConnected = true;
    }
    
    initialized = true;
    Serial.println("[HardwareAbstraction] Relay Pump Control initialized");
    return true;
}

void RelayPumpControl::turnOn() {
    if (!initialized) return;
    
    digitalWrite(RELAY_PIN, HIGH);
    running = true;
    Serial.println("[HardwareAbstraction] Pump turned ON");
}

void RelayPumpControl::turnOff() {
    if (!initialized) return;
    
    digitalWrite(RELAY_PIN, LOW);
    running = false;
    Serial.println("[HardwareAbstraction] Pump turned OFF");
}

bool RelayPumpControl::isRunning() {
    return running;
}

float RelayPumpControl::readCurrent() {
    if (!hasCurrentSensorConnected) {
        // Return estimated current based on pump state
        return running ? 5.0f : 0.0f;
    }
    
    // ACS712 30A: 2.5V = 0A, 66mV per Amp
    float voltage = (analogRead(CURRENT_SENSOR_PIN) * 3.3f) / 4095.0f;
    float current = (voltage - 1.65f) / 0.066f;
    
    // Validate reading
    if (current < 0 || current > 50) {
        return running ? 5.0f : 0.0f; // Fallback
    }
    
    return abs(current);
}

bool RelayPumpControl::hasCurrentSensor() {
    return hasCurrentSensorConnected;
}

bool RelayPumpControl::hasVoltageSensor() {
    return hasVoltageSensorConnected;
}

// ========================================
// HARDWARE MANAGER IMPLEMENTATION
// ========================================
HardwareManager* HardwareManager::instance = nullptr;

HardwareManager::HardwareManager() : initialized(false) {
    display = nullptr;
    rgb = nullptr;
    audio = nullptr;
    encoder = nullptr;
    radio = nullptr;
    pump = nullptr;
}

HardwareManager& HardwareManager::getInstance() {
    if (instance == nullptr) {
        instance = new HardwareManager();
    }
    return *instance;
}

bool HardwareManager::initializeAll() {
    if (initialized) return true;
    
    Serial.println("[HardwareManager] Initializing all hardware...");
    
    // Create hardware instances
    display = new OLEDDisplay();
    rgb = new RGBLed();
    audio = new BuzzerAudio();
    encoder = new RotaryEncoder();
    radio = new NRF24Radio();
    pump = new RelayPumpControl();
    
    // Initialize all hardware
    bool success = true;
    success &= display->initialize();
    success &= rgb->initialize();
    success &= audio->initialize();
    success &= encoder->initialize();
    success &= radio->initialize();
    success &= pump->initialize();
    
    if (success) {
        initialized = true;
        Serial.println("[HardwareManager] All hardware initialized successfully");
    } else {
        Serial.println("[HardwareManager] Some hardware failed to initialize");
    }
    
    return success;
}

void HardwareManager::updateNonBlockingHardware() {
    if (!initialized) return;
    
    // Update non-blocking hardware components
    if (audio) audio->update();
    // RGB LED updates would go here if they had animations
}

HardwareManager::~HardwareManager() {
    delete display;
    delete rgb;
    delete audio;
    delete encoder;
    delete radio;
    delete pump;
} 