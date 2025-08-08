#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <RF24.h>
#include "../config.h"
#include "../types.h"

// ========================================
// HARDWARE ABSTRACTION INTERFACES
// ========================================

class IDisplay {
public:
    virtual bool initialize() = 0;
    virtual void clearBuffer() = 0;
    virtual void sendBuffer() = 0;
    virtual void setFont(const uint8_t* font) = 0;
    virtual void drawStr(uint8_t x, uint8_t y, const char* str) = 0;
    virtual void drawBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h) = 0;
    virtual void drawFrame(uint8_t x, uint8_t y, uint8_t w, uint8_t h) = 0;
    virtual void drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) = 0;
    virtual uint8_t getStrWidth(const char* str) = 0;
    virtual void setDrawColor(uint8_t color) = 0;
};

class IRGB {
public:
    virtual bool initialize() = 0;
    virtual void setColor(const RGBColor& color) = 0;
    virtual void setColorWithBrightness(const RGBColor& color, uint8_t brightness) = 0;
    virtual void setBrightness(uint8_t brightness) = 0;
    virtual void turnOff() = 0;
};

class IAudio {
public:
    virtual bool initialize() = 0;
    virtual void playBeep(uint16_t duration) = 0;
    virtual void playAlert() = 0;
    virtual void setMuted(bool muted) = 0;
    virtual bool isMuted() = 0;
    virtual void update() = 0; // Non-blocking audio processing
};

class IEncoder {
public:
    virtual bool initialize() = 0;
    virtual int getPosition() = 0;
    virtual void resetPosition() = 0;
    virtual bool isButtonPressed() = 0;
    virtual void setInterruptCallback(void (*callback)()) = 0;
};

class IRadio {
public:
    virtual bool initialize() = 0;
    virtual bool isAvailable() = 0;
    virtual bool read(void* buffer, uint8_t length) = 0;
    virtual void startListening() = 0;
    virtual void stopListening() = 0;
    virtual void setPower(uint8_t level) = 0;
    virtual void setChannel(uint8_t channel) = 0;
};

class IPumpControl {
public:
    virtual bool initialize() = 0;
    virtual void turnOn() = 0;
    virtual void turnOff() = 0;
    virtual bool isRunning() = 0;
    virtual float readCurrent() = 0;
    virtual bool hasCurrentSensor() = 0;
    virtual bool hasVoltageSensor() = 0;
};

// ========================================
// CONCRETE IMPLEMENTATIONS
// ========================================

class OLEDDisplay : public IDisplay {
private:
    U8G2_SH1106_128X64_NONAME_F_HW_I2C* display;
    bool initialized;

public:
    OLEDDisplay();
    bool initialize() override;
    void clearBuffer() override;
    void sendBuffer() override;
    void setFont(const uint8_t* font) override;
    void drawStr(uint8_t x, uint8_t y, const char* str) override;
    void drawBox(uint8_t x, uint8_t y, uint8_t w, uint8_t h) override;
    void drawFrame(uint8_t x, uint8_t y, uint8_t w, uint8_t h) override;
    void drawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) override;
    uint8_t getStrWidth(const char* str) override;
    void setDrawColor(uint8_t color) override;
    ~OLEDDisplay();
};

class RGBLed : public IRGB {
private:
    bool initialized;
    uint8_t currentBrightness;
    RGBColor currentColor;

public:
    RGBLed();
    bool initialize() override;
    void setColor(const RGBColor& color) override;
    void setColorWithBrightness(const RGBColor& color, uint8_t brightness) override;
    void setBrightness(uint8_t brightness) override;
    void turnOff() override;
};

class BuzzerAudio : public IAudio {
private:
    bool initialized;
    bool muted;
    AudioState state;
    unsigned long startTime;
    unsigned long stepTime;
    uint8_t currentStep;
    uint8_t duration;
    uint8_t repeatCount;

public:
    BuzzerAudio();
    bool initialize() override;
    void playBeep(uint16_t duration) override;
    void playAlert() override;
    void setMuted(bool muted) override;
    bool isMuted() override;
    void update() override;
};

class RotaryEncoder : public IEncoder {
private:
    volatile int position;
    volatile bool lastClkState;
    int lastReadPosition;
    bool buttonState;
    bool lastButtonState;
    unsigned long lastDebounceTime;
    void (*interruptCallback)();

public:
    RotaryEncoder();
    bool initialize() override;
    int getPosition() override;
    void resetPosition() override;
    bool isButtonPressed() override;
    void setInterruptCallback(void (*callback)()) override;
    void handleInterrupt(); // Called from ISR
};

class NRF24Radio : public IRadio {
private:
    RF24* radio;
    bool initialized;

public:
    NRF24Radio();
    bool initialize() override;
    bool isAvailable() override;
    bool read(void* buffer, uint8_t length) override;
    void startListening() override;
    void stopListening() override;
    void setPower(uint8_t level) override;
    void setChannel(uint8_t channel) override;
    ~NRF24Radio();
};

class RelayPumpControl : public IPumpControl {
private:
    bool initialized;
    bool running;
    bool hasCurrentSensorConnected;
    bool hasVoltageSensorConnected;

public:
    RelayPumpControl();
    bool initialize() override;
    void turnOn() override;
    void turnOff() override;
    bool isRunning() override;
    float readCurrent() override;
    bool hasCurrentSensor() override;
    bool hasVoltageSensor() override;
};

// ========================================
// HARDWARE MANAGER
// ========================================

class HardwareManager {
private:
    static HardwareManager* instance;
    
    // Hardware interfaces
    IDisplay* display;
    IRGB* rgb;
    IAudio* audio;
    IEncoder* encoder;
    IRadio* radio;
    IPumpControl* pump;
    
    bool initialized;
    
    HardwareManager();

public:
    static HardwareManager& getInstance();
    
    bool initializeAll();
    
    // Accessors
    IDisplay* getDisplay() { return display; }
    IRGB* getRGB() { return rgb; }
    IAudio* getAudio() { return audio; }
    IEncoder* getEncoder() { return encoder; }
    IRadio* getRadio() { return radio; }
    IPumpControl* getPump() { return pump; }
    
    // Utility
    bool isInitialized() { return initialized; }
    void updateNonBlockingHardware(); // Called from tasks
    
    ~HardwareManager();
}; 