/**
 * @file RoboServo.cpp
 * @brief Implementation for ESP32 and ESP8266 servo control
 * 
 * ESP32: Uses LEDC peripheral for PWM
 * ESP8266: Uses analogWrite with custom frequency
 */

#include "RoboServo.h"

// =============================================================================
// Platform-specific includes and variables
// =============================================================================

#if defined(ROBOSERVO_PLATFORM_ESP32)
    #include "driver/ledc.h"
    #include "esp32-hal-ledc.h"
    
    // Thread-safe channel management (ESP32 only)
    static portMUX_TYPE _channelMux = portMUX_INITIALIZER_UNLOCKED;
    #define ROBOSERVO_ENTER_CRITICAL() portENTER_CRITICAL(&_channelMux)
    #define ROBOSERVO_EXIT_CRITICAL()  portEXIT_CRITICAL(&_channelMux)
    
#elif defined(ROBOSERVO_PLATFORM_ESP8266)
    // ESP8266 doesn't need critical sections for single-core
    #define ROBOSERVO_ENTER_CRITICAL()
    #define ROBOSERVO_EXIT_CRITICAL()
#endif

// Static variables for channel management
static uint16_t _channelMask = 0;
static bool _timerInitialized = false;

// ============================================================================
// RoboServo Implementation
// ============================================================================

RoboServo::RoboServo() 
    : _pin(-1), _channel(ROBOSERVO_INVALID_SERVO)
    , _minPulseUs(ROBOSERVO_DEFAULT_MIN_PULSE_US)
    , _maxPulseUs(ROBOSERVO_DEFAULT_MAX_PULSE_US)
    , _currentPulseUs(ROBOSERVO_DEFAULT_CENTER_PULSE_US)
    , _servoType(SERVO_TYPE_180), _maxAngle(180)
    , _frequency(ROBOSERVO_DEFAULT_FREQUENCY), _attached(false)
{}

RoboServo::~RoboServo() {
    if (_attached) detach();
}

// Static helpers
void RoboServo::initTimer() {
    if (!_timerInitialized) {
#if defined(ROBOSERVO_PLATFORM_ESP8266)
        analogWriteFreq(ROBOSERVO_DEFAULT_FREQUENCY);
        analogWriteResolution(ROBOSERVO_PWM_RESOLUTION);
#endif
        _timerInitialized = true;
    }
}

bool RoboServo::isValidPwmPin(int pin) {
    if (pin < 0) return false;
    
#if defined(ROBOSERVO_PLATFORM_ESP8266)
    // ESP8266: GPIO 0-16 can do PWM (except GPIO 6-11 which are flash pins)
    return (pin >= 0 && pin <= 5) || (pin >= 12 && pin <= 16);
#elif defined(CONFIG_IDF_TARGET_ESP32P4)
    // ESP32-P4: GPIO 0-54 (excluding some reserved pins)
    return (pin >= 0 && pin <= 54) && (pin != 24) && (pin != 25);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
    return (pin >= 1 && pin <= 21) || (pin >= 35 && pin <= 45) || (pin == 47) || (pin == 48);
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    return (pin >= 1 && pin <= 21) || (pin == 26) || (pin >= 33 && pin <= 42);
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
    return (pin >= 0 && pin <= 10) || (pin >= 18 && pin <= 21);
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
    return (pin >= 0 && pin <= 23);
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
    return (pin >= 0 && pin <= 14) || (pin >= 25 && pin <= 27);
#elif defined(CONFIG_IDF_TARGET_ESP32)
    return (pin == 2) || (pin == 4) || (pin == 5) || (pin >= 12 && pin <= 19) ||
           (pin >= 21 && pin <= 23) || (pin >= 25 && pin <= 27) || (pin == 32) || (pin == 33);
#else
    return (pin >= 0 && pin <= 48);
#endif
}

uint8_t RoboServo::allocateChannel() {
    ROBOSERVO_ENTER_CRITICAL();
    for (uint8_t i = 0; i < ROBOSERVO_MAX_SERVOS; i++) {
        if (!(_channelMask & (1 << i))) {
            _channelMask |= (1 << i);
            ROBOSERVO_EXIT_CRITICAL();
            return i;
        }
    }
    ROBOSERVO_EXIT_CRITICAL();
    return ROBOSERVO_INVALID_SERVO;
}

void RoboServo::releaseChannel(uint8_t channel) {
    if (channel >= ROBOSERVO_MAX_SERVOS) return;
    ROBOSERVO_ENTER_CRITICAL();
    _channelMask &= ~(1 << channel);
    ROBOSERVO_EXIT_CRITICAL();
}

int RoboServo::getAttachedCount() {
    int count = 0;
    uint16_t mask = _channelMask;
    while (mask) { count += (mask & 1); mask >>= 1; }
    return count;
}

// Attachment methods
uint8_t RoboServo::attach(int pin) {
    return attach(pin, ROBOSERVO_DEFAULT_MIN_PULSE_US, ROBOSERVO_DEFAULT_MAX_PULSE_US);
}

uint8_t RoboServo::attach(int pin, int minPulseUs, int maxPulseUs) {
    return attach(pin, minPulseUs, maxPulseUs, SERVO_TYPE_180);
}

uint8_t RoboServo::attach(int pin, int minPulseUs, int maxPulseUs, int maxAngle) {
    uint8_t result = attach(pin, minPulseUs, maxPulseUs, SERVO_TYPE_CUSTOM);
    if (result != ROBOSERVO_INVALID_SERVO) _maxAngle = maxAngle;
    return result;
}

uint8_t RoboServo::attach(int pin, int minPulseUs, int maxPulseUs, RoboServoType servoType) {
    return attach(pin, minPulseUs, maxPulseUs, servoType, _frequency);
}

uint8_t RoboServo::attach(int pin, int minPulseUs, int maxPulseUs, RoboServoType servoType, int frequency) {
    // Detach if already attached to different pin
    if (_attached) {
        if (_pin == pin) {
            _minPulseUs = minPulseUs;
            _maxPulseUs = maxPulseUs;
            _servoType = servoType;
            _maxAngle = (servoType == SERVO_TYPE_CUSTOM) ? _maxAngle : (int)servoType;
            return _channel;
        }
        detach();
    }
    
    if (pin < 0 || !isValidPwmPin(pin)) return ROBOSERVO_INVALID_SERVO;
    
    // Validate and store frequency
    _frequency = RoboServoUtils::constrainValue(frequency, ROBOSERVO_MIN_FREQUENCY, ROBOSERVO_MAX_FREQUENCY);
    
    initTimer();
    _channel = allocateChannel();
    if (_channel == ROBOSERVO_INVALID_SERVO) return ROBOSERVO_INVALID_SERVO;
    
    _pin = pin;
    _minPulseUs = minPulseUs;
    _maxPulseUs = maxPulseUs;
    _servoType = servoType;
    _maxAngle = (servoType == SERVO_TYPE_CUSTOM) ? _maxAngle : (int)servoType;
    
#if defined(ROBOSERVO_PLATFORM_ESP8266)
    // ESP8266: Use analogWrite with frequency setting
    analogWriteFreq(_frequency);
    pinMode(_pin, OUTPUT);
#elif defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    // ESP32 Arduino Core 3.x
    if (!ledcAttach(_pin, _frequency, ROBOSERVO_PWM_RESOLUTION)) {
        releaseChannel(_channel);
        _channel = ROBOSERVO_INVALID_SERVO;
        _pin = -1;
        return ROBOSERVO_INVALID_SERVO;
    }
#else
    // ESP32 Arduino Core 2.x
    double actualFreq = ledcSetup(_channel, _frequency, ROBOSERVO_PWM_RESOLUTION);
    if (actualFreq == 0) {
        releaseChannel(_channel);
        _channel = ROBOSERVO_INVALID_SERVO;
        _pin = -1;
        return ROBOSERVO_INVALID_SERVO;
    }
    ledcAttachPin(_pin, _channel);
#endif
    
    _attached = true;
    // FERRO patch: do NOT output the 1500us center pulse on attach.
    // The original library snapped the servo to 90 degrees at full speed on every
    // attach, causing a large current spike. The caller is responsible for issuing
    // a write() right after attach() to establish the desired position.
    _currentPulseUs = 0;

    return _channel;
}

void RoboServo::detach() {
    if (!_attached) return;
    
#if defined(ROBOSERVO_PLATFORM_ESP8266)
    analogWrite(_pin, 0);
    pinMode(_pin, INPUT);
#elif defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(_pin, 0);
    ledcDetach(_pin);
#else
    ledcWrite(_channel, 0);
    ledcDetachPin(_pin);
#endif
    
    releaseChannel(_channel);
    _pin = -1;
    _channel = ROBOSERVO_INVALID_SERVO;
    _currentPulseUs = 0;
    _attached = false;
}

bool RoboServo::attached() const { return _attached; }

// Position control
void RoboServo::write(int angle) {
    if (!_attached) return;
    
    if (angle < 0) angle = 0;
    else if (angle > _maxAngle) angle = _maxAngle;
    
    int pulseUs;
    if (_servoType == SERVO_TYPE_360) {
        int mappedAngle = (angle > 180) ? 180 : angle;
        pulseUs = RoboServoUtils::mapValue(mappedAngle, 0, 180, _minPulseUs, _maxPulseUs);
    } else {
        pulseUs = RoboServoUtils::mapValue(angle, 0, _maxAngle, _minPulseUs, _maxPulseUs);
    }
    writeMicroseconds(pulseUs);
}

void RoboServo::writeMicroseconds(int pulseUs) {
    if (!_attached) return;
    
    pulseUs = RoboServoUtils::constrainValue(pulseUs, _minPulseUs, _maxPulseUs);
    _currentPulseUs = pulseUs;
    uint32_t duty = microsecondsToTicks(pulseUs);
    
#if defined(ROBOSERVO_PLATFORM_ESP8266)
    analogWrite(_pin, duty);
#elif defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(_pin, duty);
#else
    ledcWrite(_channel, duty);
#endif
}

int RoboServo::read() const {
    if (!_attached) return 0;
    if (_servoType == SERVO_TYPE_360) {
        return RoboServoUtils::mapValue(_currentPulseUs, _minPulseUs, _maxPulseUs, 0, 180);
    }
    return RoboServoUtils::mapValue(_currentPulseUs, _minPulseUs, _maxPulseUs, 0, _maxAngle);
}

int RoboServo::readMicroseconds() const { return _currentPulseUs; }

// Configuration
void RoboServo::setServoType(RoboServoType servoType) {
    _servoType = servoType;
    if (servoType != SERVO_TYPE_CUSTOM) _maxAngle = (int)servoType;
}

RoboServoType RoboServo::getServoType() const { return _servoType; }

void RoboServo::setMaxAngle(int maxAngle) {
    if (maxAngle > 0) {
        _maxAngle = maxAngle;
        _servoType = SERVO_TYPE_CUSTOM;
    }
}

int RoboServo::getMaxAngle() const { return _maxAngle; }
void RoboServo::setPulseLimits(int minPulseUs, int maxPulseUs) { _minPulseUs = minPulseUs; _maxPulseUs = maxPulseUs; }
int RoboServo::getPin() const { return _pin; }
uint8_t RoboServo::getChannel() const { return _channel; }
int RoboServo::getMinPulse() const { return _minPulseUs; }
int RoboServo::getMaxPulse() const { return _maxPulseUs; }

void RoboServo::setFrequency(int frequency) {
    _frequency = RoboServoUtils::constrainValue(frequency, ROBOSERVO_MIN_FREQUENCY, ROBOSERVO_MAX_FREQUENCY);
    if (_attached) {
        // Re-attach with new frequency
        int savedPulse = _currentPulseUs;
        detach();
        attach(_pin, _minPulseUs, _maxPulseUs, _servoType, _frequency);
        writeMicroseconds(savedPulse);
    }
}

int RoboServo::getFrequency() const { return _frequency; }

// 360° servo methods
void RoboServo::stop() {
    if (_attached) writeMicroseconds(ROBOSERVO_DEFAULT_CENTER_PULSE_US);
}

void RoboServo::release() {
    if (!_attached) return;
#if defined(ROBOSERVO_PLATFORM_ESP8266)
    analogWrite(_pin, 0);
#elif defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(_pin, 0);
#else
    ledcWrite(_channel, 0);
#endif
}

void RoboServo::setSpeed(int speed) {
    if (!_attached) return;
    speed = RoboServoUtils::constrainValue(speed, -100, 100);
    int pulseUs = RoboServoUtils::mapValue(speed, -100, 100, _minPulseUs, _maxPulseUs);
    writeMicroseconds(pulseUs);
}

uint32_t RoboServo::microsecondsToTicks(int us) const {
    uint32_t maxDuty = (1UL << ROBOSERVO_PWM_RESOLUTION) - 1;
    uint32_t periodUs = 1000000UL / _frequency;
    return (uint32_t)(((uint64_t)us * maxDuty) / periodUs);
}

// ============================================================================
// RoboServoGroup Implementation
// ============================================================================

RoboServoGroup::RoboServoGroup() : _count(0) {
    for (int i = 0; i < ROBOSERVO_MAX_SERVOS; i++) _servos[i] = nullptr;
}

RoboServoGroup::~RoboServoGroup() { removeAll(); }

int RoboServoGroup::addServo(int pin) {
    return addServo(pin, ROBOSERVO_DEFAULT_MIN_PULSE_US, ROBOSERVO_DEFAULT_MAX_PULSE_US, SERVO_TYPE_180);
}

int RoboServoGroup::addServo(int pin, int minPulseUs, int maxPulseUs, RoboServoType servoType) {
    if (_count >= ROBOSERVO_MAX_SERVOS) return -1;
    
    int index = -1;
    for (int i = 0; i < ROBOSERVO_MAX_SERVOS; i++) {
        if (_servos[i] == nullptr) { index = i; break; }
    }
    if (index == -1) return -1;
    
    RoboServo* servo = new RoboServo();
    if (servo == nullptr) return -1;
    
    if (servo->attach(pin, minPulseUs, maxPulseUs, servoType) == ROBOSERVO_INVALID_SERVO) {
        delete servo;
        return -1;
    }
    
    _servos[index] = servo;
    _count++;
    return index;
}

bool RoboServoGroup::removeServo(int index) {
    if (index < 0 || index >= ROBOSERVO_MAX_SERVOS || _servos[index] == nullptr) return false;
    _servos[index]->detach();
    delete _servos[index];
    _servos[index] = nullptr;
    _count--;
    return true;
}

void RoboServoGroup::removeAll() {
    for (int i = 0; i < ROBOSERVO_MAX_SERVOS; i++) {
        if (_servos[i] != nullptr) {
            _servos[i]->detach();
            delete _servos[i];
            _servos[i] = nullptr;
        }
    }
    _count = 0;
}

int RoboServoGroup::count() const { return _count; }

RoboServo* RoboServoGroup::getServo(int index) {
    if (index < 0 || index >= ROBOSERVO_MAX_SERVOS) return nullptr;
    return _servos[index];
}

void RoboServoGroup::writeAll(int angle) {
    for (int i = 0; i < ROBOSERVO_MAX_SERVOS; i++)
        if (_servos[i]) _servos[i]->write(angle);
}

void RoboServoGroup::writeAllMicroseconds(int pulseUs) {
    for (int i = 0; i < ROBOSERVO_MAX_SERVOS; i++)
        if (_servos[i]) _servos[i]->writeMicroseconds(pulseUs);
}

void RoboServoGroup::writeMultiple(const int* angles, int count) {
    if (!angles) return;
    int ai = 0;
    for (int i = 0; i < ROBOSERVO_MAX_SERVOS && ai < count; i++)
        if (_servos[i]) _servos[i]->write(angles[ai++]);
}

void RoboServoGroup::writeMultipleMicroseconds(const int* pulses, int count) {
    if (!pulses) return;
    int pi = 0;
    for (int i = 0; i < ROBOSERVO_MAX_SERVOS && pi < count; i++)
        if (_servos[i]) _servos[i]->writeMicroseconds(pulses[pi++]);
}

void RoboServoGroup::write(int index, int angle) {
    if (index >= 0 && index < ROBOSERVO_MAX_SERVOS && _servos[index])
        _servos[index]->write(angle);
}

void RoboServoGroup::writeMicroseconds(int index, int pulseUs) {
    if (index >= 0 && index < ROBOSERVO_MAX_SERVOS && _servos[index])
        _servos[index]->writeMicroseconds(pulseUs);
}

int RoboServoGroup::read(int index) const {
    if (index >= 0 && index < ROBOSERVO_MAX_SERVOS && _servos[index])
        return _servos[index]->read();
    return -1;
}

void RoboServoGroup::detachAll() {
    for (int i = 0; i < ROBOSERVO_MAX_SERVOS; i++)
        if (_servos[i]) _servos[i]->detach();
}

void RoboServoGroup::stopAll() {
    for (int i = 0; i < ROBOSERVO_MAX_SERVOS; i++)
        if (_servos[i]) _servos[i]->stop();
}

// Utility functions
namespace RoboServoUtils {

int mapValue(int value, int inMin, int inMax, int outMin, int outMax) {
    if (inMax == inMin) return outMin;
    return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

int constrainValue(int value, int minVal, int maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

}
