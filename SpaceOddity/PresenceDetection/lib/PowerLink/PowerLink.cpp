/**
 * PowerLink.cpp — Implementation of the ESP32-C3 <-> CH32V003 UART power link.
 */

#include "PowerLink.h"

PowerLink::PowerLink(int rxPin, int txPin, unsigned long baud)
    : _rxPin(rxPin)
    , _txPin(txPin)
    , _baud(baud)
    , _connected(false)
    , _currentState(0)
    , _previousState(0)
    , _changedBits(0)
    , _callback(nullptr)
{}

void PowerLink::begin() {
    Serial1.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    performHandshake();
    _connected     = true;
    _currentState  = 0;
    _previousState = 0;
    _changedBits   = 0;
}

void PowerLink::performHandshake() {
    while (true) {
        Serial1.write(MSG_INIT);
        unsigned long deadline = millis() + 100;
        while (millis() < deadline) {
            if (Serial1.available() && (uint8_t)Serial1.read() == MSG_READY) {
                return;
            }
        }
    }
}

void PowerLink::update() {
    // Clear per-frame change tracking.  If multiple bytes arrive in one update()
    // call, changedBits accumulates all transitions so nothing is lost.
    _changedBits = 0;

    while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();
        processByte(b);
    }
}

void PowerLink::processByte(uint8_t b) {
    // Discard stray handshake bytes that can arrive during the transition
    // between handshake and normal operation.
    if (b == MSG_READY) return;

    // MSB set means control code.  Currently the CH32 doesn't send any
    // control codes to the ESP32 other than READY, but this guard keeps
    // the protocol extensible without misinterpreting future codes as
    // button state.
    if (b & 0x80) return;

    // It's a button state byte.
    _previousState = _currentState;
    _currentState  = b;

    uint8_t delta = _currentState ^ _previousState;
    _changedBits |= delta;

    if (delta && _callback) {
        _callback(_currentState, _previousState);
    }
}

bool PowerLink::isConnected() const {
    return _connected;
}

uint8_t PowerLink::getState() const {
    return _currentState;
}

uint8_t PowerLink::getPreviousState() const {
    return _previousState;
}

bool PowerLink::isPressed(uint8_t button) const {
    if (button >= NUM_BUTTONS) return false;
    return (_currentState >> button) & 0x01;
}

bool PowerLink::stateChanged() const {
    return _changedBits != 0;
}

bool PowerLink::buttonChanged(uint8_t button) const {
    if (button >= NUM_BUTTONS) return false;
    return (_changedBits >> button) & 0x01;
}

bool PowerLink::buttonPressed(uint8_t button) const {
    if (button >= NUM_BUTTONS) return false;
    // Changed AND now pressed.
    return ((_changedBits >> button) & 0x01) && ((_currentState >> button) & 0x01);
}

bool PowerLink::buttonReleased(uint8_t button) const {
    if (button >= NUM_BUTTONS) return false;
    // Changed AND now released.
    return ((_changedBits >> button) & 0x01) && !((_currentState >> button) & 0x01);
}

// ── Commands to CH32 ──────────────────────────────────────────────────────────

void PowerLink::shutdown() {
    Serial1.write(MSG_SHUTDOWN);
    _connected = false;
}

void PowerLink::reset() {
    Serial1.write(MSG_RESET);
    _connected = false;
}

void PowerLink::close() {
    Serial1.write(MSG_CLOSE);
    _connected = false;
}

void PowerLink::setHardOffTime(uint16_t ms) {
    Serial1.write(MSG_SET_HARD_OFF_TIME);
    Serial1.write((uint8_t)(ms & 0xFF));         // low byte first
    Serial1.write((uint8_t)((ms >> 8) & 0xFF));  // high byte second
}

// ── Callback ──────────────────────────────────────────────────────────────────

void PowerLink::onStateChange(StateChangeCallback callback) {
    _callback = callback;
}
