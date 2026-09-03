/**
 * PowerLink.h — ESP32-C3 library for communicating with a CH32V003 power controller over UART.
 *
 * The CH32V003 acts as an always-on power manager controlling a high-side switch that
 * provides power to the ESP32. This library handles the UART protocol between them:
 *   - Handshake (INIT / READY)
 *   - Receiving button state bytes from the CH32
 *   - Sending commands: shutdown, reset, set hard-off time, close
 *
 * Protocol summary:
 *   Any byte with MSB set (>= 0x80) is a control code.
 *   Any byte with MSB clear (< 0x80) is a button state byte.
 *   Bits 0..3 of a state byte map to buttons 0..3 (1 = pressed, active low on CH32 side).
 *
 * Usage:
 *   #include <PowerLink.h>
 *
 *   PowerLink power;
 *
 *   void setup() {
 *       power.begin();  // blocks until handshake completes
 *   }
 *
 *   void loop() {
 *       power.update();
 *       if (power.buttonChanged(1)) {
 *           Serial.println(power.isPressed(1) ? "btn1 pressed" : "btn1 released");
 *       }
 *   }
 */

#ifndef POWERLINK_H
#define POWERLINK_H

#include <Arduino.h>

class PowerLink {
public:
    /// Number of buttons tracked by the protocol (bits 0..3 of the state byte).
    static constexpr uint8_t NUM_BUTTONS = 4;

    /// Button indices for readability.  Button 0 is the power button on the CH32 side.
    static constexpr uint8_t BTN_POWER = 0;
    static constexpr uint8_t BTN_0     = 1;
    static constexpr uint8_t BTN_1     = 2;
    static constexpr uint8_t BTN_2     = 3;

    /**
     * Construct a PowerLink instance.
     *
     * @param rxPin   ESP32 GPIO used as UART RX (connected to CH32 TX / PD5)
     * @param txPin   ESP32 GPIO used as UART TX (connected to CH32 RX / PD6)
     * @param baud    Baud rate.  Must match the CH32 firmware (default 9600).
     */
    PowerLink(int rxPin = 2, int txPin = 8, unsigned long baud = 9600);

    /**
     * Initialize the UART and perform the handshake with the CH32.
     * Blocks until the CH32 responds with READY.
     */
    void begin();

    /**
     * Call once per loop() iteration.  Reads any available bytes from the CH32,
     * updates button state, and dispatches the state-change callback if registered.
     */
    void update();

    /**
     * Returns true if the link has completed the handshake and is operational.
     */
    bool isConnected() const;

    // ── Button state queries ──────────────────────────────────────────────

    /**
     * Returns the most recent raw button state byte (bits 0..3).
     */
    uint8_t getState() const;

    /**
     * Returns the previous button state byte (before the last change).
     */
    uint8_t getPreviousState() const;

    /**
     * Returns true if the given button is currently pressed.
     * @param button  Button index (0..3).  Use the BTN_* constants.
     */
    bool isPressed(uint8_t button) const;

    /**
     * Returns true if *any* button state changed during the most recent update() call.
     */
    bool stateChanged() const;

    /**
     * Returns true if the specified button changed state during the most recent update() call.
     */
    bool buttonChanged(uint8_t button) const;

    /**
     * Returns true if the specified button transitioned from released to pressed
     * during the most recent update() call.
     */
    bool buttonPressed(uint8_t button) const;

    /**
     * Returns true if the specified button transitioned from pressed to released
     * during the most recent update() call.
     */
    bool buttonReleased(uint8_t button) const;

    // ── Commands to CH32 ──────────────────────────────────────────────────

    /**
     * Request the CH32 to cut power (enter standby).
     * After this call the ESP32 will lose power shortly.
     */
    void shutdown();

    /**
     * Request the CH32 to power-cycle the ESP32 (off, brief delay, on).
     * The ESP32 will reboot; begin() must be called again after reset.
     */
    void reset();

    /**
     * Tell the CH32 to close the UART and release PD5/PD6.
     * Useful if those pins need to be repurposed.  After this call,
     * the link is no longer operational until a new begin().
     */
    void close();

    /**
     * Set the duration (in milliseconds) that the power button must be held
     * on the CH32 side to trigger a hard power-off.
     *
     * @param ms  Hold duration in milliseconds.  Values below 500 are rejected
     *            by the CH32 and reset to the default (5000).
     */
    void setHardOffTime(uint16_t ms);

    // ── Callback ──────────────────────────────────────────────────────────

    /// Signature for the state-change callback.
    typedef void (*StateChangeCallback)(uint8_t newState, uint8_t previousState);

    /**
     * Register a function to be called whenever the button state changes.
     * The callback receives the new state byte and the previous state byte.
     * Pass nullptr to remove the callback.
     */
    void onStateChange(StateChangeCallback callback);

private:
    int           _rxPin;
    int           _txPin;
    unsigned long _baud;
    bool          _connected;
    uint8_t       _currentState;
    uint8_t       _previousState;
    uint8_t       _changedBits;      // XOR of current and previous after last update()

    StateChangeCallback _callback;

    void performHandshake();
    void processByte(uint8_t byte);

    // ── Protocol constants ────────────────────────────────────────────────
    static constexpr uint8_t MSG_INIT              = 0xAE;
    static constexpr uint8_t MSG_READY             = 0xAB;
    static constexpr uint8_t MSG_CLOSE             = 0xB7;
    static constexpr uint8_t MSG_SET_HARD_OFF_TIME = 0xC3;
    static constexpr uint8_t MSG_SHUTDOWN          = 0x9D;
    static constexpr uint8_t MSG_RESET             = 0x9A;
};

#endif // POWERLINK_H
