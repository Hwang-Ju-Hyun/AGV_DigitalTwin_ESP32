#pragma once

#include <cstdint>

#define IRAM_ATTR
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define RISING 3
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(mux) ((void)(mux))
#define portEXIT_CRITICAL(mux) ((void)(mux))
#define portENTER_CRITICAL_ISR(mux) ((void)(mux))
#define portEXIT_CRITICAL_ISR(mux) ((void)(mux))

using portMUX_TYPE = int;

inline int g_ArduinoPinState[64]{};
inline int g_ArduinoPwmDuty[16]{};
inline void (*g_ArduinoInterruptHandler[64])(){};
inline uint32_t g_ArduinoMillis = 0;

inline void pinMode(int, int) {}
inline void digitalWrite(int pin, int value)
{
    g_ArduinoPinState[pin] = value;
}
inline int digitalRead(int pin)
{
    return g_ArduinoPinState[pin];
}
inline void ledcSetup(int, int, int) {}
inline void ledcAttachPin(int, int) {}
inline void ledcWrite(int channel, int duty)
{
    g_ArduinoPwmDuty[channel] = duty;
}
inline int digitalPinToInterrupt(int pin)
{
    return pin;
}
inline void attachInterrupt(int interrupt, void (*handler)(), int)
{
    g_ArduinoInterruptHandler[interrupt] = handler;
}
inline uint32_t millis()
{
    return g_ArduinoMillis;
}

