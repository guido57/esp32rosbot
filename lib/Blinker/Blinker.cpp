#include "Blinker.h"
#include <Arduino.h>

Blinker::Blinker(int pin) : _pin(pin), _state(false) {}

void Blinker::begin() {
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
}

void Blinker::toggle() {
    _state = !_state;
    digitalWrite(_pin, _state ? HIGH : LOW);
}

bool Blinker::getState() const {
    return _state;
}
