#include <ServoWrapper.h>

uint16_t ServoWrapper::degreesToMicroseconds(float degrees) const
{
  float clamped = constrain(degrees, MIN_ANGLE, MAX_ANGLE);
  float us = MIN_PULSE_US + (clamped / (MAX_ANGLE - MIN_ANGLE)) * (MAX_PULSE_US - MIN_PULSE_US);
  return static_cast<uint16_t>(lroundf(us));
}

void ServoWrapper::attach()
{
  _serv.attach(_pin);
  // hold the current position immediately so attach never causes a jump
  _serv.writeMicroseconds(degreesToMicroseconds(_currentPos + _trim));
}

void ServoWrapper::detach() { _serv.detach(); }

float ServoWrapper::getCurrentPos() const { return _currentPos; }

float ServoWrapper::getSmoothing() const { return _smoothing; }

void ServoWrapper::setSmoothing(float val) { _smoothing = val; }

float ServoWrapper::write(float target)
{
  _serv.writeMicroseconds(degreesToMicroseconds(target + _trim));
  _currentPos = target;
  return _currentPos;
}

float ServoWrapper::moveTo(float t)
{
  constexpr float CLOSE_ENOUGH = 0.50;
  float filtered = ((1.0f - _smoothing) * t) + (_smoothing * _currentPos);
  filtered = abs(filtered - t) <= CLOSE_ENOUGH ? t : filtered;    // make sure that we hit the target instead of Xenoing ever closer
  return write(filtered);
}