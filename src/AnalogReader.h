#include <Arduino.h>
#include <algorithm>
#include <vector>

namespace Olon {

class AnalogReader {
 private:
  uint16_t _sampleCount = 2;
  uint32_t _sampleDelay = 5; // 5 msec
  uint32_t _lastSampleTime = 0;
#ifdef ESP8266
  uint16_t _maxAnalogReading = 1023;  // 1023 for 10-bit ADC (ESP8266)  4095 for 12bits ADC (ESP32)
  uint8_t  _pin = A0;
#else
  uint16_t _maxAnalogReading = 4095;  // 1023 for 10-bit ADC (ESP8266)  4095 for 12bits ADC (ESP32)
  uint8_t  _pin;
#endif
  float    _maxVoltage = 3.3f;  // the voltage @ maxAnalogReading
  bool     _busy = false;
  mutable std::vector<uint16_t> _samples;
  void (*_callback)() = nullptr;

  float calculateMedian() const {
    size_t size = _samples.size();

    if (size == 0) {
        // Handle empty vector case
        return 0.0f; // or throw an exception, or return a special value
    }

    if (size == 1) {
        return _samples[0];
    }

    std::sort(_samples.begin(), _samples.end());

    if (size % 2 == 0) {
        return (_samples[size/2 - 1] + _samples[size/2]) / 2.0f;
    } else {
        return _samples[size/2];
    }
}

 public:
  // AnalogReader()  { }

  // AnalogReader(uint8_t pin, uint16_t sampleCount, uint32_t sampleDelay)
  //     : _pin(pin), _sampleCount(sampleCount), _sampleDelay(sampleDelay), _lastSampleTime(0), _callback(nullptr) {
  //   _samples.reserve(_sampleCount);
  // }

  void setPin(uint8_t pin) {
    _pin = pin;
  }

  void setSampleCount(uint16_t count) {
    _sampleCount = count;
    _samples.reserve(_sampleCount);
  }

  void setSampleDelay(uint32_t delay) {
    _sampleDelay = delay;
  }

  void setMaxAnalogReading(uint16_t maxAnalogReading) {
    _maxAnalogReading = maxAnalogReading;
  }

  void setMaxVoltage(float maxVoltage) {
    _maxVoltage = maxVoltage;
  }

  void onComplete(void (*callback)()) {
    _callback = callback;
  }

  void run() {
    _samples.clear();
    _lastSampleTime = 0;
    _busy = true;
  }

  bool busy() {
    return _busy;
  }

  void loop() {
    // do not continue loop if already got the number of samples
    // because they are already processed a few lines bellow.
    if (_samples.size() >= _sampleCount) return;

    uint32_t currentTime = millis();
    if (currentTime - _lastSampleTime >= _sampleDelay || _lastSampleTime == 0) {
      uint16_t reading = analogRead(_pin);
      _samples.push_back(reading);
      _lastSampleTime = currentTime;

      if (_samples.size() >= _sampleCount && _callback) {
        _callback();
        _busy = false;
      }
    }
  }

  float voltage() const {
    return (calculateMedian() * _maxVoltage) / (float)_maxAnalogReading;
  }
};

};