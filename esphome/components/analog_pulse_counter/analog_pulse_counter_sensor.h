#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"
#include "esphome/components/sensor/sensor.h"

#ifdef USE_ESP32
#include "driver/adc.h"
#include <esp_adc_cal.h>
#endif

#if defined(USE_ESP32) && !defined(USE_ESP32_VARIANT_ESP32C3)
#include <driver/pcnt.h>
#define HAS_PCNT
#endif

namespace esphome {
namespace analog_pulse_counter {

enum PulseCounterCountMode {
  PULSE_COUNTER_DISABLE = 0,
  PULSE_COUNTER_INCREMENT,
  PULSE_COUNTER_DECREMENT,
};

#ifdef HAS_PCNT
using pulse_counter_t = int16_t;
#else
using pulse_counter_t = int32_t;
#endif

struct AnalogPulseCounterStorage {
  bool pulse_counter_setup(InternalGPIOPin *pin);
  pulse_counter_t read_raw_value();

  static void gpio_intr(AnalogPulseCounterStorage *arg);

#ifndef HAS_PCNT
  volatile pulse_counter_t counter{0};
  volatile uint32_t last_pulse{0};
#endif

  InternalGPIOPin *pin;
#ifdef HAS_PCNT
  pcnt_unit_t pcnt_unit;
#else
  ISRInternalGPIOPin isr_pin;
#endif
  PulseCounterCountMode rising_edge_mode{PULSE_COUNTER_INCREMENT};
  PulseCounterCountMode falling_edge_mode{PULSE_COUNTER_DISABLE};
  uint32_t filter_us{0};
  pulse_counter_t last_value{0};
};

class AnalogPulseCounterSensor : public sensor::Sensor, public Component {
 public:
  void set_pin(InternalGPIOPin *pin) { pin_ = pin; }
  void set_rising_edge_mode(PulseCounterCountMode mode) { storage_.rising_edge_mode = mode; }
  void set_falling_edge_mode(PulseCounterCountMode mode) { storage_.falling_edge_mode = mode; }
  void set_filter_us(uint32_t filter) { storage_.filter_us = filter; }
  void set_total_sensor(sensor::Sensor *total_sensor) { total_sensor_ = total_sensor; }

  /// Unit of measurement is "pulses/min".
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void dump_config() override;
  void set_update_interval(long interval) {}

 protected:
  InternalGPIOPin *pin_;
  AnalogPulseCounterStorage storage_;
  uint32_t last_time_{0};
  uint32_t current_total_{0};
  sensor::Sensor *total_sensor_;

  HighFrequencyLoopRequester high_freq_;

 private:
  int pulseThreshold = 55;   //minimum pulse height over signalFloor

  int minInOld;

  unsigned long PulseStartMicros = 0;
  unsigned long prevPulseEndMicros = 0;

  int readingIndex = 0;
  int readingCount = 0;
  long accumulator = 0;
  bool timingCalibrated = false;
  long startTime = 0;
  int minIn = 0xFFFF;
  int maxIn = -1;
  int count = 0;
  int prevIn = 0;
  int firstIn = 0;
  int countHigh = 0;
  int countLow = 0;

  int      countLowOld;
  int      countHighOld;
  int maxInOld;
  int signalFloor;

  float watts;
  bool newWH = false;
  long cumWH = 0;
  unsigned long LoopStartMicros = micros();
  int sampleCount = 0;

    #define MAX_WINDOW_SIZE 400 // Allocate enough space to accommodate for faster processors
  int pulseWidth = 0;
  int maxWindowSize;
  int windowSize;
  int *readings;
};

}  // namespace pulse_counter
}  // namespace esphome
