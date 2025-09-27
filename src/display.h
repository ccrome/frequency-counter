#pragma once

#include <Arduino.h>
#include <Arduino.h>

struct DisplayUtcTime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

struct DisplayStatus {
  bool pps_locked;
  double ppm_error;
  double ppm_average;
  uint32_t sample_count;
  bool utc_valid;
  DisplayUtcTime utc;
  bool output_high;

};

bool display_init();
void display_update(DisplayStatus const& status);
bool display_available();

