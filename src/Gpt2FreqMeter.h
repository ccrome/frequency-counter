#pragma once
#include <Arduino.h>
#include "imxrt.h"

enum GptCaptureEdge {
  GPT_EDGE_DISABLED = 0,
  GPT_EDGE_RISING   = 1,
  GPT_EDGE_FALLING  = 2,
  GPT_EDGE_BOTH     = 3,
};

// Dual Mode Functions (Output Compare Always Active)
void gpt2_begin_dual_mode(uint32_t output_freq_hz = 1, GptCaptureEdge capture_edge = GPT_EDGE_RISING, bool use_external_clock = false);
void gpt2_set_output_frequency(uint32_t frequency_hz);
void gpt2_set_capture_edge(GptCaptureEdge edge);

// Input Capture Functions (for GPS PPS when available)
bool gpt2_capture_available();
uint32_t gpt2_read_capture();
void gpt2_poll_capture();

// System Control
void gpt2_stop();
bool gpt2_is_running();
uint32_t gpt2_get_output_frequency();
