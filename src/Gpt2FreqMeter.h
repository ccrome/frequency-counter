#pragma once
#include <Arduino.h>
#include "imxrt.h"

enum GptCaptureEdge {
  GPT_EDGE_DISABLED = 0,
  GPT_EDGE_RISING   = 1,
  GPT_EDGE_FALLING  = 2,
  GPT_EDGE_BOTH     = 3,
};

void gpt2_begin_extclk_cap1(bool use_external_clock = true, GptCaptureEdge edge = GPT_EDGE_RISING);
bool gpt2_available();
uint32_t gpt2_read();
void gpt2_poll_capture();
