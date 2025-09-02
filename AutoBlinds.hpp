#pragma once
#include <Arduino.h>

namespace AutoBlinds {
  void setupAll();
  void loopAll();
  void IRAM_ATTR readEncoderISR();
}
