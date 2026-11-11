#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(USE_CANBUS)

#include <cstdint>

class CANBus {
public:
  CANBus();
  void setup_canbus();
  void send_message(uint32_t id, uint8_t* data, uint8_t len);
  bool receive_message(uint32_t* id, uint8_t* data, uint8_t* len);
};

extern CANBus canbus;

#endif // USE_CANBUS
