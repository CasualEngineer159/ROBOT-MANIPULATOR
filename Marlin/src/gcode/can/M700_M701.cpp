#include "../../inc/MarlinConfig.h"

#if ENABLED(USE_CANBUS)

#include "../gcode.h"
#include "../../module/canbus.h"
#include "../../core/serial.h"
#include <stdlib.h>

/**
 * M700: Send a CAN message
 *
 *  I<id>   CAN message ID
 *  H<hex>  Data to send (up to 8 bytes, sent as a hex string e.g., H1122AABB)
 */
void GcodeSuite::M700() {
  uint32_t id = 0;
  if (parser.seenval('I')) id = parser.value_ulong();

  uint8_t data[8] = {0};
  uint8_t len = 0;

  if (parser.seen('H')) {
    char* hex_str = parser.value_string();
    if (hex_str) {
      if (hex_str[0] == '"') hex_str++; // Ignore leading quote if provided
      len = min((size_t)8, strlen(hex_str) / 2);
      for (uint8_t i = 0; i < len; i++) {
        char byte_str[3] = {hex_str[i * 2], hex_str[i * 2 + 1], '\0'};
        data[i] = (uint8_t)strtoul(byte_str, NULL, 16);
      }
    }
  }

  canbus.send_message(id, data, len);
  SERIAL_ECHOLN("Message sent");
}

/**
 * M701: Receive a CAN message
 */
void GcodeSuite::M701() {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;

  if (canbus.receive_message(&id, data, &len)) {
    SERIAL_ECHO("Received CAN message - ID: ");
    SERIAL_ECHO(id);
    SERIAL_ECHO(" | Data: ");
    for (uint8_t i = 0; i < len; i++)
    {
      char hex_byte[4];
      sprintf(hex_byte, "%02X ", data[i]);
      SERIAL_ECHO(hex_byte);
    }
    SERIAL_EOL();
  } else {
    SERIAL_ECHOLN("No CAN message received");
  }
}

#endif // USE_CANBUS