#include "can.h"
#include "hardware.h"
#include "module.h"
#include "registers.h"
#include "robot.h"
#include <stdint.h>

// Address of the motor module
const uint8_t MOTOR_ADDR = 21;

static int8_t register_handler(uint8_t operation, uint8_t address,
                               RadioData *radio_data) {

  switch (operation) {
  case ROP_READ_8:
    radio_data->byte = reg8_table[address];
    return TRUE;
  case ROP_WRITE_8:
    reg8_table[address] = radio_data->byte; // Allow writing to register
    return TRUE;
  }
  return FALSE;
}

int main(void) {
  hardware_init();
  registers_init();

  radio_add_reg_callback(register_handler);
  // Set head LED to visible for tracking (green=64)
  // We'll let the PC control the exact color

  // Main loop - just keep the system running
  while (1) {
    set_rgb(
        (uint8_t)((reg32_table[REG32_LED] & 0xFF0000) >> 16),
        (uint8_t)((reg32_table[REG32_LED] & 0x00FF00) >> 8),
        (uint8_t)(reg32_table[REG32_LED] & 0x0000FF)); // Only green component

    // The head will receive color commands from the PC program
    // No additional processing needed here
    pause(TEN_MS);
  }

  return 0;
}
