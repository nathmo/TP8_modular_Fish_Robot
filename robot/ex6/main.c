#include "can.h"
#include "hardware.h"
#include "module.h"
#include "registers.h"
#include "robot.h"
#include <stdint.h>

// Address of the motor module

int main(void) {
  hardware_init();
  registers_init();
  // Set head LED to visible for tracking (green=64)
  // We'll let the PC control the exact color

  // Main loop - just keep the system running
  while (1) {
    // The head will receive color commands from the PC program
    // No additional processing needed here
    pause(TEN_MS);
  }

  return 0;
}
