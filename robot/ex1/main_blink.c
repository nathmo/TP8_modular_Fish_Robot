#include <stdint.h>
#include "hardware.h"
#include "registers.h"

int main(void)
{
  int8_t i;

  hardware_init();
  reg32_table[REG32_LED] = LED_MANUAL;  // manual LED control

  while (1) {
    pause(HALF_SEC);
    set_rgb(0, 255, 0); // turn on greenlight
    pause(HALF_SEC);
    set_rgb(0, 0, 0); // turn off greenlight
  }
  return 0;
}