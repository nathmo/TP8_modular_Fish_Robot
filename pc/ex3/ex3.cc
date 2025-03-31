#include <iostream>
#include <thread>
#include <chrono>
#include "remregs.h"
#include "robot.h"
#include "utils.h"

using namespace std;

const uint8_t RADIO_CHANNEL = 201;  ///< Robot radio channel
const char* INTERFACE = "COM1";     ///< Robot radio interface

// Displays the contents of a multibyte register as a list of bytes
void display_multibyte_register(CRemoteRegs& regs, const uint8_t addr)
{
  uint8_t data_buffer[32], len;
  if (regs.get_reg_mb(addr, data_buffer, len)) {
    cout << (int) len << " bytes: ";
    for (unsigned int i(0); i < len; i++) {
      if (i > 0) cout << ", ";
      cout << static_cast<int32_t>(static_cast<int8_t>(data_buffer[i]));
    }
    cout << endl;
  } else {
    cerr << "Unable to read multibyte register." << endl;
  }
}

int main()
{
  CRemoteRegs regs;

  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    return 1;
  }

  // Reboots the head microcontroller to ensure a consistent state
  reboot_head(regs);

  while (!kbhit()) {
    display_multibyte_register(regs, 2);
  }

  regs.close();
  return 0;
}