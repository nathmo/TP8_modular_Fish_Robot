#include "regdefs.h"
#include "remregs.h"
#include "robot.h"
#include "utils.h"
#include <cmath>
#include <conio.h> // For kbhit() and getch()
#include <iostream>

#define IMODE_IDLE 0
#define IMODE_MOTOR_DEMO 1

// Constants
const uint8_t RADIO_CHANNEL = 201; // Radio channel
const char *INTERFACE = "COM1";    // Serial port for radio interface
const uint8_t MOTOR_ADDR = 21;     // Motor address (from modes.c)
const double AMPLITUDE_DEG = 40.0; // Amplitude in degrees
const double FREQUENCY_HZ = 1.0;   // Frequency in Hz

using namespace std;

int main() {
  CRemoteRegs regs;

  cout << "Initializing robot connection..." << endl;
  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    return 1;
  }

  // Reboot the head microcontroller to ensure it is in a known state.
  reboot_head(regs);

  // Show initial mode
  uint8_t mode = regs.get_reg_b(REG8_MODE);
  cout << "Initial mode: " << static_cast<int>(mode) << endl;

  bool exitProgram = false;
  char choice = '\0';

  while (!exitProgram) {
    cout << "\n=====================================\n";
    cout << "Select an option:\n";
    cout << "1. Motor Demo Mode\n";
    cout << "2. Sine Wave Control Mode\n";
    cout << "q. Quit\n";
    cout << "Enter your choice: ";

    cin >> choice;
    // Clear any extra characters from the input buffer.
    cin.ignore(10000, '\n');

    switch (choice) {
    case '1': {
      cout << "\nStarting Motor Demo Mode..." << endl;
      regs.set_reg_b(REG8_MODE, IMODE_MOTOR_DEMO);
      cout << "Press Enter to stop motor demo." << endl;
      cin.get();
      regs.set_reg_b(REG8_MODE, IMODE_IDLE);
      cout << "Motor demo stopped. Waiting for motor to return to zero..."
           << endl;
      Sleep(2000); // Give time for the motor to settle
      break;
    }
    case '2': {
      cout << "\nStarting Sine Wave Control Mode..." << endl;
      cout << "Press any key to stop." << endl;

      double startTime = time_d(); // Get the start time
      double currentTime = 0.0;
      bool running = true;

      while (running) {
        // Calculate elapsed time since start
        currentTime = time_d() - startTime;

        // Calculate sine wave value (-1 to 1) and scale it to the desired
        // amplitude
        double angle =
            AMPLITUDE_DEG * sin(2.0 * M_PI * FREQUENCY_HZ * currentTime);

        // Send the setpoint to the motor (assuming register 0x06 is the
        // setpoint)
        regs.set_reg_b(0x06, static_cast<int8_t>(angle));

        // If a key is pressed, break out of the loop
        if (kbhit()) {
          // Clear the key from the buffer.
          getch();
          running = false;
        }

        // Small delay to prevent overwhelming communication
        Sleep(10);
      }
      // Stop the motor and return to idle mode
      regs.set_reg_b(0x06, 0); // Set setpoint to 0
      regs.set_reg_b(REG8_MODE, IMODE_IDLE);
      cout << "Sine wave control stopped." << endl;
      break;
    }
    case 'q':
    case 'Q': {
      exitProgram = true;
      cout << "\nExiting program." << endl;
      break;
    }
    default: {
      cout << "\nInvalid choice. Please try again." << endl;
      break;
    }
    }
  }

  return 0;
}
