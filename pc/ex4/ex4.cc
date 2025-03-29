#include "regdefs.h"
#include "remregs.h"
#include "robot.h"
#include "utils.h"
#include <cmath>
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

  // Reboot the head microcontroller to make sure it's in a known state
  reboot_head(regs);

  // First, check if the robot is in idle mode
  uint8_t mode = regs.get_reg_b(REG8_MODE);
  cout << "Initial mode: " << (int)mode << endl;

  // Option 1: Just set the mode to start motor demo
  cout << "Starting motor demo mode..." << endl;
  regs.set_reg_b(REG8_MODE, IMODE_MOTOR_DEMO);

  // Wait for user to press Enter to stop
  cout << "Press Enter to stop and continue to sine wave..." << endl;
  cin.get();

  // Stop the motor by setting mode back to idle
  regs.set_reg_b(REG8_MODE, IMODE_IDLE);
  cout << "Motor stopped. Waiting for motor to return to zero..." << endl;
  Sleep(2000); // Give time for the motor to settle

  // Option 2: Sine wave control
  cout << "Starting sine wave control. Press Enter to stop." << endl;

  double startTime = time_d(); // Get start time
  double currentTime;
  bool running = true;

  while (running) {
    // Calculate current time since start
    currentTime = time_d() - startTime;

    // Calculate sine wave value (-1 to 1) and scale to desired amplitude
    double angle = AMPLITUDE_DEG * sin(2.0 * M_PI * FREQUENCY_HZ * currentTime);

    // Send the setpoint to the motor
    // Note: The setpoint register address is MREG_SETPOINT (defined in module.h
    // as 0x2F) We need to convert the angle to the motor's units using
    // DEG_TO_OUTPUT_BODY macro Since we can't directly use the macro, we'll
    // approximate it (21 units per 40 degrees)
    int8_t setpoint = static_cast<int8_t>(angle * (21.0 / 40.0));
    regs.set_reg_b(0x2F, static_cast<uint8_t>(setpoint));

    // Check if user pressed a key (non-blocking)
    if (kbhit()) {
      running = false;
    }

    // Small delay to prevent overwhelming the communication
    Sleep(10);
  }

  // Stop the motor and set mode back to idle
  regs.set_reg_b(0x2F, 0); // Set setpoint to 0
  regs.set_reg_b(REG8_MODE, IMODE_IDLE);

  cout << "Program terminated." << endl;
  return 0;
}
