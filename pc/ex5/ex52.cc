#include "regdefs.h"
#include "remregs.h"
#include "robot.h"
#include "utils.h"
#include <cmath>
#include <iostream>

#define IMODE_IDLE 0
#define IMODE_MOTOR_DEMO 1
#define IMODE_SINE_DEMO 2

// Constants
const uint8_t RADIO_CHANNEL = 201; // Radio channel
const char *INTERFACE = "COM1";    // Serial port for radio interface

// Define registers for frequency and amplitude control
#define REG8_SINE_FREQ 10 // Register for sine wave frequency
#define REG8_SINE_AMP 11  // Register for sine wave amplitude

// Define limits for frequency and amplitude
#define MAX_FREQ 2.0f // Maximum frequency in Hz
#define MAX_AMP 60.0f // Maximum amplitude in degrees

using namespace std;

// Function to display current settings
void display_settings(CRemoteRegs &regs) {
  uint8_t freq_reg = regs.get_reg_b(REG8_SINE_FREQ);
  uint8_t amp_reg = regs.get_reg_b(REG8_SINE_AMP);

  float freq = DECODE_PARAM_8(freq_reg, 0.1f, MAX_FREQ);
  float amplitude = DECODE_PARAM_8(amp_reg, 1.0f, MAX_AMP);

  cout << "Current settings:" << endl;
  cout << "  Frequency: " << freq << " Hz (encoded: " << (int)freq_reg << ")"
       << endl;
  cout << "  Amplitude: " << amplitude << " degrees (encoded: " << (int)amp_reg
       << ")" << endl;
}

// Function to update a parameter
void update_parameter(CRemoteRegs &regs, const char *name, uint8_t reg,
                      float min_value, float max_value, float scale,
                      float current) {
  float new_value;
  cout << "Enter new " << name << " (" << min_value << " - " << max_value
       << ") [" << current << "]: ";

  string input;
  getline(cin, input);

  if (input.empty()) {
    cout << "Keeping current value." << endl;
    return;
  }

  try {
    new_value = stof(input);

    // Validate the input
    if (new_value < min_value || new_value > max_value) {
      cout << "Value out of range. Using closest valid value." << endl;
      new_value = (new_value < min_value) ? min_value : max_value;
    }

    // Encode and set the register
    uint8_t encoded = ENCODE_PARAM_8(new_value, scale, max_value);
    regs.set_reg_b(reg, encoded);

    cout << name << " set to " << new_value << " (encoded: " << (int)encoded
         << ")" << endl;
  } catch (const exception &e) {
    cout << "Invalid input. Keeping current value." << endl;
  }
}

int main() {
  CRemoteRegs regs;

  cout << "Sine Wave Controller for Fish Robot" << endl;
  cout << "-----------------------------------" << endl;

  cout << "Initializing robot connection on " << INTERFACE << ", channel "
       << (int)RADIO_CHANNEL << endl;
  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    cerr << "Failed to initialize radio interface" << endl;
    return 1;
  }

  // Reboot the head microcontroller
  cout << "Rebooting head..." << endl;
  reboot_head(regs);

  // Main control loop
  bool running = true;
  while (running) {
    // Get current parameter values
    uint8_t freq_reg = regs.get_reg_b(REG8_SINE_FREQ);
    uint8_t amp_reg = regs.get_reg_b(REG8_SINE_AMP);

    float freq = DECODE_PARAM_8(freq_reg, 0.1f, MAX_FREQ);
    float amplitude = DECODE_PARAM_8(amp_reg, 1.0f, MAX_AMP);

    // Check current mode
    uint8_t currentMode = regs.get_reg_b(REG8_MODE);

    // Display menu
    cout << "\n----- MENU -----" << endl;
    cout << "1. Start/Stop Sine Wave Demo" << endl;
    cout << "2. Set Frequency (current: " << freq << " Hz)" << endl;
    cout << "3. Set Amplitude (current: " << amplitude << " degrees)" << endl;
    cout << "4. Display Current Settings" << endl;
    cout << "5. Exit" << endl;
    cout << "Current mode: "
         << (currentMode == IMODE_SINE_DEMO ? "RUNNING" : "STOPPED") << endl;
    cout << "Selection: ";

    string input;
    getline(cin, input);

    if (input.empty())
      continue;

    switch (input[0]) {
    case '1':
      if (currentMode != IMODE_SINE_DEMO) {
        cout << "Starting sine wave demo..." << endl;
        regs.set_reg_b(REG8_MODE, IMODE_SINE_DEMO);
      } else {
        cout << "Stopping sine wave demo..." << endl;
        regs.set_reg_b(REG8_MODE, IMODE_IDLE);
      }
      break;

    case '2':
      update_parameter(regs, "frequency", REG8_SINE_FREQ, 0.1f, MAX_FREQ, 0.1f,
                       freq);
      break;

    case '3':
      update_parameter(regs, "amplitude", REG8_SINE_AMP, 1.0f, MAX_AMP, 1.0f,
                       amplitude);
      break;

    case '4':
      display_settings(regs);
      break;

    case '5':
      // Make sure to stop the demo before exiting
      if (regs.get_reg_b(REG8_MODE) == IMODE_SINE_DEMO) {
        cout << "Stopping sine wave demo..." << endl;
        regs.set_reg_b(REG8_MODE, IMODE_IDLE);
        // Give it time to stop properly
        Sleep(2000);
      }
      running = false;
      break;

    default:
      cout << "Invalid selection. Please try again." << endl;
      break;
    }
  }

  cout << "Program terminated." << endl;
  return 0;
}
