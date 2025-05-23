#include "regdefs.h"
#include "remregs.h"
#include "robot.h"
#include "trkcli.h"
#include "utils.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdint.h>
#include <string>
#include <windows.h>

using namespace std;

/// Idle mode: do nothing
#define IMODE_IDLE 0

/// set the robot in a warped and rigid position (prevent capsizing)
#define IMODE_READY 1

/// active swimming mode
#define IMODE_SWIM 2

// Define limits for frequency and amplitude
#define MAX_FREQ 1.5f  // Maximum frequency in Hz
#define MAX_AMP 60.0f  // Maximum amplitude in degrees
#define MAX_LAG 1.5f // Maximum lag between elements in degrees
#define MAX_OFF 3.0f // Maximum offset in degrees

#define MIN_FREQ 0.1f
#define MIN_AMP 1.0f  // Min amplitude in degrees
#define MIN_LAG 0.5f // Min lag between elements in degrees
#define MIN_OFF -3.0f // Min offset in degrees

// Define registers for frequency, lag, offset and amplitude control
#define REG8_SINE_FREQ 10 // Register for sine wave frequency
#define REG8_SINE_AMP 11  // Register for sine wave amplitude
#define REG8_SINE_LAG 12  // Register for sine wave lag between elements
#define REG8_SINE_OFF 13  // Register for sine wave offset

const char *TRACKING_PC_NAME = "biorobpc6"; ///< host name of the tracking PC
const uint16_t TRACKING_PORT = 10502;       ///< port number of the tracking PC
const uint8_t RADIO_CHANNEL = 126;          ///< robot radio channel
const char *INTERFACE = "COM3";             ///< robot radio interface

// Aquarium dimensions in meters
const double AQUARIUM_WIDTH = 6.0;
const double AQUARIUM_HEIGHT = 2.0;

// Function to map a value from one range to another
double map_value(double value, double in_min, double in_max, double out_min,
                 double out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Function to display current settings
void display_settings(CRemoteRegs &regs) {
  uint8_t freq_reg = regs.get_reg_b(REG8_SINE_FREQ);
  uint8_t amp_reg = regs.get_reg_b(REG8_SINE_AMP);
  uint8_t lag_reg = regs.get_reg_b(REG8_SINE_LAG);
  uint8_t off_reg = regs.get_reg_b(REG8_SINE_OFF);

  float freq = DECODE_PARAM_8(freq_reg, MIN_FREQ, MAX_FREQ);
  float amplitude = DECODE_PARAM_8(amp_reg, MIN_AMP, MAX_AMP);
  float lag = DECODE_PARAM_8(lag_reg, MIN_LAG, MAX_LAG);
  float offset = DECODE_PARAM_8(off_reg, MIN_OFF, MAX_OFF);

  cout << "Current settings:" << endl;
  cout << "  Frequency: " << freq << " Hz (encoded: " << (int)freq_reg << ")"
       << endl;
  cout << "  Amplitude: " << amplitude << " degrees (encoded: " << (int)amp_reg
       << ")" << endl;
  cout << "  Lag: " << lag << " degrees (encoded: " << (int)lag_reg << ")"
       << endl;
  cout << "  Offset: " << offset << " degrees (encoded: " << (int)off_reg << ")"
       << endl;
}

// Function to update a parameter
void update_parameter(CRemoteRegs &regs, const char *name, uint8_t reg,
                      float min_value, float max_value,
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
    uint8_t encoded = ENCODE_PARAM_8(new_value, min_value, max_value);
    regs.set_reg_b(reg, encoded);

    cout << name << " set to " << new_value << " (encoded: " << (int)encoded
         << ")" << endl;
  } catch (const exception &e) {
    cout << "Invalid input. Keeping current value." << endl;
  }
}

void update_parameter_force(CRemoteRegs &regs, uint8_t reg, float min_value,
                            float max_value, float value) {
  // Ensure value is within bounds
  if (value < min_value)
    value = min_value;
  if (value > max_value)
    value = max_value;

  // Encode and set the register
  uint8_t encoded = ENCODE_PARAM_8(value, min_value, max_value);
  regs.set_reg_b(reg, encoded);
}

int main() {
  CTrackingClient trk;
  CRemoteRegs regs;

  cout << "Initializing robot connection..." << endl;
  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    cerr << "Failed to initialize radio interface" << endl;
    return 1;
  }

  // Reboot the head microcontroller to ensure it's in a known state
  reboot_head(regs);

  cout << "Connecting to tracking system..." << endl;
  if (!trk.connect(TRACKING_PC_NAME, TRACKING_PORT)) {
    cerr << "Failed to connect to tracking system" << endl;
    return 1;
  }

  cout << "Connected to tracking system. Set the robot to ready mode "
          "then place it in the aquarium."
       << endl;

  bool exitProgram = false;
  char choice = '\0';

  // Initialize parameters
  float freq = 0.8f;       // Default frequency in Hz
  float amplitude = 40.0f; // Default amplitude
  float lag = 0.75f;       // Default lag
  float offset = 0.0f;     // Default offset

  // Initialize the registers with default values
  update_parameter_force(regs, REG8_SINE_FREQ, MIN_FREQ, MAX_FREQ, freq);
  update_parameter_force(regs, REG8_SINE_AMP, MIN_AMP, MAX_AMP, amplitude);
  update_parameter_force(regs, REG8_SINE_LAG, MIN_LAG, MAX_LAG, lag);
  update_parameter_force(regs, REG8_SINE_OFF, MIN_OFF, MAX_OFF, offset);

  while (!exitProgram) {
    // Get current parameter values
    uint8_t freq_reg = regs.get_reg_b(REG8_SINE_FREQ);
    uint8_t amp_reg = regs.get_reg_b(REG8_SINE_AMP);
    uint8_t lag_reg = regs.get_reg_b(REG8_SINE_LAG);
    uint8_t off_reg = regs.get_reg_b(REG8_SINE_OFF);
    uint8_t mode_reg = regs.get_reg_b(REG8_MODE);

    freq = DECODE_PARAM_8(freq_reg, MIN_FREQ, MAX_FREQ);
    amplitude = DECODE_PARAM_8(amp_reg, MIN_AMP, MAX_AMP);
    lag = DECODE_PARAM_8(lag_reg, MIN_LAG, MAX_LAG);
    offset = DECODE_PARAM_8(off_reg, MIN_OFF, MAX_OFF);

    cout << "\n=====================================\n";
    cout << "Current mode: "
         << (mode_reg == IMODE_IDLE
                 ? "IDLE"
                 : (mode_reg == IMODE_READY
                        ? "READY"
                        : (mode_reg == IMODE_SWIM ? "SWIM" : "UNKNOWN")))
         << endl;
    cout << "Select an option:\n";
    cout << "1. Edit frequency\n";
    cout << "2. Edit amplitude\n";
    cout << "3. Edit lag\n";
    cout << "4. Edit offset\n";
    cout << "5. Display current settings\n";
    cout << "6. Ready mode\n";
    cout << "7. Swim mode\n";
    cout << "8. Interactive mode\n";
    cout << "0. Stop (idle mode)\n";
    cout << "q. Quit\n";

    cout << "Enter your choice: ";

    cin >> choice;
    // Clear any extra characters from the input buffer.
    cin.ignore(10000, '\n');

    switch (choice) {
    case '1':
      update_parameter(regs, "frequency", REG8_SINE_FREQ, MIN_FREQ, MAX_FREQ,
                       freq);
      break;

    case '2':
      update_parameter(regs, "amplitude", REG8_SINE_AMP, MIN_AMP, MAX_AMP,
                       amplitude);
      break;

    case '3':
      update_parameter(regs, "lag", REG8_SINE_LAG, MIN_LAG, MAX_LAG, lag);
      break;

    case '4':
      update_parameter(regs, "offset", REG8_SINE_OFF, MIN_OFF, MAX_OFF,
                       offset);
      break;

    case '5':
      display_settings(regs);
      break;

    case '6':
      cout << "Setting robot to ready mode..." << endl;
      regs.set_reg_b(REG8_MODE, IMODE_READY);
      break;

    case '7': {
      cout << "Setting robot to swim mode..." << endl;
      regs.set_reg_b(REG8_MODE, IMODE_SWIM);

      // Create a filename with timestamp
      time_t now = time(0);
      tm *ltm = localtime(&now);
      string filename = "robot_position_" + to_string(ltm->tm_year + 1900) +
                        "_" + to_string(ltm->tm_mon + 1) + "_" +
                        to_string(ltm->tm_mday) + "_" +
                        to_string(ltm->tm_hour) + "_" + to_string(ltm->tm_min) +
                        "_" + to_string(ltm->tm_sec) + "_freq_" + to_string(freq) + "_amp_" + 
                        to_string(amplitude) + "_lag_" + to_string(lag) + "_off_" + 
                        to_string(offset) +".csv";

      // Write header to CSV file
      ofstream file(filename);
      if (file.is_open()) {
        file << "Timestamp,X,Y" << endl;
        file.close();
      } else {
        cerr << "Unable to create log file" << endl;
      }

      cout << "Press any key to stop swimming..." << endl;

      bool swimming = true;
      while (swimming) {
        uint32_t frame_time;
        // Gets the current position
        if (!trk.update(frame_time)) {
          cerr << "Error updating tracking data" << endl;
          break;
        }

        double x = 0, y = 0;
        // bool detected = false;

        // Gets the ID of the first spot
        int id = trk.get_first_id();

        // Reads its coordinates (if (id == -1), then no spot is detected)
        if (id != -1 && trk.get_pos(id, x, y)) {
          // detected = true;

          // Get the current time as milliseconds since epoch
          auto now_ms = chrono::duration_cast<chrono::milliseconds>(
              chrono::system_clock::now().time_since_epoch());

          // Log the position to file
          ofstream datafile(filename, ios::app);
          if (datafile.is_open()) {
            datafile << now_ms.count() << "," << fixed << setprecision(3) << x
                     << "," << y << endl;
            datafile.close();
          }

          cout << "Position: (" << fixed << setprecision(3) << x << ", " << y
               << ") m                     \r";
        } else {
          cout << "Position: (not detected)                             \r";
        }
        cout.flush();

        // Check for key press to exit
        if (kbhit()) {
          swimming = false;
          ext_key(); // Consume the key
        }

        // Small delay to prevent excessive CPU usage
        Sleep(10);
      }

      cout << endl << "Swimming stopped." << endl;
      regs.set_reg_b(REG8_MODE, IMODE_IDLE);
      break;
    }

    case '8': {
      cout << "Starting interactive mode..." << endl;
      regs.set_reg_b(REG8_MODE, IMODE_SWIM);

      cout << "Use keyboard controls:" << endl;
      cout << "  W/S: Increase/decrease speed (frequency)" << endl;
      cout << "  A/D: Turn left/right (offset)" << endl;
      cout << "  Q: Return to menu" << endl;

      bool interactiveMode = true;
      while (interactiveMode) {
        if (kbhit()) {
          DWORD key = ext_key();
          char c = key & 0xFF;

          switch (c) {
          case 'w':
          case 'W':
            freq += 0.1f;
            update_parameter_force(regs, REG8_SINE_FREQ, MIN_FREQ, MAX_FREQ,
                                   freq);
            cout << "Frequency: " << freq << " Hz          \r";
            break;

          case 's':
          case 'S':
            freq = max(0.1f, freq - 0.1f);
            update_parameter_force(regs, REG8_SINE_FREQ, MIN_FREQ, MAX_FREQ,
                                   freq);
            cout << "Frequency: " << freq << " Hz          \r";
            break;

          case 'a':
          case 'A':
            offset -= 0.1f;
            update_parameter_force(regs, REG8_SINE_OFF, MIN_OFF, MAX_OFF,
                                   offset);
            cout << "Offset: " << offset << " degrees      \r";
            break;

          case 'd':
          case 'D':
            offset += 0.1f;
            update_parameter_force(regs, REG8_SINE_OFF, MIN_OFF, MAX_OFF,
                                   offset);
            cout << "Offset: " << offset << " degrees      \r";
            break;

          case 'q':
          case 'Q':
            interactiveMode = false;
            break;
          }
          cout.flush();
        }

        // Update tracking display
        uint32_t frame_time;
        if (trk.update(frame_time)) {
          double x, y;
          int id = trk.get_first_id();
          if (id != -1 && trk.get_pos(id, x, y)) {
            cout << "Position: (" << fixed << setprecision(3) << x << ", " << y
                 << ") m | Freq: " << freq << " Hz | Offset: " << offset
                 << "°   \r";
            cout.flush();
          }
        }

        Sleep(10); // Small delay to prevent excessive CPU usage
      }

      cout << endl << "Interactive mode stopped." << endl;
      regs.set_reg_b(REG8_MODE, IMODE_IDLE);
      break;
    }

    case '0':
      cout << "Stopping robot (idle mode)..." << endl;
      regs.set_reg_b(REG8_MODE, IMODE_IDLE);
      break;

    case 'q':
    case 'Q':
      exitProgram = true;
      cout << "\nExiting program." << endl;
      break;

    default:
      cout << "\nInvalid choice. Please try again." << endl;
      break;
    }
  }

  // Make sure the robot is stopped before exiting
  regs.set_reg_b(REG8_MODE, IMODE_IDLE);

  // Clears the console input buffer
  FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));

  return 0;
}
