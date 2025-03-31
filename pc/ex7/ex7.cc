#include "remregs.h"
#include "robot.h"
#include "trkcli.h"
#include "utils.h"
#include "regdefs.h"
#include <cstdlib>
#include <iostream>
#include <stdint.h>
#include <windows.h>

using namespace std;

/*
the code has 4 component : 
the localisation, get the robot X,Y position and log the position over time
the terminal, allow to control the robot and change mode (stop, active, key mapping to control parameters)
the radio, send parameters to the robot
the business logic:
  Mode A -> implement the robot behaviour (steering (sinus offset),phase lag (between tail element), speed/frequency, amplitude)
*/

// Define registers for frequency, lag, offset and amplitude control
#define REG8_SINE_FREQ 10 // Register for sine wave frequency
#define REG8_SINE_AMP 11  // Register for sine wave amplitude
#define REG8_SINE_LAG 12  // Register for sine wave lag between elements
#define REG8_SINE_OFF 13  // Register for sine wave offset 

const char *TRACKING_PC_NAME = "biorobpc6"; ///< host name of the tracking PC
const uint16_t TRACKING_PORT = 10502;        ///< port number of the tracking PC
const uint8_t RADIO_CHANNEL = 201;           ///< robot radio channel
const char *INTERFACE = "COM1";              ///< robot radio interface

// Aquarium dimensions in meters
const double AQUARIUM_WIDTH = 6.0;
const double AQUARIUM_HEIGHT = 2.0;

// Function to map a value from one range to another
double map_value(double value, double in_min, double in_max, double out_min,
                 double out_max) {
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

  cout << "Connected to tracking system. set the robot to ready mode "
          "then place it in the aquarium."
       << endl;
  cout << "Press q to exit at any time" << endl;

  while (!kbhit()) {
    uint32_t frame_time;
    // Gets the current position
    if (!trk.update(frame_time)) {
      cerr << "Error updating tracking data" << endl;
      return 1;
    }

    double x, y;

    // Gets the ID of the first spot
    int id = trk.get_first_id();

    // Reads its coordinates (if (id == -1), then no spot is detected)
    if (id != -1 && trk.get_pos(id, x, y)) {
      regs.get_reg_b(REG8_SINE_FREQ)
      regs.get_reg_b(REG8_SINE_FREQ)
      cout << "Position: (" << fixed << x << ", " << y << ") m  |  Color: RGB("
           << (int)r << ", " << (int)g << ", " << (int)b << ")      \r";
      cout.flush();
    } else {
      cout << "Position: (not detected)                             \r";
      cout.flush();
    }

    // Wait 10 ms before getting the info next time
    Sleep(10);
  }

  // Clears the console input buffer (as kbhit() doesn't)
  FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
  cout << endl << "Program terminated." << endl;

  return 0;
}
