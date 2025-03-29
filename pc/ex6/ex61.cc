#include "remregs.h"
#include "robot.h"
#include "trkcli.h"
#include "utils.h"
#include <cstdlib>
#include <iostream>
#include <stdint.h>
#include <windows.h>

using namespace std;

const char *TRACKING_PC_NAME = "biorobpc11"; ///< host name of the tracking PC
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

  cout << "Connected to tracking system. Turn off other module LEDs and place "
          "the robot in the aquarium."
       << endl;
  cout << "Press any key to exit." << endl;

  // Turn off the LED of another module (assuming module address 21)
  const uint8_t MOTOR_ADDR = 21;
  // This code doesn't work directly from PC, needs to be on the robot side
  // Will be handled in the robot part of the solution

  // Fixed green component for tracking
  const uint8_t GREEN_COMPONENT = 64;

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
      // Calculate LED color based on position
      // Red increases with x (left to right)
      uint8_t r = (uint8_t)map_value(x, 0, AQUARIUM_WIDTH, 0, 255);

      // Blue increases with y (bottom to top)
      uint8_t b = (uint8_t)map_value(y, 0, AQUARIUM_HEIGHT, 0, 255);

      // Keep green fixed for tracking
      uint8_t g = GREEN_COMPONENT;

      // Set the LED color
      uint32_t rgb = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
      regs.set_reg_dw(REG32_LED, rgb);

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
