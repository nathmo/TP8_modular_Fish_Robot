#include "config.h"
#include "hardware.h"
#include "modes.h"
#include "module.h"
#include "registers.h"
#include "robot.h"

const float FREQ = 1.0;        // Hz
const uint8_t MOTOR_ADDR = 21; // Motor address

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

void sine_demo_mode() {
  uint32_t dt, cycletimer;
  float my_time, delta_t, angle;
  int8_t angle_rounded;

  // Initialize and start the motor's PID controller
  init_body_module(MOTOR_ADDR);
  start_pid(MOTOR_ADDR);
  set_color(4); // Set LED to red to indicate motor is active

  cycletimer = getSysTICs();
  my_time = 0;

  do {
    // Calculates the delta_t in seconds and adds it to the current time
    dt = getElapsedSysTICs(cycletimer);
    cycletimer = getSysTICs();
    delta_t = (float)dt / sysTICSperSEC;
    my_time += delta_t;

    // Calculates the sine wave for motor angle (±40 degrees)
    angle = 40.0 * sin(M_TWOPI * FREQ * my_time);

    // Convert angle to motor units
    angle_rounded = DEG_TO_OUTPUT_BODY(angle);

    // Send the angle to the motor
    bus_set(MOTOR_ADDR, MREG_SETPOINT, angle_rounded);

    // Also update LED for visual feedback (similar to original)
    if (angle >= 0) {
      set_rgb(0, angle / 2 + 20, 32); // Brighter green as angle increases
    } else {
      set_rgb(-angle / 2 + 20, 0, 32); // Brighter red as angle decreases
    }

    // Make sure there is some delay, so that the timer output is not zero
    pause(ONE_MS);

  } while (reg8_table[REG8_MODE] == IMODE_SINE_DEMO);

  // Clean up: set motor to zero position
  bus_set(MOTOR_ADDR, MREG_SETPOINT, 0);
  pause(ONE_SEC); // Let motor return to center

  // Stop the motor
  bus_set(MOTOR_ADDR, MREG_MODE, MODE_IDLE);

  // Back to the "normal" green
  set_color(1);
}

void main_mode_loop() {
  reg8_table[REG8_MODE] = IMODE_IDLE;
  radio_add_reg_callback(register_handler);

  while (1) {
    switch (reg8_table[REG8_MODE]) {
    case IMODE_IDLE:
      break;
    case IMODE_SINE_DEMO:
      sine_demo_mode();
      break;
    default:
      reg8_table[REG8_MODE] = IMODE_IDLE;
    }
  }
}
