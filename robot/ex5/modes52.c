#include "config.h"
#include "hardware.h"
#include "modes.h"
#include "module.h"
#include "registers.h"
#include "robot.h"

// Define registers for frequency and amplitude control
#define REG8_SINE_FREQ 10 // Register for sine wave frequency
#define REG8_SINE_AMP 11  // Register for sine wave amplitude

// Define limits for frequency and amplitude
#define MAX_FREQ 2.0f // Maximum frequency in Hz
#define MAX_AMP 60.0f // Maximum amplitude in degrees

// Default values
#define DEFAULT_FREQ 1.0f // Default frequency in Hz
#define DEFAULT_AMP 40.0f // Default amplitude in degrees

const uint8_t MOTOR_ADDR = 21; // Motor address

uint8_t freq_enc = DEFAULT_FREQ;
uint8_t amp_enc = DEFAULT_AMP;


static int8_t register_handler(uint8_t operation, uint8_t address,
                               RadioData *radio_data) {

  switch (address) {
    case REG8_MODE:
      switch (operation) {
      case ROP_READ_8:
        radio_data->byte = reg8_table[REG8_MODE];
        return TRUE;
      case ROP_WRITE_8:
        reg8_table[REG8_MODE] = radio_data->byte; // Allow writing to register
        return TRUE;
      }
    case REG8_SINE_FREQ:
      switch (operation) {
      case ROP_READ_8:
        radio_data->byte = freq_enc;
        return TRUE;
      case ROP_WRITE_8:
        freq_enc = radio_data->byte; // Allow writing to register
        return TRUE;
      }
    case REG8_SINE_AMP:
      switch (operation) {
      case ROP_READ_8:
        radio_data->byte = amp_enc;
        return TRUE;
      case ROP_WRITE_8:
        amp_enc = radio_data->byte; // Allow writing to register
        return TRUE;
      }
  }
  return FALSE;

}

// Function to initialize default parameters
void init_sine_params(void) {
  // Set default values for frequency and amplitude
  reg8_table[REG8_SINE_FREQ] = ENCODE_PARAM_8(DEFAULT_FREQ, 0.1f, MAX_FREQ);
  reg8_table[REG8_SINE_AMP] = ENCODE_PARAM_8(DEFAULT_AMP, 1.0f, MAX_AMP);
}

void sine_demo_mode(void) {
  uint32_t dt, cycletimer;
  float my_time, delta_t, angle;
  float freq, amplitude;
  int8_t angle_rounded;

  // Initialize and start the motor's PID controller
  init_body_module(MOTOR_ADDR);
  start_pid(MOTOR_ADDR);

  // Set visual indicator that motor is active
  set_color(4); // Set LED to red

  // Initialize sine wave time
  cycletimer = getSysTICs();
  my_time = 0;

  // Make sure parameters are initialized
  if (freq_enc == 0)
    freq_enc = ENCODE_PARAM_8(DEFAULT_FREQ, 0.1f, MAX_FREQ);
  if (amp_enc == 0)
    amp_enc = ENCODE_PARAM_8(DEFAULT_AMP, 1.0f, MAX_AMP);

  do {
    // Decode current parameters from registers
    freq = DECODE_PARAM_8(freq_enc, 0.1f, MAX_FREQ);
    amplitude = DECODE_PARAM_8(amp_enc, 1.0f, MAX_AMP);

    // Apply limits to ensure safety
    if (freq > MAX_FREQ)
      freq = MAX_FREQ;
    if (amplitude > MAX_AMP)
      amplitude = MAX_AMP;

    // Calculate elapsed time
    dt = getElapsedSysTICs(cycletimer);
    cycletimer = getSysTICs();
    delta_t = (float)dt / sysTICSperSEC;
    my_time += delta_t;

    // Calculate the sine wave for motor angle
    angle = amplitude * sin(M_TWOPI * freq * my_time);

    // Convert angle to motor units
    angle_rounded = DEG_TO_OUTPUT_BODY(angle);

    // Send the angle to the motor
    bus_set(MOTOR_ADDR, MREG_SETPOINT, angle_rounded);

    // Update LED for visual feedback - color indicates frequency,
    // brightness indicates amplitude
    uint8_t red = (uint8_t)(freq * 127.0f / MAX_FREQ);
    uint8_t green = (uint8_t)(amplitude * 127.0f / MAX_AMP);

    if (angle >= 0) {
      // Positive angle - more green
      set_rgb(red, green + 20, 20);
    } else {
      // Negative angle - more red
      set_rgb(red + 20, green, 20);
    }

    // Small delay to ensure timer updates properly
    pause(ONE_MS);

  } while (reg8_table[REG8_MODE] == IMODE_SINE_DEMO);

  // Clean up: return motor to zero position
  bus_set(MOTOR_ADDR, MREG_SETPOINT, 0);
  pause(ONE_SEC); // Give the motor time to return to center

  // Stop the motor
  bus_set(MOTOR_ADDR, MREG_MODE, MODE_IDLE);

  // Return LED to normal state
  set_color(2);
}

void main_mode_loop(void) {
  // Initialize the default parameters
  init_sine_params();

  // Set initial mode
  reg8_table[REG8_MODE] = IMODE_IDLE;


  // Add the register handler
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
