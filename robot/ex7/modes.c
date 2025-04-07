#include "config.h"
#include "hardware.h"
#include "modes.h"
#include "module.h"
#include "registers.h"
#include "robot.h"

// Define registers for frequency and amplitude control
#define REG8_SINE_FREQ 10 // Register for sine wave frequency
#define REG8_SINE_AMP 11  // Register for sine wave amplitude
#define REG8_SINE_LAG 12  // Register for sine wave lag between elements
#define REG8_SINE_OFF 13  // Register for sine wave offset 

// Define limits for frequency and amplitude
#define MAX_FREQ 1.5f // Maximum frequency in Hz
//#define MIN_FREQ 0.0f // Minimum frequency in Hz
#define MAX_AMP 60.0f // Maximum amplitude in degrees
//#define MIN_AMP 0.0f // Minimum amplitude in degrees
#define MAX_LAG 1.5f // Maximum lag between elements in degrees
//#define MIN_LAG 0.0f // Minimum lag between elements in degrees
#define MAX_OFF 3.0f // Maximum lag between elements in degrees (180 mean straight for float conversion)
//#define MIN_OFF -180.0f // Minimum lag between elements in degrees

#define MIN_FREQ 0.1f
#define MIN_AMP 1.0f  // Min amplitude in degrees
#define MIN_LAG 0.5f // Min lag between elements in degrees
#define MIN_OFF -3.0f // Min offset in degrees

// Default values
#define DEFAULT_FREQ 0.8f // Default frequency in Hz
#define DEFAULT_AMP 40.0f // Default amplitude in degrees
#define DEFAULT_LAG 0.75f // Default amplitude in degrees
#define DEFAULT_OFF 0.0f // Default amplitude in degrees


const uint8_t MOTOR_ADDR_HEAD = 25; // Motor address (this is the second element, the first is the head that is actuated by this one)
const uint8_t MOTOR_ADDR_NECK = 22; // Motor address
const uint8_t MOTOR_ADDR_TORSO = 24; // Motor address
const uint8_t MOTOR_ADDR_HIP = 26; // Motor address
const uint8_t MOTOR_ADDR_TAIL = 5; // Motor address

uint8_t freq_enc = DEFAULT_FREQ;
uint8_t amp_enc = DEFAULT_AMP;
uint8_t lag_enc = DEFAULT_LAG;
uint8_t off_enc = DEFAULT_OFF;

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
    case REG8_SINE_LAG:
      switch (operation) {
      case ROP_READ_8:
        radio_data->byte = lag_enc;
        return TRUE;
      case ROP_WRITE_8:
        lag_enc = radio_data->byte; // Allow writing to register
        return TRUE;
      }
    case REG8_SINE_OFF:
      switch (operation) {
      case ROP_READ_8:
        radio_data->byte = off_enc;
        return TRUE;
      case ROP_WRITE_8:
        off_enc = radio_data->byte; // Allow writing to register
        return TRUE;
      }
  }
  return FALSE;

}

// Function to initialize default parameters
void init_sine_params(void) {
  // Set default values for frequency and amplitude
  freq_enc = ENCODE_PARAM_8(DEFAULT_FREQ, MIN_FREQ, MAX_FREQ);
  amp_enc = ENCODE_PARAM_8(DEFAULT_AMP, MIN_AMP, MAX_AMP);
  lag_enc = ENCODE_PARAM_8(DEFAULT_LAG, MIN_LAG, MAX_LAG);
  off_enc = ENCODE_PARAM_8(DEFAULT_OFF, MIN_OFF, MAX_OFF);
}

void swim_mode(void) {
  uint32_t dt, cycletimer;
  float my_time, delta_t, angle0, angle1, angle2, angle3, angle4;
  float freq, amplitude, lag, offset;
  int8_t angle0_rounded, angle1_rounded, angle2_rounded, angle3_rounded, angle4_rounded;

  // Initialize and start the motor's PID controller
  init_body_module(MOTOR_ADDR_HEAD);
  init_body_module(MOTOR_ADDR_NECK);
  init_body_module(MOTOR_ADDR_TORSO);
  init_body_module(MOTOR_ADDR_HIP);
  init_body_module(MOTOR_ADDR_TAIL);

  start_pid(MOTOR_ADDR_HEAD);
  start_pid(MOTOR_ADDR_NECK);
  start_pid(MOTOR_ADDR_TORSO);
  start_pid(MOTOR_ADDR_HIP);
  start_pid(MOTOR_ADDR_TAIL);

  set_reg_value_dw(MOTOR_ADDR_HEAD, MREG32_LED, 0);
  set_reg_value_dw(MOTOR_ADDR_NECK, MREG32_LED, 0);
  set_reg_value_dw(MOTOR_ADDR_TORSO, MREG32_LED, 0);
  set_reg_value_dw(MOTOR_ADDR_HIP, MREG32_LED, 0);
  set_reg_value_dw(MOTOR_ADDR_TAIL, MREG32_LED, 0);

  // Set visual indicator that motor is active
  set_color(4); // Set LED to red

  // Initialize sine wave time
  cycletimer = getSysTICs();
  my_time = 0;

  do {
    // Decode current parameters from registers
    freq = DECODE_PARAM_8(freq_enc, MIN_FREQ, MAX_FREQ);
    amplitude = DECODE_PARAM_8(amp_enc, MIN_AMP, MAX_AMP);
    lag = DECODE_PARAM_8(lag_enc, MIN_LAG, MAX_LAG);
    offset = DECODE_PARAM_8(off_enc, MIN_OFF, MAX_OFF);

    // Apply limits to ensure safety
    if (freq > MAX_FREQ)
      freq = MAX_FREQ;
    if (amplitude > MAX_AMP)
      amplitude = MAX_AMP;
    if (lag > MAX_LAG)
      lag = MAX_LAG;
    if (offset > MAX_OFF)
      offset = MAX_OFF;

    // Calculate elapsed time
    dt = getElapsedSysTICs(cycletimer);
    cycletimer = getSysTICs();
    delta_t = (float)dt / sysTICSperSEC;
    my_time += delta_t;

    // Calculate the sine wave for motor angle
    angle0 = amplitude * sin(M_TWOPI * ((freq * my_time)+(0*lag/5)+offset));
    angle1 = amplitude * sin(M_TWOPI * ((freq * my_time)+(1*lag/5)+offset));
    angle2 = amplitude * sin(M_TWOPI * ((freq * my_time)+(2*lag/5)+offset));
    angle3 = amplitude * sin(M_TWOPI * ((freq * my_time)+(3*lag/5)+offset));
    angle4 = amplitude * sin(M_TWOPI * ((freq * my_time)+(4*lag/5)+offset));

    // Convert angle to motor units
    angle0_rounded = DEG_TO_OUTPUT_BODY(angle0);
    angle1_rounded = DEG_TO_OUTPUT_BODY(angle1);
    angle2_rounded = DEG_TO_OUTPUT_BODY(angle2);
    angle3_rounded = DEG_TO_OUTPUT_BODY(angle3);
    angle4_rounded = DEG_TO_OUTPUT_BODY(angle4);

    // Send the angle to the motor
    bus_set(MOTOR_ADDR_HEAD, MREG_SETPOINT, angle4_rounded);
    bus_set(MOTOR_ADDR_NECK, MREG_SETPOINT, angle3_rounded);
    bus_set(MOTOR_ADDR_TORSO, MREG_SETPOINT, angle2_rounded);
    bus_set(MOTOR_ADDR_HIP, MREG_SETPOINT, angle1_rounded);
    bus_set(MOTOR_ADDR_TAIL, MREG_SETPOINT, angle0_rounded);

    set_rgb(255, 255, 255);

    // Small delay to ensure timer updates properly
    pause(ONE_MS);

  } while (reg8_table[REG8_MODE] == IMODE_SWIM);

  // Clean up: return motor to zero position
  bus_set(MOTOR_ADDR_HEAD, MREG_SETPOINT, 0);
  bus_set(MOTOR_ADDR_NECK, MREG_SETPOINT, 0);
  bus_set(MOTOR_ADDR_TORSO, MREG_SETPOINT, 0);
  bus_set(MOTOR_ADDR_HIP, MREG_SETPOINT, 0);
  bus_set(MOTOR_ADDR_TAIL, MREG_SETPOINT, 0);

  pause(ONE_SEC); // Give the motor time to return to center

  // Stop the motor
  bus_set(MOTOR_ADDR_HEAD, MREG_MODE, MODE_IDLE);
  bus_set(MOTOR_ADDR_NECK, MREG_MODE, MODE_IDLE);
  bus_set(MOTOR_ADDR_TORSO, MREG_MODE, MODE_IDLE);
  bus_set(MOTOR_ADDR_HIP, MREG_MODE, MODE_IDLE);
  bus_set(MOTOR_ADDR_TAIL, MREG_MODE, MODE_IDLE);

  // Return LED to normal state
  set_color(2);
}

void ready_mode(void) {
  // Initialize and start the motor's PID controller
  init_body_module(MOTOR_ADDR_HEAD);
  init_body_module(MOTOR_ADDR_NECK);
  init_body_module(MOTOR_ADDR_TORSO);
  init_body_module(MOTOR_ADDR_HIP);
  init_body_module(MOTOR_ADDR_TAIL);

  start_pid(MOTOR_ADDR_HEAD);
  start_pid(MOTOR_ADDR_NECK);
  start_pid(MOTOR_ADDR_TORSO);
  start_pid(MOTOR_ADDR_HIP);
  start_pid(MOTOR_ADDR_TAIL);

  set_reg_value_dw(MOTOR_ADDR_HEAD, MREG32_LED, 0);
  set_reg_value_dw(MOTOR_ADDR_NECK, MREG32_LED, 0);
  set_reg_value_dw(MOTOR_ADDR_TORSO, MREG32_LED, 0);
  set_reg_value_dw(MOTOR_ADDR_HIP, MREG32_LED, 0);
  set_reg_value_dw(MOTOR_ADDR_TAIL, MREG32_LED, 0);
  set_rgb(255, 255, 255);


  // Send the angle to the motor
  bus_set(MOTOR_ADDR_HEAD, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(40)); // adopt a rigid S shape to prevent capsizing
  bus_set(MOTOR_ADDR_NECK, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(40));
  bus_set(MOTOR_ADDR_TORSO, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(-40));
  bus_set(MOTOR_ADDR_HIP, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(-40));
  bus_set(MOTOR_ADDR_TAIL, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(40));

  do { // wait until we start swimming or revert to limp mode.
    // Small delay to ensure timer updates properly
    pause(ONE_MS);

  } while (reg8_table[REG8_MODE] == IMODE_READY);

  // Clean up: return motor to zero position
  bus_set(MOTOR_ADDR_HEAD, MREG_SETPOINT, 0);
  bus_set(MOTOR_ADDR_NECK, MREG_SETPOINT, 0);
  bus_set(MOTOR_ADDR_TORSO, MREG_SETPOINT, 0);
  bus_set(MOTOR_ADDR_HIP, MREG_SETPOINT, 0);
  bus_set(MOTOR_ADDR_TAIL, MREG_SETPOINT, 0);

  pause(ONE_SEC); // Give the motor time to return to center

  // Stop the motor
  bus_set(MOTOR_ADDR_HEAD, MREG_MODE, MODE_IDLE);
  bus_set(MOTOR_ADDR_NECK, MREG_MODE, MODE_IDLE);
  bus_set(MOTOR_ADDR_TORSO, MREG_MODE, MODE_IDLE);
  bus_set(MOTOR_ADDR_HIP, MREG_MODE, MODE_IDLE);
  bus_set(MOTOR_ADDR_TAIL, MREG_MODE, MODE_IDLE);

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
    case IMODE_READY :
      ready_mode();
      break;
    case IMODE_SWIM:
      swim_mode();
      break;
    default:
      reg8_table[REG8_MODE] = IMODE_IDLE;
    }
  }
}
