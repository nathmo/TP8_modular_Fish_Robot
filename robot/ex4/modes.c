#include "modes.h"
#include "config.h"
#include "hardware.h"
#include "module.h"
#include "regdefs.h"
#include "registers.h"
#include "robot.h"
#include "sysTime.h"

const uint8_t MOTOR_ADDR = 21;
int8_t motor_position = 0;

static int8_t register_handler(uint8_t operation, uint8_t address,
                               RadioData *radio_data) {

  if (address == REG8_MODE) {
    switch (operation) {
    case ROP_READ_8:
      radio_data->byte = reg8_table[REG8_MODE];
      return TRUE;
    case ROP_WRITE_8:
      reg8_table[REG8_MODE] = radio_data->byte; // Allow writing to register
      return TRUE;
    }
  }
  if (address == 0x06) {
    switch (operation) {
    case ROP_READ_8:
      radio_data->byte = motor_position;
      return TRUE;
    case ROP_WRITE_8:
      motor_position = radio_data->byte; // Allow writing to register
      return TRUE;
    }
  }
  return FALSE;
}

void motor_demo_mode() {
  init_body_module(MOTOR_ADDR);
  start_pid(MOTOR_ADDR);
  set_color(4);
  while (reg8_table[REG8_MODE] == IMODE_MOTOR_DEMO) {
    bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(21.0));
    pause(ONE_SEC);
    bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(-21.0));
    pause(ONE_SEC);
  }
  bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(0.0));
  pause(ONE_SEC);
  bus_set(MOTOR_ADDR, MREG_MODE, MODE_IDLE);
  set_color(2);
}

void motor_sine_demo() {
  init_body_module(MOTOR_ADDR);
  start_pid(MOTOR_ADDR);
  set_color(3);
  while (reg8_table[REG8_MODE] == IMODE_SINE_DEMO) {
    bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(motor_position));
    pause(TEN_MS);
  }
  bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(0.0));
  pause(ONE_SEC);
  bus_set(MOTOR_ADDR, MREG_MODE, MODE_IDLE);
  set_color(2);
}

void main_mode_loop() {
  reg8_table[REG8_MODE] = IMODE_IDLE;
  radio_add_reg_callback(register_handler);
  while (1) {
    switch (reg8_table[REG8_MODE]) {
    case IMODE_IDLE:
      break;
    case IMODE_MOTOR_DEMO:
      motor_demo_mode();
      break;
    case IMODE_SINE_DEMO:
      motor_sine_demo();
    default:
      reg8_table[REG8_MODE] = IMODE_IDLE;
    }
  }
}
