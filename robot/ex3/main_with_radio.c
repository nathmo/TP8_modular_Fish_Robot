#include "hardware.h"
#include "registers.h"
#include "module.h"
#include "robot.h"

const uint8_t TAIL_MOTOR_ADDR = 21;
const uint8_t BODY_MOTOR_ADDR = 72;
const uint8_t BODY_MOTOR_FIN_LEFT = 73; // swap if not correct
const uint8_t BODY_MOTOR_FIN_RIGHT = 74;

static uint8_t motor_position = 0;
static uint8_t mb_buffer[4];
static uint8_t last_mb_size = 4;

/* Register callback function for 8-bit read/write operations */
static int8_t register_handler(uint8_t operation, uint8_t address, RadioData* radio_data)
{
    uint8_t i;
    switch (operation) {
    case ROP_READ_8:
        if (address == 6) {
            radio_data->byte = motor_position;
            return TRUE;
        }
    case ROP_WRITE_8:
        if (address == 6) {
            motor_position = radio_data->byte;  // Allow writing to register
            return TRUE;
        }
    case ROP_READ_MB:
        if (address == 2) {
            radio_data->multibyte.size = last_mb_size;
            for (i = 0; i < last_mb_size; i++) {
            radio_data->multibyte.data[i] = mb_buffer[i];
            }
            return TRUE;
        }
        break;
    case ROP_WRITE_MB:
        if (address == 2) {
            last_mb_size = radio_data->multibyte.size;
            for (i = 0; i < last_mb_size; i++) {
                mb_buffer[i] = radio_data->multibyte.data[i];
            }
            return TRUE;
        }
        break;
    }
    return FALSE;
}

int main(void)
{
   
    hardware_init();
    radio_add_reg_callback(register_handler); // Register the 8-bit handler
    init_body_module(TAIL_MOTOR_ADDR);
    init_body_module(BODY_MOTOR_ADDR);
    init_limb_module(BODY_MOTOR_FIN_LEFT);
    init_limb_module(BODY_MOTOR_FIN_RIGHT);

    // Indicate boot sequence
    set_color_i(4, 0);
    pause(ONE_SEC);
    set_color_i(2, 0);
    
    while (1) {
        motor_position = bus_get(TAIL_MOTOR_ADDR, MREG_POSITION); // Store position in register
        mb_buffer[0] = bus_get(TAIL_MOTOR_ADDR, MREG_POSITION); // Store position in register
        mb_buffer[1] = bus_get(BODY_MOTOR_ADDR, MREG_POSITION); // Store position in register
        mb_buffer[2] = bus_get(BODY_MOTOR_FIN_LEFT, MREG_POSITION); // Store position in register
        mb_buffer[3] = bus_get(BODY_MOTOR_FIN_RIGHT, MREG_POSITION); // Store position in register
        set_rgb((motor_position < 0) ? -motor_position : motor_position, 32, 0);
    }
    return 0;
}