#ifndef __MODES_H
#define __MODES_H

/// Idle mode: do nothing
#define IMODE_IDLE          0

/// set the robot in a warped and rigid position (prevent capsizing)
#define IMODE_READY    1

/// active swimming mode
#define IMODE_SWIM     2

/// The IDLE mode switching
void main_mode_loop(void);
/// The swimming mode (active motor movement)
void swim_mode(void);
/// The ready mode (move to a safe state to prevent capsize)
void ready_mode(void);

void init_sine_params(void);

#endif // __MODES_H
