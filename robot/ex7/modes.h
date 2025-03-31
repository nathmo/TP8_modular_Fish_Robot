#ifndef __MODES_H
#define __MODES_H

/// Idle mode: do nothing
#define IMODE_IDLE          0

/// set the robot in a warped and rigid position (prevent capsizing)
#define IMODE_READY    1

/// active swimming mode
#define IMODE_SWIM     2

/// The main loop for mode switching
void main_mode_loop(void);

#endif // __MODES_H
