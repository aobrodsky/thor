#ifndef GLOBAL_H
#define GLOBAL_H

#define SET_FLAG(x) global_state |= (x)
#define ISSET_FLAG(x) ((global_state & (x)) != 0)
#define RESET_FLAG(x) global_state &= ~(x)

#define GLOBAL_ALLOW_REMOTE 0x00000001
#define GLOBAL_V1_PORT      0x00000002
#define GLOBAL_QUIET        0x00000004
#define GLOBAL_DEBUG        0x00000008
#define GLOBAL_HISPEED      0x00000010
#define GLOBAL_RECORD       0x00000020
#define GLOBAL_LOG_MORE     0x00000040

extern int global_state;
extern int performance;

#endif
