#ifndef REPLAY_H
#define REPLAY_H

#define THOR_REPLAY_PORT 35333

extern void replay_start_log();
extern void replay_start_rerun( int fd, int hispeed );
extern void replay_checkpoint();
extern void replay_append( void *data, int len );
extern void replay_flush();

#endif
