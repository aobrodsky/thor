#ifndef ROBOT_H
#define ROBOT_H

#include <sys/time.h>

#include "../../vm/vm.h"
#include "../../vm/natives.h"
#include "../../common/productids.h"
#include "../../common/consts.h"
#include "../../transport/buffer//vm-buffer.h"

#include "world.h"
#include "msg_com.h"

#define TICK 1000
#define TICKS_PER_SEC (1000000 / TICK)
#if TICK < 1000
  #error "Clock tick too short."
#endif

#define SET_EVENT( rs, i )  (rs)->event_flags |= 1 << (i)
#define RESET_EVENT( rs, i )  (rs)->event_flags &= ~(1 << (i))
#define ISSET_EVENT( rs, i )  ((rs)->event_flags & (1 << (i)))
#define WAITING_EVENTS( rs ) ((rs)->event_flags)

typedef struct robot_state {
  struct robot_state *next;
  struct robot_model *model;
  void               *model_state;
  void               *physical_state;
  void               *send_state;
  void               *prev_state;
  unsigned long       event_flags;
  int                 incoming_events;
  AsebaVMState        vmState;
  int                 next_event;
  unsigned long       tick;
  int                 enabled;
} robot_state;

typedef const AsebaVMDescription * (*robot_vm)(AsebaVMState *);
typedef const AsebaNativeFunctionDescription * const * 
                       (*robot_native_funcs)(AsebaVMState *);
typedef const AsebaLocalEventDescription * (*robot_events)(AsebaVMState *);
typedef void (*robot_error)(AsebaVMState *, const char *);
typedef void (*robot_function_vm)(AsebaVMState *);
typedef void (*robot_function_vm16)(AsebaVMState *, uint16_t);
typedef void (*robot_function_rs16)(robot_state *, uint16_t);
typedef uint32_t (*robot_function_rs32)(robot_state *, uint32_t);
typedef void (*robot_function_rs)(robot_state *);
typedef int (*robot_function_rsva)(robot_state *, tw_vect *, tw_angle);
typedef void (*robot_function_rsp)(robot_state *, void *);

typedef struct robot_model {
  struct robot_model  *next;
  uint16_t             typ;
  uint16_t             event_count;
  struct msg_com_fmt  *state_fmt;
  struct msg_com_fmt  *input_fmt;
  robot_vm             get_vm_desc;
  robot_native_funcs   get_func_desc;
  robot_events         get_local_events_desc;
  robot_function_vm16  native_function;
  robot_function_vm    write_bytecode;
  robot_function_vm    reset_into_bootloader;
  robot_function_vm    vm_run_cb;
  robot_error          vm_error_cb;
  robot_function_vm    vm_reset_cb;
  robot_function_rs    init_state;
  robot_function_rs    init_aseba_vars;
  robot_function_rs    read_aseba_vars;
  robot_function_rs    write_aseba_vars;
  robot_function_rs16  set_source;
  robot_function_rs32  do_step;
  robot_function_rs    delete;
  robot_function_rsva  set_position;
  robot_function_rsp   user_action;
} robot_model;

extern robot_state * robot_get_state( uint16_t id );
extern robot_model * robot_get_model( uint16_t typ );
extern robot_state * robot_add( uint16_t id, uint16_t typ );
extern robot_model * robot_get_model_list();
extern void robot_add_model( robot_model *m );
extern void robot_init( int add_bot );
extern void robot_step();
extern void robot_incoming_event();
extern int robot_event_pending( struct timeval *tv );
extern void robot_disable_all();

#endif
