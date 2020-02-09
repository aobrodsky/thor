#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/resource.h>

#include "../../vm/natives.h"
#include "../../vm/vm.h"
#include "../../common/consts.h"
#include "../../common/types.h"
#include "../../transport/buffer/vm-buffer.h"

#include "thor.h"
#include "robot.h"
#include "thymio.h"
#include "thymio_desc.h"
#include "thysim.h"
#include "logger.h"
#include "buffer.h"

typedef struct thymio_vm_state {
  uint16_t                      timers[2];
  uint16_t                      old_timer[2];
  struct _vmVariables         vmVariables;
  uint16_t                      vmBytecode[VM_BYTECODE_SIZE];
  int16_t                      vmStack[VM_STACK_SIZE];
  unsigned long               dt_total;
  int                         prev_tap;
  char                        prev_buttons[5];
} thymio_vm_state;

static void update_aseba_variables_read( robot_state *rs );
static void update_aseba_variables_write( robot_state *rs );
static void write_bytecode( AsebaVMState *vm );
static void init_state( robot_state *rs );
static void init_aseba_vars( robot_state *rs );
static void set_source( robot_state *rs, uint16_t src );
static uint32_t do_step( robot_state *rs, uint32_t dt );
static void delete_bot( robot_state *rs );

static int button_events[5] = { EVENT_B_BACKWARD, EVENT_B_LEFT, 
                EVENT_B_CENTER, EVENT_B_FORWARD, EVENT_B_RIGHT };

static robot_model thymio_model = {
  NULL,
  ROBOT_TYP_THYMIO,
  EVENT_COUNT,
  NULL,
  NULL,
  &ThymioGetVMDescription,
  &ThymioGetNativeFunctionsDescriptions,
  &ThymioGetLocalEventsDescriptions,
  &ThymioNativeFunction, 
  &write_bytecode,
  NULL,
  NULL,
  NULL,
  NULL,
  &init_state,
  &init_aseba_vars,
  &update_aseba_variables_read,
  &update_aseba_variables_write,
  &set_source,
  &do_step,
  &delete_bot,
  &thy_sim_set_position,
  &thy_sim_user_action,
};


static uint32_t do_step( robot_state *rs, uint32_t dt ) {
  thymio_vm_state *ts = rs->model_state;
  int i;
  int tap;
  int16_t buttons[5];

  rs->tick++;

  if( !rs->enabled ) { /* ignore disabled bots */
    return rs->tick;
  }

  // The following events are presently not supported.
  //  EVENT_MIC,
  //  EVENT_SOUND_FINISHED,
  //  EVENT_RC5,
  //  EVENT_PROX_COMM,
 
  ts->dt_total += dt;
  if( !( rs->tick % ( TICKS_PER_SEC / 100 ) ) ) { 
    // update position 100 Hz
    thy_sim_update_pos( rs, ts->dt_total );
    ts->dt_total = 0;
  }

  if( !( rs->tick % ( TICKS_PER_SEC / 100 ) ) ) {
    // Motor 100Hz
    thy_sim_get_speed( rs, ts->vmVariables.uind );
    SET_EVENT( rs, EVENT_MOTOR );
  }

  // When a button is pressed or released
  // activate corresponding button event.
  // Sample as often as possible, (1000Hz)
  thy_sim_get_buttons( rs, buttons );
  for( i = 0; i < 5; i++ ) {
    if( ts->prev_buttons[i] != buttons[i] ) {
      SET_EVENT( rs, button_events[i] );
      ts->prev_buttons[i] = buttons[i];
    }
  }

  if( !( rs->tick % ( TICKS_PER_SEC / 20 ) ) ) {
    // Buttons 20Hz
    // EVENT_BUTTONS,
    memcpy( ts->vmVariables.buttons_state, buttons, sizeof( int16_t ) * 5 );
    SET_EVENT( rs, EVENT_BUTTONS );
  }

  if( !( rs->tick % ( TICKS_PER_SEC / 10 ) ) ) {
    // Prox 10Hz
    thy_sim_get_prox_horiz( rs, ts->vmVariables.prox );
    thy_sim_get_prox_grnd( rs, ts->vmVariables.ground_ambiant, 
                           ts->vmVariables.ground_reflected,
                           ts->vmVariables.ground_delta );
    SET_EVENT( rs, EVENT_PROX );
  }

  tap = thy_sim_tap( rs );
  if( tap ) {
    // EVENT_TAP
    if( !ts->prev_tap ) { /* only handle one tap */
      thy_sim_get_acc( rs, ts->vmVariables.acc );
      SET_EVENT( rs, EVENT_TAP );
    }
  }
  ts->prev_tap = tap;

  if( !( rs->tick % ( TICKS_PER_SEC / 16 ) ) ) {
    // ACC 16Hz 
    thy_sim_get_acc( rs, ts->vmVariables.acc );
    SET_EVENT( rs, EVENT_ACC );
  }

  if( !( rs->tick % TICKS_PER_SEC ) ) {
    // Temperature 1Hz
    thy_sim_get_temp( rs, &ts->vmVariables.ntc );
    SET_EVENT( rs, EVENT_TEMPERATURE );
  }

  for( i = 0; i < 2; i++ ) {
    if( ts->timers[i] ) {
      if( ts->timers[i] < ( TICK / 1000 ) ) {
        ts->timers[i] = 0;
      } else {
        ts->timers[i] -= TICK / 1000;
      }
      if( !ts->timers[i] ) {
        SET_EVENT( rs, EVENT_TIMER0 + i );
        ts->timers[i] = ts->vmVariables.timers[i];
      }
    }
  }

  return rs->tick;
}


static void update_aseba_variables_read( robot_state *rs ) {
  /* Update all sensor variables, when we have sensors? */
  /* No, updates occur at tick times */
}


static void update_aseba_variables_write( robot_state *rs ) {
  thymio_vm_state *ts = rs->model_state;
  int i;

  for( i = 0; i < 2; i++ ) {
    if( ts->vmVariables.timers[i] != ts->old_timer[i]) {
      ts->old_timer[i] = ts->vmVariables.timers[i];
      ts->timers[i] = ts->old_timer[i];
    }
  }
  
  thy_sim_set_speed( rs, ts->vmVariables.target );
}


static void write_bytecode( AsebaVMState *vm ) {
  AsebaVMEmitNodeSpecificError( vm, "Flashing not supported" );
}


static void init_state( robot_state *rs ) {
  thymio_vm_state *ts;

  if( rs->model && ( rs->model != &thymio_model ) ) {
    if( rs->enabled ) {
      rs->model->delete( rs );
    }
    free( rs->model_state );
    rs->model_state = NULL;
    free( rs->physical_state );
    rs->physical_state = NULL;
  }

  if( !rs->model_state ) {
     rs->model_state = malloc( sizeof( thymio_vm_state ) );
  }
  ts = rs->model_state;
  
  if( !ts ) {
    byebye( "thymio init()" );
  }

  memset( ts, 0, sizeof( thymio_vm_state ) );
  rs->vmState.bytecodeSize = VM_BYTECODE_SIZE;
  rs->vmState.bytecode = ts->vmBytecode;
  rs->vmState.variablesSize = sizeof( ts->vmVariables ) / sizeof( int16_t );
  rs->vmState.variables = (int16_t*) &ts->vmVariables;
  rs->vmState.stackSize = VM_STACK_SIZE;
  rs->vmState.stack = ts->vmStack;

  thy_sim_init_state( rs );
  rs->enabled = 1;
}


static void init_aseba_vars( robot_state *rs ) {
  thymio_vm_state *ts = rs->model_state;
  ts->vmVariables.id = rs->vmState.nodeId;
  ts->vmVariables.productid = ASEBA_PID_THYMIO2;
  ts->vmVariables.fwversion[0] = FIRMWARE_VER_LOW;
  ts->vmVariables.fwversion[1] = FIRMWARE_VER_HIGH;
}


static void set_source( robot_state *rs, uint16_t src ) {
  thymio_vm_state *ts = rs->model_state;
  ts->vmVariables.source = src;
}


static void delete_bot( robot_state *rs ) {
  rs->enabled = 0;
  thy_sim_delete( rs );
}


extern void thymio_init_model() {
  thy_sim_init( &thymio_model );  
  robot_add_model( &thymio_model );
} 
