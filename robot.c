#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/resource.h>
#include <sys/time.h>

#include "../../vm/natives.h"
#include "../../vm/vm.h"
#include "../../common/consts.h"
#include "../../common/types.h"
#include "../../transport/buffer/vm-buffer.h"

#include "thor.h"
#include "robot.h"
#include "sim.h"
#include "logger.h"
#include "buffer.h"
#include "global.h"

#define MODEL_TAB_SIZE 65536

static robot_model *model_list;
static robot_state *state_tab[MODEL_TAB_SIZE];
static robot_state *state_list;
static struct timeval prev_tick;

extern robot_model *robot_get_model( uint16_t typ ) {
  robot_model *m;

  for( m = model_list; m; m = m->next ) {
    if( m->typ == typ ) {
      break;
    }
  }
  return m;
}

extern void robot_add_model( robot_model *m ) {
  if( m ) {
    m->next = model_list;
    model_list = m;
  }
}

extern robot_model * robot_get_model_list() {
  return model_list;
}

extern robot_state * robot_get_state( uint16_t id ) {
  return state_tab[id];
}

extern void robot_time_check() {
  static struct timeval old_tick;
  long diff = (prev_tick.tv_sec - old_tick.tv_sec) * 1000000 +
           (prev_tick.tv_usec - old_tick.tv_usec);
  old_tick = prev_tick;
  if( diff > 500000 ) {
    printf( "Delay: %ld\n", diff );
  }
}


static int next_local_event( robot_state *rs ) {
  int i;

  for( i = 0; i < rs->model->event_count; i++ ) {
    rs->next_event = ( rs->next_event + 1 ) % rs->model->event_count;
    if( ISSET_EVENT( rs, rs->next_event ) ) {
      RESET_EVENT( rs, rs->next_event );
      return ASEBA_EVENT_LOCAL_EVENTS_START - rs->next_event;
    }
  }
  return -1; /* Should never reach this */
}


static void rusage() {
  static struct timeval lasttime;
  struct rusage r;
  static double prev;
  double curr;
  double interval = ( prev_tick.tv_sec - lasttime.tv_sec ) * 1000000 +
                  ( prev_tick.tv_usec - lasttime.tv_usec );

  if( !getrusage( RUSAGE_SELF, &r ) ) {
    curr = 1000000 * r.ru_utime.tv_sec + r.ru_utime.tv_usec;
    if( interval > 0 ) {
      logger_printf( "Usage: %2.2lf%% CPU", 100 * (curr - prev) / interval );
    }
    prev = curr;
    lasttime = prev_tick;
  }
}


static robot_state *alloc_state( uint16_t id ) {
  robot_state *rs = malloc( sizeof( robot_state ) );
  
  if( !rs ) {
    byebye( "make_robot_state()" );
  }

  memset( rs, 0, sizeof( robot_state ) );
  rs->next_event = -1;
  rs->next = state_list;
  state_list = rs;
  state_tab[id] = rs;

  return rs;
}


void robot_init( int add_bot ) {
  sim_init();

  if( add_bot ) { /* add a robot for sensory dep tank mode */
    robot_add( 1, model_list->typ );
  }
}


robot_state * robot_add( uint16_t id, uint16_t typ ) {
  robot_state *rs;
  robot_model *m = robot_get_model( typ );

  if( !m ) { 
    return NULL; /* no such model available */
  }

  rs = state_tab[id];
  if( !rs ) {
    rs = alloc_state( id );
  }

  m->init_state( rs );
  rs->model = m;

  rs->vmState.nodeId = id;
  buffer_add_reader( &rs->vmState );
  AsebaVMInit( &rs->vmState );
  m->init_aseba_vars( rs );
  AsebaVMSetupEvent( &rs->vmState, ASEBA_EVENT_INIT );
  return rs;
}


void robot_step( void ) {
  robot_state *rs;

  for( rs = state_list; rs; rs = rs->next ) {
    if( WAITING_EVENTS( rs ) &&
        !( AsebaMaskIsSet( rs->vmState.flags, ASEBA_VM_STEP_BY_STEP_MASK ) &&  
           AsebaMaskIsSet( rs->vmState.flags, ASEBA_VM_EVENT_ACTIVE_MASK ) ) ) {
      rs->model->set_source( rs, rs->vmState.nodeId );
      AsebaVMSetupEvent( &rs->vmState, next_local_event( rs ) );
    } else if( rs->incoming_events ) {
      AsebaProcessIncomingEvents( &rs->vmState );
      rs->incoming_events = buffer_remaining( &rs->vmState );
    }

    rs->model->read_aseba_vars( rs );
    AsebaVMRun( &rs->vmState, 1000 );
    rs->model->write_aseba_vars( rs );
  }
}


void robot_incoming_event( void ) {
  robot_state *rs;

  buffer_load();
  for( rs = state_list; rs; rs = rs->next ) {
    rs->incoming_events = buffer_remaining( &rs->vmState );
  }
}


int robot_event_pending( struct timeval *tv ) {
  static unsigned long diff;
  robot_state *rs;
  int step = 1;

  for( rs = state_list; rs; rs = rs->next ) {
    step = step && AsebaMaskIsSet(rs->vmState.flags,ASEBA_VM_STEP_BY_STEP_MASK);
  }
  
  if( step ) {
    /* Disable ALL local events if in STEP mode. */
    tv->tv_sec = 0;
    tv->tv_usec = 500000;
    sim_step_begin();
    sim_step_end();
    return 0;
  }

  if( !prev_tick.tv_sec && !prev_tick.tv_usec ) {
    gettimeofday( &prev_tick, NULL );
    return 1;
  } else if( diff < TICK ) {
    gettimeofday( tv, NULL );
    diff = (tv->tv_sec - prev_tick.tv_sec) * 1000000 + 
           (tv->tv_usec - prev_tick.tv_usec);
    if( diff > 100000 ) { /* avoid initial jumpiness (assume diff << 0.1 sec) */
      diff = 0;
      prev_tick = *tv;
      return 1;
    } else if( diff >= TICK ) {
      prev_tick = *tv;
    }
  }

  if( diff >= TICK ) {
    sim_step_begin();
    for( rs = state_list; rs; rs = rs->next ) {
      if( !AsebaMaskIsSet( rs->vmState.flags, ASEBA_VM_STEP_BY_STEP_MASK ) ) {
        /* don't rely on system time, use TICK, not diff */
        rs->model->do_step( rs, TICK ); 
      }
    }

    if( (performance > 0) && !(rs->tick % (performance * TICKS_PER_SEC)) ) {
      // rusage 1Hz
      rusage();
    }

    diff -= TICK;
    sim_step_end();
    return 1;
  } else {
    for( rs = state_list; rs; rs = rs->next ) {
      if( WAITING_EVENTS( rs ) || rs->incoming_events ) {
        return 1;
      }
    }
  }

  tv->tv_sec = 0;
  tv->tv_usec = TICK - diff;
   
  return 0;
}


void robot_disable_all() {
  robot_state *rs;

  for( rs = state_list; rs; rs = rs->next ) {
    rs->model->delete( rs );
  }
}
