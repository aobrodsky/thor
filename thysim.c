#include <math.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "../../vm/vm.h"

#include "thymio.h"
#include "thysim.h"
#include "world.h"
#include "thystate.h"
#include "simcom.h"
#include "msg_com.h"
#include "logger.h"
#include "normal.h"

typedef struct thymio_inputs {
  int16_t id;
  int16_t buttons[NUM_BUTTONS];
  int16_t tap;
  tw_angle noise[NUM_NOISE];
  int16_t sound;
  int16_t temp;
} thymio_inputs; 

static struct msg_com_fmt thymio_input_fmt[] = {
  { MSG_FIELD_SINT16, NULL, "id", offsetof( thymio_inputs, id ),
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "buttons", offsetof( thymio_inputs, buttons ),
        sizeof( int16_t ), NUM_BUTTONS },
  { MSG_FIELD_SINT16, NULL, "tap", offsetof( thymio_inputs, tap ),
        sizeof( int16_t ), 1 },
  { MSG_FIELD_ANGLE, NULL, "noise", offsetof( thymio_inputs, noise ),
        sizeof( tw_angle ), NUM_NOISE },
  { MSG_FIELD_SINT16, NULL, "sound", offsetof( thymio_inputs, sound ),
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "temp", offsetof( thymio_inputs, temp ),
        sizeof( int16_t ), 1 },
  { 0, 0, NULL, 0, 0, 0 } };

/* Thymio Physical Measurements, relative to center hole (0,0) */
/* All measurements in mm and degress */
/* Body */
static tw_vect back_left = { -55, -30 };
static tw_vect front_left = { -55, 55 };
static tw_vect back_right = { 55, -30 };
static tw_vect front_right = { 55, 55 };
static tw_coord front_arc_rad = 80;
static tw_coord front_arc_start = -45;
static tw_coord front_arc_end = 45;

/* prox sensors */
static double prox_ang[NUM_PROX_HORIZ] = { RAD(-40), RAD(-20), RAD(0), 
                                             RAD(20), RAD(40), 
                                             RAD(180), RAD(180) };
static tw_coord front_prox_rad = 80;
static double bot_prox_ang[NUM_PROX_GRND] = {RAD(-7), RAD(7) };
static tw_coord bot_prox_rad = 75;
static tw_vect  rear_left_prox = { -30, -30 };
static tw_vect  rear_right_prox = { 30, -30 };

/* wheels  and front point of contact */
static tw_coord wheel_diameter = 94;
static tw_vect left_wheel_pos = { -47, 0 }; 
static tw_vect right_wheel_pos = { 47, 0 };
static tw_vect front_pivot_pos = { 0, 60 };

/* thymio world representation */
static tw_vect simcom_vertices[NUM_THY_VERTICES];
static tw_vect simcom_contacts[NUM_THY_CONTACTS];
static tw_vect simcom_origin = {0, 0};
static tw_vect prox_horiz[NUM_PROX_HORIZ];
static tw_vect prox_grnd[NUM_PROX_GRND];

/* Obstacle colour/max distance table 
 * Colours are grey scale [0 - 1024]
 * Response is in mm in range [0 - 120], normally [25 - 90] */
unsigned char prox_max_dist[1024];

/* Obstacle colour / C calibration (response factor)
 * Colours are grey scale [0 - 1024]
 * C is in range [0 - 5000], normally [200 - 2500] */
unsigned int prox_c_tab[1024];

/* Obstacle colour / M maximum respnse (response factor)
 * Colours are grey scale [0 - 1024]
 * M is in range [0 - 5000], normally [4000 - 5000] */
unsigned int prox_m_tab[1024];

enum {
  LEDS_PROX_H,
  LEDS_PROX_G,
  LEDS_TEMP,
  LEDS_SOUND,
  LEDS_BUTTONS,
  LEDS_LAST,
};

#define DISABLE_AUTO(ps, c) (ps)->auto_behaviour |= 1 << (c);
#define AUTO_BEHAVIOUR(ps, c) (!( (ps)->auto_behaviour & (1 << (c))))

static struct msg_com_fmt thymio_state_fmt[] = {
  { MSG_FIELD_SINT16, NULL, "ver", offsetof( struct thymio_state, ver ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "id", offsetof( struct thymio_state, id ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "error", offsetof( struct thymio_state, error ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "model", offsetof( struct thymio_state, model ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "target", offsetof( struct thymio_state, target ), 
        sizeof( int16_t ), NUM_WHEELS },
  { MSG_FIELD_SINT16, NULL, "speed", offsetof( struct thymio_state, speed ), 
        sizeof( int16_t ), NUM_WHEELS },
  { MSG_FIELD_VECT, NULL, "position", offsetof( struct thymio_state, position ),
        sizeof( tw_vect ), 1 },
  { MSG_FIELD_ANGLE, NULL, "direction", 
        offsetof( struct thymio_state, direction ), sizeof( tw_angle ), 1 },
  { MSG_FIELD_SINT16, NULL, "buttons", 
        offsetof( struct thymio_state, button_state ), sizeof( int16_t ), 
        NUM_BUTTONS},
  { MSG_FIELD_SINT16, NULL, "tap", offsetof( struct thymio_state, tap ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "leds_circle", offsetof( struct thymio_state, 
        circle_leds ), sizeof( int16_t ),  NUM_CIRCLE_LEDS },
  { MSG_FIELD_SINT16, NULL, "leds_top", 
        offsetof( struct thymio_state, leds_top ), sizeof( int16_t ), 3 },
  { MSG_FIELD_SINT16, NULL, "leds_bot_left", 
        offsetof( struct thymio_state, leds_bot_left ), sizeof( int16_t ), 3 },
  { MSG_FIELD_SINT16, NULL, "leds_bot_right", 
        offsetof( struct thymio_state, leds_bot_right ), sizeof( int16_t ), 3 },
  { MSG_FIELD_SINT16, NULL, "leds_prox", 
        offsetof( struct thymio_state, leds_prox), sizeof( int16_t ), 
        NUM_PROX_LEDS},
  { MSG_FIELD_SINT16, NULL, "leds_prox_grnd", 
        offsetof( struct thymio_state, leds_prox_grnd ), sizeof( int16_t ), 2 },
  { MSG_FIELD_SINT16, NULL, "leds_buttons", 
        offsetof( struct thymio_state, leds_buttons ), sizeof( int16_t ), 4 },
  { MSG_FIELD_SINT16, NULL, "leds_rc", offsetof( struct thymio_state, leds_rc ),
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "leds_temp", 
        offsetof( struct thymio_state, leds_temp ), sizeof( int16_t ), 2 },
  { MSG_FIELD_SINT16, NULL, "leds_sound", 
        offsetof( struct thymio_state, leds_sound ), sizeof( int16_t ), 1 },
  { 0, NULL, NULL, 0, 0, 0 } };

typedef struct physical_state {
  struct thymio_state           state;
  tw_obj                       *robot;
  tw_obj                       *ground_sensors[NUM_PROX_GRND];
  long                          auto_behaviour;
  double                        x_pos;
  double                        y_pos;
  double                        dir;
  int                           deg;
  int                           prev_tap;
  int                           heartbeat;
  struct thymio_state           prev_state;
} physical_state;

#define MIN_UPD_DIR ( M_PI / 360 ) /* half a degree */


static physical_state *alloc_physical_state( robot_state *rs ) {
  physical_state *ps;

  if( !rs->physical_state ) {
    rs->physical_state = malloc( sizeof( physical_state ) );
  }
  ps = rs->physical_state;

  if( !ps) {
    byebye( "make_physical_state()" );
  }

  memset( ps, 0, sizeof( physical_state ) );
  ps->state.id = rs->vmState.nodeId;
  ps->state.ver = THOR_PROTOCOL_VERSION;
  ps->state.model = ROBOT_TYP_THYMIO;

  return ps;
}


static void update_position( physical_state *ps ) {
  if( fabs( ps->x_pos - ps->state.position.x ) >= 1 ) {
    ps->x_pos = ps->state.position.x;
  }
  if( fabs( ps->y_pos - ps->state.position.y ) >= 1 ) {
    ps->y_pos = ps->state.position.y;
  }
  ps->dir = ps->state.direction;
}


void thy_sim_init_state( robot_state *rs ) {
  int rc = 1;
  tw_vect *pos = &front_right; /* we may want to change these later */
  tw_angle dir = 0;

  physical_state *ps = alloc_physical_state( rs );
  ps->ground_sensors[0] = thor_world_make_object( THOR_WORLD_POINT, -1, 1, 
                                                  &simcom_origin, dir, pos, 
                                                  &prox_grnd[0], NULL );
  rc = rc && ps->ground_sensors[0];

  ps->ground_sensors[1] = thor_world_make_object( THOR_WORLD_POINT, -1, 1, 
                                                  &simcom_origin, dir, pos, 
                                                  &prox_grnd[1], 
                                                  ps->ground_sensors[0] );
  rc = rc && ps->ground_sensors[1];

  ps->robot = thor_world_make_object( THOR_WORLD_ROBOT | THOR_WORLD_IN_BOUNDS,
                                   -1, NUM_THY_CONTACTS, &simcom_origin, dir, 
                                   pos, simcom_contacts, ps->ground_sensors[1]);
  rc = rc && ps->robot;

  ps->robot = thor_world_make_object( THOR_WORLD_ROBOT, ROBOT_COLOUR, 
                                     NUM_THY_VERTICES, &simcom_origin, dir, pos,
                                     simcom_vertices, ps->robot );
    
  rc = rc && ps->robot && thor_world_add_object( ps->robot );

  if( !rc ) {
    ps->state.error = ROBOT_ERROR_OBJ_OUT_OF_BOUNDS;
  }
  rs->physical_state = ps;
  rs->send_state = &ps->state;
  rs->prev_state = &ps->prev_state;
}


int thy_sim_set_position( robot_state *rs, tw_vect *pos, tw_angle dir ) {
  physical_state *ps = rs->physical_state;
  int rc = thor_world_move_object( ps->robot, pos, dir );

  if( rc ) {
    ps->state.position = *pos;
    ps->state.direction = dir;
    update_position( ps );
  } else { 
    thor_world_move_object(ps->robot, &ps->state.position, ps->state.direction);
  }

  return rc;
}


void thy_sim_delete( robot_state *rs ) {
  physical_state *ps = rs->physical_state;
  thor_world_free_object( ps->robot );
  ps->robot = NULL; /* for safety */
}


void thy_sim_user_action( robot_state *rs, void *input ) {
  int i;
  thymio_inputs *inp = input;
  physical_state *ps = rs->physical_state;

  for( i = 0; i < NUM_BUTTONS; i++ ) {
    ps->state.button_state[i] = inp->buttons[i];
  }

  ps->state.tap = inp->tap;

  for( i = 0; i < NUM_NOISE; i++ ) {
    ps->state.noise[i] = inp->noise[i];
  }

  if( AUTO_BEHAVIOUR( ps, LEDS_BUTTONS ) ) {
    i = ps->state.button_state[2];
    ps->state.leds_buttons[0] = ( i || ps->state.button_state[3] ) * 32;
    ps->state.leds_buttons[1] = ( i || ps->state.button_state[4] ) * 32;
    ps->state.leds_buttons[2] = ( i || ps->state.button_state[0] ) * 32;
    ps->state.leds_buttons[3] = ( i || ps->state.button_state[1] ) * 32;
    simcom_send_state( rs );
  }
}


static int16_t get_speed( int16_t targ, double var, double bias ) { 
  double speed = targ;

  if( targ != 0 ) {
    if( targ > 500 ) { /* limit |speed| <= 500 */
      targ = 500;
    } else if( targ < -500 ) {
      targ = -500;
    }

    if( var > 0 ) {
      speed = next_gaussian( targ, var * abs( targ ) );
    } else {
      speed = targ;
    }

    speed += bias * targ;

    if( speed * targ < 0 ) {
      speed = 0;
    }
  }
  return speed;
}


void thy_sim_init( robot_model *m ) {
  int i = 0;
  tw_coord d;
  double a;
  double r;

  simcom_vertices[i++] = front_right;
  simcom_vertices[i++] = back_right;
  simcom_vertices[i++] = back_left;
  simcom_vertices[i++] = front_left;

  for(d = front_arc_start + ARC_REP_GRAN; d < front_arc_end; d += ARC_REP_GRAN){
    simcom_vertices[i].x = (tw_coord) ( front_arc_rad * sin( RAD( d ) ) );
    simcom_vertices[i++].y = (tw_coord) ( front_arc_rad * cos( RAD( d ) ) );
  }

  i = 0;	
  simcom_contacts[i++] = right_wheel_pos;
  simcom_contacts[i++] = left_wheel_pos;
  simcom_contacts[i++] = front_pivot_pos;

  for( i = 0; i < 5; i++ ) {
    prox_horiz[i].x = (tw_coord) ( front_prox_rad * sin( prox_ang[i] ) );
    prox_horiz[i].y = (tw_coord) ( front_prox_rad * cos( prox_ang[i] ) );
  }

  prox_horiz[i++] = rear_right_prox;
  prox_horiz[i++] = rear_left_prox;

  for( i = 0; i < 2; i++ ) {
    prox_grnd[i].x = (tw_coord) ( bot_prox_rad * sin( bot_prox_ang[i] ) );
    prox_grnd[i].y = (tw_coord) ( bot_prox_rad * cos( bot_prox_ang[i] ) );
  }

  a = log( 210 ) / ( 200 * 4);
  for( i = 40; i <= 840; i++ ) {
    prox_max_dist[i] = 90;
    prox_max_dist[i] -= 60 * ( 210 / exp( a * ( i - 40 ) ) - 1 ) / 209;
    assert( prox_max_dist[i] <= MAX_PROX_DIST );

    /* R(g) = 4200 - (840/g - 1) * (40 / (840 - 40)) * (4200 - 1100) */
    /* C(R(g)) = 625 * R / (PROX_MAX_VAL - R) */
    r = 4355.0 - ( 130200.0 / i );
    prox_c_tab[i] = (int)( 625 * r / ( PROX_MAX_VAL - r ) );

    /* M(g) = 5000 - (840/g - 1) * (40 / (840 - 40)) * (5000 - 3900) */
    prox_m_tab[i] = 5055 - ( 840 * 55 / i );
  }

  for( i = 0; i < 40; i++ ) {
    prox_max_dist[i] = prox_max_dist[40];
    prox_c_tab[i] = prox_c_tab[40];
    prox_m_tab[i] = prox_m_tab[40];
  }

  for( i = 841; i < 1024; i++ ) {
    prox_max_dist[i] = prox_max_dist[840];
    prox_c_tab[i] = prox_c_tab[840];
    prox_m_tab[i] = prox_m_tab[840];
  }

  m->state_fmt = thymio_state_fmt;
  m->input_fmt = thymio_input_fmt;
}


int thy_sim_set_speed( robot_state *rs, int16_t *targ ) {
  physical_state *ps = rs->physical_state;

  if( memcmp( ps->state.target, targ, sizeof( int16_t ) * NUM_WHEELS ) ) {
    /* no noise or bias for now */
    ps->state.target[LEFT_WHEEL] = targ[LEFT_WHEEL];  
    ps->state.target[RIGHT_WHEEL] = targ[RIGHT_WHEEL];
    simcom_send_state( rs );
  }
  return 1;
}


int thy_sim_get_speed( robot_state *rs, int16_t *speed ) {
  physical_state *ps = rs->physical_state;

  /* noise and bias included in update_pos() */
  speed[LEFT_WHEEL] = ps->state.speed[LEFT_WHEEL]; 
  speed[RIGHT_WHEEL] = ps->state.speed[RIGHT_WHEEL];

  return 1;
}


int thy_sim_update_pos( robot_state *rs, unsigned long dt ) {
  physical_state *ps = rs->physical_state;
  static unsigned long local_heartbeat;
  double x_dt;
  double y_dt;
  tw_vect prev_pos;
  tw_angle prev_dir;
  int prev_deg;
  double ang_dt;
  double speed_dt;
  double left_dt;
  double right_dt;

  ps->heartbeat += dt;
  local_heartbeat += dt;
  if( local_heartbeat > ps->heartbeat ) {
    local_heartbeat = ps->heartbeat;
  }

  if( dt > 100000 ) { /* avoid initial jumpiness (assume dt << 1/10th sec) */
    return 0;
  }

  ps->state.speed[LEFT_WHEEL] = get_speed( ps->state.target[LEFT_WHEEL],
                                           ps->state.noise[LEFT_MOTOR_VAR ],
                                           ps->state.noise[LEFT_MOTOR_BIAS] );
  ps->state.speed[RIGHT_WHEEL] = get_speed( ps->state.target[RIGHT_WHEEL], 
                                            ps->state.noise[RIGHT_MOTOR_VAR ],
                                            ps->state.noise[RIGHT_MOTOR_BIAS] );
  left_dt = dt * SPEED_CONV * ps->state.speed[LEFT_WHEEL];
  right_dt = dt * SPEED_CONV * ps->state.speed[RIGHT_WHEEL];
  
  speed_dt = ( left_dt + right_dt ) / 2;
  ang_dt = ( left_dt - right_dt ) / wheel_diameter;

  x_dt = speed_dt * sin( ps->state.direction + ang_dt * 0.5 );
  y_dt = speed_dt * cos( ps->state.direction + ang_dt * 0.5 );
  ps->x_pos += x_dt;
  ps->y_pos += y_dt;
  ps->dir += ang_dt;

  while( ps->dir > ( 2 * M_PI ) ) {
    ps->dir -= 2 * M_PI;
  }

  while( ps->dir < ( -2 * M_PI ) ) {
    ps->dir += 2 * M_PI;
  }

  prev_pos = ps->state.position;
  prev_dir = ps->state.direction;
  prev_deg = ps->deg;
  ps->deg = (int) ( ps->dir * 360 / M_PI ); /* half -degree increments */

  ps->state.position.x = (tw_coord)ps->x_pos;
  ps->state.position.y = (tw_coord)ps->y_pos;
  if( ps->prev_tap == ps->state.tap ) {
    ps->state.tap = 0;
  }
  ps->prev_tap = ps->state.tap;

  if( ps->robot && ( ( prev_pos.x - ps->state.position.x ) || 
                 ( prev_pos.y - ps->state.position.y ) ||
                 ( prev_deg - ps->deg ) || ps->state.tap ) ) {
    ps->state.direction = ps->dir; /* only update dir if change is sufficient */

    if( thor_world_move_object( ps->robot, &ps->state.position, 
                                ps->state.direction ) ) {
      if( !thor_world_collision( ps->robot ) ) {
        if( ( ps->heartbeat > MIN_SEND_INTERVAL ) || ps->state.tap ) {
          ps->heartbeat = 0;
          simcom_send_state( rs );
        }
        return 1;
      }
    } 
    ps->state.tap = 1;
    if( ps->heartbeat > MIN_SEND_INTERVAL ) {
      ps->state.error = ROBOT_ERROR_OUT_OF_BOUNDS;
      ps->heartbeat = 0;
      simcom_send_state( rs );
      ps->state.error = ROBOT_ERROR_NONE;
    }

    ps->x_pos -= x_dt;
    ps->y_pos -= y_dt;
    ps->dir = prev_dir;
    ps->state.position = prev_pos;
    ps->state.direction = prev_dir;
  } else if( local_heartbeat >= HEARTBEAT_INTERVAL ) { 
    ps->heartbeat = 0;
    local_heartbeat = 0;
    simcom_send_heartbeat();
  }
  return 0;
}


int thy_sim_get_buttons( robot_state *rs, int16_t *buttons ) {
  physical_state *ps = rs->physical_state;

  memcpy( buttons, ps->state.button_state, sizeof( int16_t ) * NUM_BUTTONS );

  return 1;
}


static int16_t get_horiz_light( int16_t light, double var, double bias ) {
  if( var > 0 ) {
    light = next_gaussian( light, var );
  }
  light += bias;

  if( light < 0 ) {
    light = 0;
  } else if( light > PROX_MAX_VAL ) {
    light = PROX_MAX_VAL;
  }
  return light;
}


int thy_sim_get_prox_horiz( robot_state *rs, int16_t *horiz ) {
  physical_state *ps = rs->physical_state;

  int i, j;
  int d;
  double dist;
  double dir;
  tw_vect s;
  tw_vect t;
  tw_vect w;
  tw_obj *o;
  static int max_led = PROX_C_PARAM;
  static int ray_dir[5] = { -15, -8, 0, 8, 15};
  int d_max;
  int c;
  int m;

  if( !ps->robot ) {
    return 0;
  }

  for( i = 0; i < NUM_PROX_HORIZ; i++ ) {
    horiz[i] = 0;
    s = prox_horiz[i];
    thor_world_transform( ps->robot, &s );
    for( d = 0; d < 5; d++ ) {
      o = NULL;
      dir = ps->robot->dir + prox_ang[i] + RAD( ray_dir[d] );
      t.x = (tw_coord) ( (double)( MAX_PROX_DIST + 1 ) * sin( dir ) + s.x );
      t.y = (tw_coord) ( (double)( MAX_PROX_DIST + 1 ) * cos( dir ) + s.y );
      dist = thor_world_intersect( ps->robot, &s, &t, &w, &o );

      if( o && ( o->colour >= 0 ) && ( o->colour < 1024 ) ) { 
        /* maximum distance to object depends on its colour as well */
        d_max = prox_max_dist[o->colour];
        c = prox_c_tab[o->colour];
        m = prox_m_tab[o->colour];
      } else {
        d_max = MAX_PROX_DIST;
        c = PROX_C_PARAM;
        m = PROX_MAX_VAL;
      }

      // printf( "(%d.%d: %d %d %d %d %lf %d %d)\n", i, d, s.x, s.y, t.x, t.y, 
      //                                           dist, w.x, w.y );
      if( dist <= d_max ) {
       /* See formula in ENKI IRSensor.h, x0 = 0, c = 2500, m = 5000 */
        horiz[i] += PROX_VAL( dist, c, m );
      }
    }

    if( horiz[i] < 0 ) {
      horiz[i] = 0;
    } else {
      horiz[i] /= 5; /* average the 5 rays cast */
    }

    horiz[i] = get_horiz_light( horiz[i], ps->state.noise[HORIZ_SENSOR_VAR],
                                ps->state.noise[HORIZ_SENSOR_BIAS] );

    if( AUTO_BEHAVIOUR( ps, LEDS_PROX_H ) ) {
      if( max_led < horiz[i] ) {
        max_led = horiz[i];
      }
      j = i > 2 ? i + 1 : i;
      ps->state.leds_prox[j] = horiz[i] * 32 / max_led;
      if( j == 2 ) {
        ps->state.leds_prox[j + 1] = ps->state.leds_prox[j];
      }
    }
  }
  return 1;
}


static int16_t get_ground_light( int16_t light, double var, double bias ) {
  if( var > 0 ) {
    light = next_gaussian( light, var );
  }
  light += bias;

  if( light < 0 ) {
    light = 0;
  } else if( light > MAX_LIGHT ) {
    light = MAX_LIGHT;
  }
  return light;
}


int thy_sim_get_prox_grnd( robot_state *rs, int16_t *amb, int16_t *ref, 
                           int16_t *delta  ) {
  physical_state *ps = rs->physical_state;

  tw_obj *mark;
  int i;

  for( i = 0; i < NUM_PROX_GRND; i++ ) {
    amb[i] = get_ground_light( AMB_LIGHT, ps->state.noise[GROUND_SENSOR_VAR],
                               ps->state.noise[GROUND_SENSOR_BIAS] );
    if( ps->ground_sensors[i] ) {
      mark = thor_world_over_mark( ps->ground_sensors[i] );
      if( mark ) {
        ref[i] = mark->colour;
      } else {
        ref[i] = TABLE_LIGHT;
      }
    } else {
      ref[i] = 0;
    }
    ref[i] = get_ground_light( ref[i], ps->state.noise[GROUND_SENSOR_VAR],
                               ps->state.noise[GROUND_SENSOR_BIAS] );
    delta[i] = ref[i] - amb[i];
    if( delta[i] < 0 ) {
      delta[i] = 0;
    }

    if( AUTO_BEHAVIOUR( ps, LEDS_PROX_G ) ) {
      ps->state.leds_prox_grnd[i] = delta[i] * 32 / MAX_LIGHT;
    }
  }

  return 1;
}


int thy_sim_tap( robot_state *rs ) {
  physical_state *ps = rs->physical_state;

  return ps->state.tap;
}


int thy_sim_get_acc( robot_state *rs, int16_t *acc ) {
  acc[0] = 0;  /* X and Y accelleromaters are unaffected */
  acc[1] = 0;
  acc[2] = 23; /* 1g */
  return 1;
}


int thy_sim_get_temp( robot_state *rs, int16_t *temp ) {
  *temp = 200; /* it's always a balmy 20'C */
  return 1;
}



int thy_sim_set_leds_circle( AsebaVMState *vm, int16_t *leds ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  physical_state *ps = rs->physical_state;

  memcpy( ps->state.circle_leds, leds, sizeof( int16_t ) * NUM_CIRCLE_LEDS );
  simcom_send_state( rs );
  return 1;
}


int thy_sim_set_leds_top( AsebaVMState *vm, int16_t red, int16_t green, 
                          int16_t blue ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  physical_state *ps = rs->physical_state;

  ps->state.leds_top[0] = red;
  ps->state.leds_top[1] = green;
  ps->state.leds_top[2] = blue;
  simcom_send_state( rs );
  return 1;
}


int thy_sim_set_leds_bot_left( AsebaVMState *vm, int16_t red, int16_t green, 
                               int16_t blue ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  physical_state *ps = rs->physical_state;

  ps->state.leds_bot_left[0] = red;
  ps->state.leds_bot_left[1] = green;
  ps->state.leds_bot_left[2] = blue;
  simcom_send_state( rs );
  return 1;
}


int thy_sim_set_leds_bot_right( AsebaVMState *vm, int16_t red, int16_t green, 
                                int16_t blue ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  physical_state *ps = rs->physical_state;

  ps->state.leds_bot_right[0] = red;
  ps->state.leds_bot_right[1] = green;
  ps->state.leds_bot_right[2] = blue;
  simcom_send_state( rs );
  return 1;
}


int thy_sim_set_leds_prox_horiz( AsebaVMState *vm, int16_t *leds ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  physical_state *ps = rs->physical_state;

  DISABLE_AUTO( ps, LEDS_PROX_H ); 
  memcpy( ps->state.leds_prox, leds, sizeof( int16_t ) * NUM_PROX_LEDS );
  simcom_send_state( rs );
  return 1;
}


int thy_sim_set_leds_prox_grnd( AsebaVMState *vm, int16_t left, int16_t right ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  physical_state *ps = rs->physical_state;

  DISABLE_AUTO( ps, LEDS_PROX_G ); 
  ps->state.leds_prox_grnd[0] = left;
  ps->state.leds_prox_grnd[1] = right;
  simcom_send_state( rs );
  return 1;
}


int thy_sim_set_leds_buttons( AsebaVMState *vm, int16_t top, int16_t right, 
                              int16_t back, int16_t left ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  physical_state *ps = rs->physical_state;

  DISABLE_AUTO( ps, LEDS_BUTTONS ); 
  ps->state.leds_buttons[0] = top;
  ps->state.leds_buttons[1] = right;
  ps->state.leds_buttons[2] = back;
  ps->state.leds_buttons[3] = left;
  simcom_send_state( rs );
  return 1;
}


int thy_sim_set_leds_rc( AsebaVMState *vm, int16_t led ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  physical_state *ps = rs->physical_state;

  ps->state.leds_rc = led;
  simcom_send_state( rs );
  return 1;
}


int thy_sim_set_leds_temp( AsebaVMState *vm, int16_t red, int16_t blue ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  physical_state *ps = rs->physical_state;

  DISABLE_AUTO( ps, LEDS_TEMP ); 
  ps->state.leds_temp[0] = red;
  ps->state.leds_temp[1] = blue;
  simcom_send_state( rs );
  return 1;
}


int thy_sim_set_leds_sound( AsebaVMState *vm, int16_t led ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  physical_state *ps = rs->physical_state;

  DISABLE_AUTO( ps, LEDS_SOUND ); 
  ps->state.leds_sound = led;
  simcom_send_state( rs );
  return 1;
}


