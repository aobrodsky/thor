#include <math.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "../../vm/vm.h"

#include "world.h"
#include "robot.h"
#include "simcom.h"
#include "bot_err.h"
#include "objects.h"
#include "logger.h"
#include "normal.h"
#include "global.h"
#include "sim.h"

#define OBJS_ARRAY_INC 15

static struct simcom_env_new env;
static tw_obj **objects;

int16_t client_prot_ver = THOR_PROTOCOL_VERSION; /* modified in simcom.c */

static int build_path( struct simcom_env_obj *o ) {
  tw_vect *v;
  int n = 0;
  int i, j, k, p = 0;
  int dx, dy;
  double dir, c, s;
  struct simcom_env_poly *polys = o->polys;
  static tw_vect seg[4] = { { 0, PATH_WIDTH }, { 0, -PATH_WIDTH },
                            { 0, -PATH_WIDTH }, { 0, PATH_WIDTH } };

  for( i = 0; i < o->num; i++ ) {
    polys[i].num--; /* number of segments in a path */
    n += polys[i].num;
  }

  o->polys = malloc( sizeof( struct simcom_env_poly ) * n );
  if( !o->polys ) {
    return 0;
  }

  for( i = 0; i < o->num; i++ ) {
    v = polys[i].verts;
    for( j = 0; j < polys[i].num; j++ ) {
      o->polys[p].num = 4;
      o->polys[p].verts = malloc( sizeof( tw_vect ) * 4 );
      if( !o->polys[p].verts ) {
        return 0;
      }

      dx = v[j + 1].x - v[j].x;
      dy = v[j + 1].y - v[j].y;
      dir = atan2( dy, dx );
      c = cos( -dir );
      s = sin( -dir );
      seg[0].x = (int)sqrt( dx * dx + dy * dy ) + PATH_WIDTH;
      seg[1].x = seg[0].x;

      for( k = 0; k < 4; k++ ) {
        o->polys[p].verts[k].x = c * seg[k].x + s * seg[k].y + v[j].x;
        o->polys[p].verts[k].y = -s * seg[k].x + c * seg[k].y + v[j].y;
      }
      p++;
    }
    free( polys->verts );
  }
  free( polys );

  o->num = n;

  return 1;
}


static void add_robot( int16_t id, uint16_t typ, tw_angle dir, tw_vect *pos ) {
  robot_state *rs = robot_add( id, typ );

  if( rs ) {
    rs->model->set_position( rs, pos, dir );
    simcom_send_state( rs );
  } else {
    simcom_send_error( ROBOT_ERROR_INVALID_MODEL );
  }
}


static tw_obj *build_object( struct simcom_env_obj *o ) {
  int j;
  int typ; 
  tw_obj *two = NULL;

  if( o->typ == OBJ_TYP_PATH ) {
    build_path( o );
  }

  for( j = 0; j < o->num; j++ ) {
    if( ( o->typ == OBJ_TYP_BLOCK ) || ( o->typ == OBJ_TYP_POLYGON_BLOCK ) ) {
      typ = THOR_WORLD_OBJECT;
    } else if( (o->typ == OBJ_TYP_PATH) || (o->typ == OBJ_TYP_POLYGON_MARK) ) {
      typ = THOR_WORLD_MARK;
    } else {
      typ = o->clr <= 0 ? THOR_WORLD_OBJECT : THOR_WORLD_MARK;
    }

    if( o->clr < 0 ) { /* from old protocol */
      o->clr = -o->clr;
    }
    two = thor_world_make_object( typ, o->clr, o->polys[j].num,  &o->org, 
                                  o->dir, &o->pos, o->polys[j].verts, two );
  }
  return two;
}
 

static void free_env_obj( struct simcom_env_obj *o ) {
  int j;

  if( o->polys ) {
    for( j = 0; j < o->num; j++ ) {
      free( o->polys[j].verts );
    }
    free( o->polys );
  }
}


static int alloc_objects() {
  static int objects_size;
  tw_obj **o;
  int num;

  if( env.num > objects_size ) {
    num = env.num + OBJS_ARRAY_INC;
    o = realloc( objects, num * sizeof( tw_obj * ) );

    if( o ) {
      memset( &o[objects_size], 0, ( num - objects_size ) * sizeof(tw_obj *) );
      objects = o;
      objects_size = num;
    } else {
      simcom_send_error( ROBOT_ERROR_ALLOC_FAILED );
      logger_printf( "realloc failed allocating environment: %d", num );
    }
  }
  return env.num <= objects_size;
}


static int remove_object( int16_t id ) {
  int i;

  for( i = 0; i < env.num; i++ ) {
    if( env.objs[i].id == id ) {
      break;
    }
  }
    
  if( objects && ( i < env.num ) ) { 
    free_env_obj( &env.objs[i] );
    thor_world_free_object( objects[i] );
  } 
  return i;
}


static void free_env( struct simcom_env_new *e ) {
  int i;

  if( e->objs ) {  
    for( i = 0; i < e->num; i++ ) {
      free_env_obj( &e->objs[i] );
    }
    free( e->objs );
  } 

  if( e->robots ) {  
    free( e->robots );
  }
}
 
static void process_env_msg( struct simcom_env_msg *msg ) {
  static int objs_size;
  int i;
  int rc = 1;
  struct simcom_env_obj *o;
  robot_state *rs;

  if( ISSET_FLAG( GLOBAL_LOG_MORE ) ) {
    logger_printf( "Received packet of type: %d", msg->typ );
  }

  switch( msg->typ ) {
  case MSG_TYP_ENV:
    free_env( &env );
    env = msg->data.new_env;
    objs_size = env.num;

    robot_disable_all();
    thor_world_init( env.length, env.width );

    if( ( env.num > 0 ) && !alloc_objects() ) { 
      return;
    }

    for( i = 0; i < env.num; i++ ) {
      objects[i] = build_object( &env.objs[i] );
    
      /* invalid objects are NULLed and are not added */
      if( objects[i] ) {
        rc = thor_world_add_object( objects[i] ) && rc;
      }
    }

    for( i = 0; i < env.num_bots; i++ ) {
      add_robot( env.robots[i].id, env.robots[i].typ, env.robots[i].dir, 
                 &env.robots[i].pos );
    }

    break;
  case MSG_TYP_MOD_ARENA:
    free_env( &msg->data.new_env );
    thor_world_resize(msg->data.new_env.length, msg->data.new_env.width );
    break;
  case MSG_TYP_MOD_OBJ:
    i = remove_object( msg->data.env_obj.id );
    if( i == env.num ) {
      env.num++;
      if( objs_size < env.num ) {
        objs_size = env.num + OBJS_ARRAY_INC;
        o = realloc( env.objs, objs_size * sizeof( struct simcom_env_obj ) );
        if( o ) {
          env.objs = o;
        } else {
          logger_printf( "realloc failed allocating environment: %d", env.num );
          env.num--;
          break;
        }
      }

      if( !alloc_objects() ) {
        env.num--;
        break;
      }
    }      

    if( objects ) { 
      env.objs[i] = msg->data.env_obj;
      objects[i] = build_object( &env.objs[i] );
      thor_world_add_object( objects[i] );
    }
    break;
  case MSG_TYP_DEL_OBJ:
    i = remove_object( msg->data.env_obj.id );
    if( i < env.num ) {
      env.num--;
      if( i < env.num ) {
        env.objs[i] = env.objs[env.num];
        objects[i] = objects[env.num];
      }
    }
    /* avoid leaking memory if client is malicious */
    free_env_obj( &msg->data.env_obj ); 
    break;
  case MSG_TYP_DEL_BOT:
    rs = robot_get_state( msg->data.mov_obj.id );
    if( rs ) {
      rs->model->delete( rs );
    }
    break;
  case MSG_TYP_MOD_BOT:
    i = msg->data.mov_obj.id;
    rs = robot_get_state( i );
    if( !rs || !rs->enabled ) {
      add_robot( i, msg->data.mov_obj.typ, msg->data.mov_obj.dir, 
                   &msg->data.mov_obj.pos );
    } else if( rs->model->set_position( rs, &msg->data.mov_obj.pos, 
                                             msg->data.mov_obj.dir ) ) {
      simcom_send_state( rs );
    }
    break;
  case MSG_TYP_INP:
    rs = robot_get_state( msg->id );
    if( rs ) {
      rs->model->user_action( rs, msg->data.inputs );
    } else {
      logger_printf( "Invalid robot id %d received", msg->id );
    }
    break;
  }
}


void sim_step_end() {
  simcom_flush();
}


void sim_step_begin() {
  struct simcom_env_msg msg;

  memset( &msg, 0, sizeof( struct simcom_env_msg ) );
  if( simcom_recv_env_msg( &msg ) ) {
    process_env_msg( &msg );
  }
}

void sim_init() {
  simcom_init();
}
