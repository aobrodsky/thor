#include <stddef.h>
#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "thor.h"
#include "robot.h"
#include "sim.h"
#include "world.h"
#include "simcom.h"
#include "bot_err.h"
#include "msg_com.h"


struct base_state {
  int16_t ver;
  int16_t id;
  int16_t error;
  int16_t model;
};


static struct msg_com_fmt id_fmt[] = {
  { MSG_FIELD_SINT16, NULL, "id", offsetof( struct base_state, id ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "error", offsetof( struct base_state, error ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "model", offsetof( struct base_state, model ), 
        sizeof( int16_t ), 1 },
  { 0, NULL, NULL, 0, 0, 0 } };

static struct msg_com_fmt err_fmt[] = {
  { MSG_FIELD_SINT16, NULL, "ver", offsetof( struct base_state, ver ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "id", offsetof( struct base_state, id ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "error", offsetof( struct base_state, error ), 
        sizeof( int16_t ), 1 },
  { 0, NULL, NULL, 0, 0, 0 } };

static struct msg_com_fmt poly_fmt[] = {
  { MSG_FIELD_UINT32, NULL, "num", offsetof( struct simcom_env_poly, num ), 
        sizeof( uint32_t ), 1 },
  { MSG_FIELD_VECT, NULL, "verts", offsetof( struct simcom_env_poly, verts ),
        sizeof( tw_vect ), -1 },
  { 0, NULL, NULL, 0, 0, 0 } };

static struct msg_com_fmt obj_fmt[] = {
  { MSG_FIELD_SINT16, NULL, "id", offsetof( struct simcom_env_obj, id ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "typ", offsetof( struct simcom_env_obj, typ ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "clr", offsetof( struct simcom_env_obj, clr ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_VECT, NULL, "org", offsetof( struct simcom_env_obj, org ), 
        sizeof( tw_vect ), 1 },
  { MSG_FIELD_VECT, NULL, "pos", offsetof( struct simcom_env_obj, pos ), 
        sizeof( tw_vect ), 1 },
  { MSG_FIELD_ANGLE, NULL, "dir", offsetof( struct simcom_env_obj, dir ),
        sizeof( tw_angle ), 1 },
  { MSG_FIELD_UINT32, NULL, "num", offsetof( struct simcom_env_obj, num ), 
        sizeof( uint32_t ), 1 },
  { MSG_FIELD_STRUCT, poly_fmt, "polys", 
        offsetof( struct simcom_env_obj, polys ),
        sizeof( struct simcom_env_poly ), -1 },
  { 0, NULL, NULL, 0, 0, 0 } };

static struct msg_com_fmt msg_fmt[] = {
  { MSG_FIELD_SINT16, NULL, "typ", offsetof( struct simcom_env_msg, typ ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "ver", offsetof( struct simcom_env_msg, ver ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "id", offsetof( struct simcom_env_msg, id ), 
        sizeof( int16_t ), 1 },
  { 0, NULL, NULL, 0, 0, 0 } };

static struct msg_com_fmt cfg_fmt[] = {
  { MSG_FIELD_SINT16, NULL, "id", offsetof( struct simcom_env_cfg, id ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_VECT, NULL, "pos", offsetof( struct simcom_env_cfg, pos ), 
        sizeof( tw_vect ), 1 },
  { MSG_FIELD_ANGLE, NULL, "dir", offsetof( struct simcom_env_cfg, dir),
        sizeof( tw_angle ), 1 },
  { MSG_FIELD_SINT16, NULL, "typ", offsetof( struct simcom_env_cfg, typ ), 
        sizeof( int16_t ), 1 },
  { 0, NULL, NULL, 0, 0, 0 } };

static struct msg_com_fmt new_fmt[] = {
  { MSG_FIELD_COORD, NULL, "length", offsetof( struct simcom_env_new, length ), 
        sizeof( tw_coord ), 1 },
  { MSG_FIELD_COORD, NULL, "width", offsetof( struct simcom_env_new, width ), 
        sizeof( tw_coord ), 1 },
  { MSG_FIELD_UINT32, NULL, "num", offsetof( struct simcom_env_new, num ),
        sizeof( uint32_t ), 1 },
  { MSG_FIELD_STRUCT, obj_fmt, "objs", offsetof( struct simcom_env_new, objs ), 
        sizeof( struct simcom_env_obj ), -1 },
  { MSG_FIELD_UINT32, NULL, "num_bots", 
        offsetof( struct simcom_env_new, num_bots ), sizeof( uint32_t ), 1 },
  { MSG_FIELD_STRUCT, cfg_fmt, "robots", 
        offsetof( struct simcom_env_new, robots ), 
        sizeof( struct simcom_env_cfg ), -1 },
  { 0, NULL, NULL, 0, 0, 0 } };

static struct msg_com_fmt hello_fmt[] = {
  { MSG_FIELD_SINT16, NULL, "ver", offsetof( struct simcom_hello, ver ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "id", offsetof( struct simcom_hello, id ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "bots", offsetof( struct simcom_hello, bots ), 
        sizeof( int16_t ), 1 },
  { MSG_FIELD_UINT32, NULL, "num_models", offsetof( struct simcom_hello, 
        num_models ), sizeof( uint16_t ), 1 },
  { MSG_FIELD_SINT16, NULL, "models", offsetof( struct simcom_hello, models ),
        sizeof( int16_t ), -1 },
  { 0, NULL, NULL, 0, 0, 0 } };

static char buffer[65536]; /* buffer for inputs, makes code nonrentrant */

int simcom_recv_env_msg( struct simcom_env_msg *msg ) {
  static int msg_typ = MSG_TYP_NONE;
  robot_state *rs;
  int r = 0;

  if( msg_typ == MSG_TYP_NONE ) {
    if( msg_com_recv( msg, msg_fmt ) > 0 ) {
      msg_typ = msg->typ;
    } else {
      return 0;
    }
  }

  switch( msg_typ ) {
  case MSG_TYP_ENV:
    r = msg_com_recv( &msg->data.new_env, new_fmt );
    break;
  case MSG_TYP_MOD_ARENA:
    r = msg_com_recv( &msg->data.new_env, new_fmt );
    break;
  case MSG_TYP_MOD_OBJ:
    r = msg_com_recv( &msg->data.env_obj, obj_fmt );
    break;
  case MSG_TYP_MOD_BOT:
    r = msg_com_recv( &msg->data.mov_obj, cfg_fmt );
    break;
  case MSG_TYP_DEL_OBJ:
    r = msg_com_recv( &msg->data.env_obj, obj_fmt );
    break;
  case MSG_TYP_DEL_BOT:
    r = msg_com_recv( &msg->data.mov_obj, cfg_fmt );
    break;
  case MSG_TYP_INP:
    rs = robot_get_state( msg->id );
    msg->data.inputs = buffer;
    if( rs ) { 
      r = msg_com_recv( buffer, rs->model->input_fmt );
    } else {
      r = msg_com_recv( NULL, NULL ); /* flush input */
    }
    break;
  case MSG_TYP_HB:
    msg_typ = MSG_TYP_NONE;
  case MSG_TYP_NONE:
    return 0;
  }

  if( r ) {
    msg_typ = MSG_TYP_NONE;
  }

  return r > 0;
}


void simcom_send_state( robot_state *rs ) {
  struct base_state id_state = { THOR_PROTOCOL_VERSION, rs->vmState.nodeId, 
                                          ROBOT_ERROR_ID, rs->model->typ };
  msg_com_send( rs->vmState.nodeId, &id_state, id_fmt, NULL );
  msg_com_send( rs->vmState.nodeId, rs->send_state, rs->model->state_fmt, 
                rs->prev_state );
}


void simcom_flush() {
  msg_com_flush();
}


void simcom_send_heartbeat() {
  static struct base_state hb_state = { THOR_PROTOCOL_VERSION, 0, 
                                          ROBOT_ERROR_HEARTBEAT };
  msg_com_send( hb_state.id, &hb_state, err_fmt, NULL );
  msg_com_flush();
}


extern void simcom_send_error( uint16_t err ) {
  struct base_state err_state = { THOR_PROTOCOL_VERSION, 0, err };
  msg_com_send( err_state.id, &err_state, err_fmt, NULL );
  msg_com_flush();
}

void simcom_init() { 
  struct simcom_hello hello = { THOR_PROTOCOL_VERSION, 1, 32767, 0, 
                                (int16_t *)buffer };
  robot_model *m;

  for( m = robot_get_model_list(); m; m = m->next ) {
    hello.models[hello.num_models] = m->typ;
    hello.num_models++;
  }

  msg_com_send( 1, &hello, hello_fmt, NULL );
  msg_com_flush();
}
