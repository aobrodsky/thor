#ifndef SIM_COM_H
#define SIM_COM_H

#include "thor.h"
#include "sim.h"
#include "world.h"

enum {
  MSG_TYP_ENV,
  MSG_TYP_MOD_BOT,
  MSG_TYP_INP,
  MSG_TYP_HB, /* heart beat, do nothing */
  MSG_TYP_MOD_OBJ,
  MSG_TYP_DEL_OBJ,
  MSG_TYP_DEL_BOT,
  MSG_TYP_MOD_ARENA,
  MSG_TYP_NONE = -1
};

enum {
  OBJ_TYP_BLOCK,
  OBJ_TYP_PATH,
  OBJ_TYP_POLYGON_BLOCK,
  OBJ_TYP_POLYGON_MARK,
  OBJ_TYP_LAST
};

struct simcom_env_poly {
  uint32_t num;
  tw_vect *verts;
};

struct simcom_env_obj {
  int16_t                 id;
  int16_t                 typ;
  int16_t                 clr;
  tw_vect                 org;
  tw_vect                 pos;
  tw_angle                dir;
  uint32_t                num;
  struct simcom_env_poly *polys;
};

struct simcom_env_cfg {
  int16_t   id;
  tw_vect  pos;
  tw_angle dir;
  int16_t   typ;
};

struct simcom_env_new {
  tw_coord                length;
  tw_coord                width;
  uint32_t                  num;
  struct simcom_env_obj  *objs;
  uint32_t                  num_bots;
  struct simcom_env_cfg  *robots;
};

struct simcom_hello {
  int16_t  ver;
  int16_t  id;
  int16_t  bots;
  uint32_t  num_models;
  int16_t *models;
};

union simcom_env_msgs {
  struct simcom_env_new    new_env;
  struct simcom_env_cfg    mov_obj;
  struct simcom_env_obj    env_obj;
  void *                   inputs;
};

struct simcom_env_msg {
  int16_t               typ;
  int16_t               ver;
  int16_t               id;
  union simcom_env_msgs data;
};


extern void simcom_init();
extern void simcom_send_state( robot_state *rs );
extern void simcom_send_error( uint16_t err );
extern void simcom_send_heartbeat();
extern void simcom_flush();
extern int simcom_recv_env_msg( struct simcom_env_msg *msg );

#endif
