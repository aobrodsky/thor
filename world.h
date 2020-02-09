#ifndef THOR_WORLD_H
#define THOR_WORLD_H

#include <math.h>

typedef int tw_coord;
typedef double tw_angle;

#define RAD(d) (M_PI * (d) / 180)

enum {
  THOR_WORLD_ABYSS,
  THOR_WORLD_ROBOT,
  THOR_WORLD_MARK,
  THOR_WORLD_OBJECT,
  THOR_WORLD_POINT,
  THOR_WORLD_NULL
};

#define THOR_WORLD_IN_BOUNDS 0x8000
#define THOR_WORLD_TYPE(x) ((x) & 0xff)

typedef struct thor_world_vect {
  tw_coord x;
  tw_coord y;
} tw_vect;

typedef struct thor_world_obj {
  struct thor_world_obj *next;
  struct thor_world_obj *part;
  int                    type;
  int                    size;
  int                    colour;
  tw_vect                bb_bot_left;
  tw_vect                bb_top_right;
  tw_vect                orig;
  tw_angle               dir;
  tw_vect                pos;
  tw_vect                *init;
  tw_vect                vert[1];
} tw_obj;

int thor_world_init( tw_coord length, tw_coord width );
int thor_world_resize( tw_coord length, tw_coord width );
tw_obj * thor_world_make_object( int type, int clr, int size, tw_vect *orig, 
                                 tw_angle dir, tw_vect *pos, tw_vect *v, 
                                 tw_obj *part );
int thor_world_add_object( tw_obj *obj );
int thor_world_free_object( tw_obj *obj );
int thor_world_move_object( tw_obj *obj, tw_vect *pos, tw_angle dir );
tw_obj * thor_world_over_mark( tw_obj *pt );
tw_obj * thor_world_collision( tw_obj *obj );
void thor_world_transform( tw_obj *obj, tw_vect *v );
tw_coord thor_world_intersect( tw_obj *org, tw_vect *s, tw_vect *t, tw_vect *p, 
                               tw_obj **o );
int thor_world_cross_prod( tw_vect *s, tw_vect *t, tw_vect *u );

#endif
