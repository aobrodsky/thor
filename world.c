#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "world.h"

static tw_obj *objs;
static tw_obj *marks;
static tw_obj *robot;
static tw_coord length;
static tw_coord width;

static void nuke_objs( tw_obj *o );
static int bounding_box( tw_obj *o );
static int in_bounding_box( tw_vect *pos, tw_obj *o );
static int in_hull( tw_vect *pos, tw_obj *o );
static int transform( tw_obj *o );
static tw_obj * collision( tw_obj *list, tw_obj *obj );
static int hulls_intersect( tw_obj *o, tw_obj *p );
static double dist( tw_vect *u, tw_vect *v );
static int bounding_box_intersect( tw_obj *o, tw_vect *s, tw_vect *t );
static int line_intersect( tw_vect *s, tw_vect *t, tw_vect *u, tw_vect *v, 
                           tw_vect *res );
static double intersect( tw_obj *list, tw_obj *src, tw_vect *s, tw_vect *t, 
                         tw_vect *w, tw_obj **obj );
static tw_coord cross_prod( tw_vect *o, tw_vect *l, tw_vect *r );
static int in_bounds( tw_obj *o );
static int ray_line_intersect( tw_vect *s, tw_vect *t, tw_vect *p );

#define DET(a,b,c,d) (((a) * (d)) - ((b) * (c)))
#define BTW(a,s,t) ((s) < (t) ? ((s) <= (a)) && ((a) <= (t)) : \
                                ((t) <= (a)) && ((a) <= (s)))

int thor_world_init( tw_coord l, tw_coord w ) {
   nuke_objs( objs );
   nuke_objs( marks );
   nuke_objs( robot );
   objs = NULL; 
   marks = NULL; 
   robot = NULL; 
   length = l;
   width = w;

   return 1;
}

int thor_world_resize( tw_coord l, tw_coord w ) {
   length = l;
   width = w;

   return 1;
}

int thor_world_cross_prod( tw_vect *s, tw_vect *t, tw_vect *u ) {
  return cross_prod( s, t, u );
}

tw_obj * thor_world_make_object( int type, int clr, int size, tw_vect *orig, 
                                 tw_angle dir, tw_vect *pos, tw_vect *v, 
                                 tw_obj *part ) {
  tw_obj *o;

  assert( orig );
  assert( pos );
  assert( v );

  o = malloc( sizeof( tw_obj ) + ( size * sizeof( tw_vect ) ) );

  if( o ) {
    memset( o, 0, sizeof( tw_obj ) + ( size * sizeof( tw_vect ) ) ); 
    o->type = type;
    o->colour = clr;
    o->size = size;
    o->orig = *orig;
    o->dir = dir;
    o->pos = *pos;
    o->part = part;
    o->init = v;
  }
  return o;
}


int thor_world_add_object( tw_obj *obj ) {
  tw_obj *o;

  assert( obj );

  if( obj ) {
    for( o = obj; o && transform( o ); o = o->part );
    if( !o ) { 
      switch( THOR_WORLD_TYPE( obj->type ) ) {
      case THOR_WORLD_ROBOT:
        obj->next = robot;
        robot = obj;
        break;
      case THOR_WORLD_MARK:
        obj->next = marks;
        marks = obj;
        break;
      case THOR_WORLD_OBJECT:
        obj->next = objs;
        objs = obj;
        break;
      default:
        return 0;
      }
      return 1;
    }
  }
  return 0;
}


static void remove_object( tw_obj **list, tw_obj *obj ) {
  tw_obj *o;

  if( !*list ) { /* object already gone */
    return;
  } else if( *list == obj ) {
    *list = obj->next; 
  } else {
    for( o = *list; o->next && ( o->next != obj ); o = o->next );
    if( o->next ) {
      o->next = obj->next;
    }
  }
  obj->next = NULL;
  nuke_objs( obj );
}


int thor_world_free_object( tw_obj *obj ) {
  if( obj ) {
    switch( THOR_WORLD_TYPE( obj->type ) ) {
    case THOR_WORLD_ROBOT:
      remove_object( &robot, obj );
      break;
    case THOR_WORLD_MARK:
      remove_object( &marks, obj );
      break;
    case THOR_WORLD_OBJECT:
      remove_object( &objs, obj );
      break;
    default:
      return 0;
    }
    return 1;
  }
  return 0;
}


int thor_world_move_object( tw_obj *obj, tw_vect *pos, tw_angle dir ) {
  tw_obj *o;
  int rc = 1;
  
  assert( obj );
  assert( pos );

  for( o = obj; rc && o; o = o->part ) {
    o->dir = dir;
    o->pos.x = pos->x;
    o->pos.y = pos->y;
    rc = transform( o );
  }

  return rc; 
}


tw_obj * thor_world_over_mark( tw_obj *pt ) {
  static tw_obj abyss;
  tw_obj *o;
  tw_obj *p;
 
  assert( pt );

  if( !in_bounds( pt ) ) {
    return &abyss;
  }

  for( o = marks; o; o = o->next ) {
    for( p = o; p; p = p->part ) {
      if( in_bounding_box( pt->vert, p ) && in_hull( pt->vert, p ) ) {
        return o;
      }
    }
  }      
  return o;
}


void thor_world_transform( tw_obj *obj, tw_vect * v ) {
  double c = cos( obj->dir );
  double s = sin( obj->dir );
  double x = v->x;
  double y = v->y;

  v->x = (tw_coord)( ( x * c ) + ( y * s ) ); /* rotate */
  v->y = (tw_coord)( ( -x * s ) + ( y * c ) );

  v->x += obj->pos.x; /* translate to pos */
  v->y += obj->pos.y;
}


tw_obj * thor_world_collision( tw_obj *obj ) {
  tw_obj *o;

  assert( obj );
  o = collision( robot, obj );
  if( !o ) {
    o = collision( objs, obj );
  }
  return o;
}


static tw_obj * collision( tw_obj *list, tw_obj *obj ) {
  tw_obj *p;
  tw_obj *q;

  assert( obj );

  for( ; list; list = list->next ) {
    if( list == obj ) {
      continue;
    }

    for( p = list; p; p = p->part ) {
      for( q = obj; q; q = q->part ) {
        if( hulls_intersect( q, p ) ) {
          return list;
        }
      }
    }
  }
  return list;
}


tw_coord thor_world_intersect( tw_obj *src, tw_vect *s, tw_vect *t, tw_vect *w, 
                               tw_obj **o ){
  tw_coord omin;
  tw_obj   *targ;
  tw_coord rmin;

  assert( s );
  assert( t );
  assert( w );

  rmin = (tw_coord) ceil( intersect( robot, src, s, t, w, &targ ) );
  omin = (tw_coord) ceil( intersect( objs, NULL, s, t, w, o ) );
  if( rmin < omin ) {
    *o = targ;
    omin = rmin;
  }
  return omin;
}


static double intersect( tw_obj *list, tw_obj *src, tw_vect *s, tw_vect *t, 
                         tw_vect *w, tw_obj **obj ) {
  tw_obj *p;
  tw_vect u;
  double d_min = dist( s, t ) + 1;
  tw_coord d;
  int i;

  assert( s );
  assert( t );
  assert( w );

  for( ; list; list = list->next ) {
    if( list != src ) {
      for( p = list; p; p = p->part ) {
        if( ( p->size <=  5 ) || bounding_box_intersect( p, s, t ) ) {
          for( i = 0; i < p->size; i++ ) {
            if( line_intersect( s, t, &p->vert[i], &p->vert[(i + 1) % p->size], 
                                &u ) ) {
              d = dist( s, &u );
              if( d < d_min ) {
                *w = u;
                d_min = d;
                if( obj ) {
                  *obj = p;
                }
              }
            }
          }
        }
      }
    }
  }
  return d_min;
}


static int transform( tw_obj *o ) {
  int i;
  double c;
  double s;
  tw_coord x, y;

  assert( o );

  c = cos( o->dir );
  s = sin( o->dir );

  if( !o->init ) {
    return 0;
  }

  for( i = 0; i < o->size; i++ ) {
    x = o->init[i].x - o->orig.x; /* translate to 0,0 */
    y = o->init[i].y - o->orig.y;

    /* rotate */
    o->vert[i].x = (tw_coord)( ( x * c ) + ( y * s ) );
    o->vert[i].y = (tw_coord)( ( -x * s ) + ( y * c ) );

    o->vert[i].x = o->vert[i].x + o->orig.x; /* translate back to orign */
    o->vert[i].y = o->vert[i].y + o->orig.y;

    o->vert[i].x = o->vert[i].x + o->pos.x; /* translate back to pos */
    o->vert[i].y = o->vert[i].y + o->pos.y;
  }

  return bounding_box( o );
}


static int in_bounding_box( tw_vect *pos, tw_obj *o ) {
  assert( pos );
  assert( o );

  return ( pos->x >= o->bb_bot_left.x ) && ( pos->x <= o->bb_top_right.x ) &&
         ( pos->y >= o->bb_bot_left.y ) && ( pos->y <= o->bb_top_right.y );
}


static tw_coord cross_prod( tw_vect *o, tw_vect *l, tw_vect *r ) {
  assert( l );
  assert( o );
  assert( r );

  return DET( l->x - o->x, l->y - o->y, r->x - o->x, r->y - o->y );

  /* return ( ( l->x - o->x ) * ( r->y - o->y ) ) - 
           ( r->x - o->x ) * ( l->y - o->y ) ); */
}


static int in_hull( tw_vect *pos, tw_obj *o ) {
  int i, y1, y2;
  int n = 0;

  assert( pos );
  assert( o );

  for( i = 0; i < o->size; i++ ) {
    n += ray_line_intersect( &o->vert[i], &o->vert[( i + 1 ) % o->size], pos );
  }

  if( n & 1 ) {  /* n is odd, so return 1  */
    return 1;
  } else if( !n ) { /* no crossings, so 0 */
    return 0;
  }

  for( i = 0; i < o->size; i++ ) {
    /* check for vertex intersections */
    if( ( o->vert[i].y == pos->y ) && ( o->vert[i].x < pos->x ) ) {
      y1 = o->vert[( i + 1 ) % o->size].y - pos->y;
      y2 = o->vert[( i - 1 + o->size ) % o->size].y - pos->y;
      if( ( y1 * y2 ) < 0 ) {
        n--;
      }
    }
  }

  return n & 1;  /* if n is odd, point is in hull */
}


static int bounding_boxes_intersect( tw_obj *o, tw_obj *p ) {
  assert( o );
  assert( p );

  return !( ( o->bb_bot_left.x > p->bb_top_right.x ) ||
            ( p->bb_bot_left.x > o->bb_top_right.x ) ||
            ( p->bb_bot_left.y > o->bb_top_right.y ) ||
            ( o->bb_bot_left.y > p->bb_top_right.y ) );
}


static int segs_cross( tw_vect *s, tw_vect *t, tw_vect *u, tw_vect *v ) {
  int d; 
  double a, b;

  d = DET( t->x - s->x, t->y - s->y, u->x - v->x, u->y - v->y );
  if( d ) {
    a = DET( u->x - s->x, u->y - s->y, u->x - v->x, u->y - v->y ) / (double) d;
    b = DET( t->x - s->x, t->y - s->y, u->x - s->x, u->y - s->y ) / (double) d;
  }
  return !( !d || ( a < 0 ) || ( b < 0 ) || ( a > 1 ) || ( b > 1 ) );
}


static int hulls_intersect( tw_obj *o, tw_obj *p ) {
  int i, j;
  tw_vect *v;
  tw_vect *u;

  assert( o );
  assert( p );

  if( !bounding_boxes_intersect( o, p ) ) {
    return 0;
  }

  for( i = 0; i < o->size; i++ ) {
    u = &o->vert[i];
    v = &o->vert[( i + 1 ) % o->size];
    for( j = 0; j < p->size; j++ ) {
      if( segs_cross( u, v, &p->vert[j], &p->vert[( j + 1 ) % p->size] ) ) {
        return 1;
      }
    }
  }
  return 0;
}


static double dist( tw_vect *u, tw_vect *v ) {
  int x;
  int y;

  assert( u );
  assert( v );

  x = u->x - v->x;
  y = u->y - v->y;

  return sqrt( ( x * x ) + ( y * y ) );
}


static int bounding_box_intersect( tw_obj *o, tw_vect *s, tw_vect *t ) {
  tw_vect v;
  tw_vect u;
  tw_vect w;

  assert( s );
  assert( t );
  assert( o );

  if( in_bounding_box( t, o ) ) {
    return 1;
  }

  u.x = o->bb_bot_left.x;
  u.y = o->bb_top_right.y;
  v.x = o->bb_top_right.x;
  v.y = o->bb_bot_left.y;
  return line_intersect( s, t, &o->bb_bot_left, &u, &w ) ||
         line_intersect( s, t, &o->bb_bot_left, &v, &w ) ||
         line_intersect( s, t, &o->bb_top_right, &u, &w ) ||
         line_intersect( s, t, &o->bb_top_right, &v, &w );
}


static int line_intersect( tw_vect *s, tw_vect *t, tw_vect *u, tw_vect *v,
                           tw_vect *res ) {
  tw_coord det_den;
  tw_coord det_l1;
  tw_coord det_l2;

  assert( s );
  assert( t );
  assert( u );
  assert( v );
  assert( res );

  det_den = DET( s->x - t->x, s->y - t->y, u->x - v->x, u->y - v->y );

  if( det_den ) {
    det_l1 = DET( s->x, s->y, t->x, t->y );
    det_l2 = DET( u->x, u->y, v->x, v->y );
    res->x = DET( det_l1, s->x - t->x, det_l2, u->x - v->x ) / det_den;
    res->y = DET( det_l1, s->y - t->y, det_l2, u->y - v->y ) / det_den;

    return BTW( res->x, s->x, t->x ) && BTW( res->y, s->y, t->y ) &&
           BTW( res->x, u->x, v->x ) && BTW( res->y, u->y, v->y );
  }
  return 0;
}

static int ray_line_intersect( tw_vect *s, tw_vect *t, tw_vect *p ) {
  int d;
  double a, b;

  d = p->x * ( s->y - t->y );
  if( d ) {
    a = p->x * ( s->y - p->y ) / (double) d;
    b = DET( t->x - s->x, t->y - s->y, p->x - s->x, p->y - s->y ) / (double) d;
  }

  return !( !d || ( a < 0 ) || ( b < 0 ) || ( a > 1 ) || ( b > 1 ) );
}


static int in_bounds( tw_obj *o ) {
  assert( o );

  return ( ( o->bb_bot_left.x >= 0 ) && ( o->bb_bot_left.y >= 0 ) && 
           ( o->bb_top_right.x <= length ) && ( o->bb_top_right.y <= width ) );
}


static int bounding_box( tw_obj *o ) {
  int i;

  assert( o );

  o->bb_bot_left = o->vert[0];
  o->bb_top_right = o->vert[0];

  for( i = 0; i < o->size; i++ ) {
    if( o->bb_bot_left.x > o->vert[i].x ) {
        o->bb_bot_left.x = o->vert[i].x;
    }
    if( o->bb_bot_left.y > o->vert[i].y ) {
        o->bb_bot_left.y = o->vert[i].y;
    }
    if( o->bb_top_right.x < o->vert[i].x ) {
        o->bb_top_right.x = o->vert[i].x;
    }
    if( o->bb_top_right.y < o->vert[i].y ) {
        o->bb_top_right.y = o->vert[i].y;
    }
  }

  return !( o->type & THOR_WORLD_IN_BOUNDS ) || in_bounds( o );
}


static void nuke_objs( tw_obj *o ) {
  tw_obj *p;

  while( o ) {
    while( o->part ) {
      p = o->part;
      o->part = p->part;
      free( p );
    }
    p = o;
    o = o->next;
    free( p );
  }
}
