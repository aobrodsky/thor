#ifndef MSG_COM_H
#define MSG_COM_H

#include <stddef.h>

enum {
  MSG_FIELD_NONE,
  MSG_FIELD_CHAR,
  MSG_FIELD_SINT16,
  MSG_FIELD_UINT32,
  MSG_FIELD_ANGLE,
  MSG_FIELD_COORD,
  MSG_FIELD_VECT,
  MSG_FIELD_STRUCT,
 };

struct msg_com_fmt {
  char                typ;  /* see above */
  struct msg_com_fmt *fmt;
  char               *fld;
  int                 off;
  int                 siz;
  int                 num;
};

void msg_com_send( uint16_t src, void *data, struct msg_com_fmt *fmt, 
                   void *prv_dt );
void msg_com_flush();
int msg_com_recv( void *data, struct msg_com_fmt *fmt );

#endif
