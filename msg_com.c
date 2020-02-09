#include <stddef.h>
#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/select.h>
#include <poll.h>

#include "thor.h"
#include "world.h"
#include "../../common/types.h"
#include "msg_com.h"
#include "logger.h"
#include "buffer.h"

#define EOT "EOT"
#define EOS "EOS"
#define FLDT ";"
#define HUP "EOT"
#define BUF_SIZ 65536

#ifndef EBADMSG
  #define EBADMSG ENOMSG
#endif

static char *readbuf;
static char *errbuf;
static char *curptr;

static int com_error;

static int msg_received() {
  static uint8_t buffer[65536];
  static int buflen;
  int offset;
  int len;
  int r;

  if( !readbuf ) {
    readbuf = malloc( BUF_SIZ );
    errbuf = malloc( BUF_SIZ );
    if( !readbuf || !errbuf ) {
      byebye( "msg_recevied()" );
    } 
    buflen = BUF_SIZ;
    curptr = readbuf;
    *readbuf = 0;
  }

  while( !strstr( readbuf, EOT ) ) {
    r = buffer_ob_recv( buffer, 65535, NULL );
    if( !r ) {
      return 0;
    }
    buffer[r] = 0;
    offset = strlen( readbuf );
    len = buflen - offset;

    if( r >= len ) {
      buflen += BUF_SIZ;
      readbuf = realloc( readbuf, buflen );
      errbuf = realloc( errbuf, buflen );
      if( !readbuf || !errbuf ) {
        byebye( "msg_recevied()" );
      }
      curptr = readbuf;
    }

    strncpy( &readbuf[offset], (char *)buffer, r + 1 );
    offset += r;
    readbuf[offset] = 0;
  } 

  strncpy( errbuf, readbuf, buflen );
  return 1;
}


static char *get_token() {
  char *tok;

  for( ; isspace( *curptr ); curptr++ );
  if( *curptr ) {
    for( tok = curptr; *curptr && !isspace( *curptr ); curptr++ );
    *curptr = 0;
    curptr++;
    for( ; isspace( *curptr ); curptr++ );
    if( !strcmp( tok, EOT ) ) {
      memmove( readbuf, curptr, strlen( curptr ) + 1 );
      curptr = readbuf;
      tok = EOT;
    }
  } else {
    tok = EOT;
    curptr = readbuf;
    *curptr = 0;
  }
  return tok;
}


static void flush_tokens() {
  while( strcmp( get_token(), EOT ) ); 
}


static void unknown_field( char *tok, struct msg_com_fmt *fmt ) {
  static char buffer[8192];
  int len = 0;
  int i;

  for( i = 0; fmt[i].fld && ( len < 8192 ); i++ ) {
    len += snprintf( buffer + len, 8192 - len, "%s ", fmt[i].fld );
  }
  logger_printf( "Field '%s' not found in: %s", tok, buffer );
  logger_printf( "Error buffer: %s", errbuf );
}


static int msg_decode( void *dst, struct msg_com_fmt *fmt ) {
  int i, j, r, num;
  void *p;
  char *t;

  for( t = get_token(); strcmp( t, EOS ); t = get_token() ) {
    for( i = 0; fmt[i].fld && strcmp( fmt[i].fld, t ); i++ );

    if( !fmt[i].fld ) { 
      unknown_field( t, fmt );
      flush_tokens();
      com_error = EBADMSG;
      return 0;
    }

    t = fmt[i].off + (char *)dst;
    if( fmt[i].num < 0 ) {
      num = *(uint32_t *)(dst + fmt[i - 1].off);
      p = malloc( num * fmt[i].siz );
      if( !p ) {
        flush_tokens();
        com_error = errno;
        return 0;
      }
      
      *(void **)t = p;
      t = (char *)p;
    } else {
      num = fmt[i].num;
    }
    for( j = 0; j < num; j++ ) {
      switch( fmt[i].typ ) {
      case MSG_FIELD_SINT16:
        r = sscanf( get_token(), "%hd", (int16_t *)t );
        break;
      case MSG_FIELD_CHAR:
        r = sscanf( get_token(), "%hhd", t );
        break;
      case MSG_FIELD_UINT32:
        r = sscanf( get_token(), "%u", (uint32_t *)t );
        break;
      case MSG_FIELD_COORD:
        r = sscanf( get_token(), "%d", (int *)t );
        break;
      case MSG_FIELD_ANGLE:
        r = sscanf( get_token(), "%lf", (double *)t );
        break;
      case MSG_FIELD_VECT:
        r = sscanf( get_token(), "%d", (int *)(t + offsetof(tw_vect, x)) );
        r += sscanf( get_token(), "%d", (int *)(t + offsetof(tw_vect, y)) );
        r /= 2;
        break;
      case MSG_FIELD_STRUCT:
        r = msg_decode( t, fmt[i].fmt );
        break;
      default:
        abort();
      }
      t += fmt[i].siz;
      if( r < 1 ) {
        flush_tokens();
        com_error = EBADMSG;
        return 0;
      }
    }

    if( strcmp( FLDT, get_token() ) ) {
      flush_tokens();
      com_error = EBADMSG;
      return 0;
    }
  }

  return 1;
}


int msg_com_recv( void *dst, struct msg_com_fmt *fmt ) {
  if( msg_received() ) {
    if( !dst || !fmt ) {
      flush_tokens();
      return 1;
    } else if( msg_decode( dst, fmt ) ) {
      if( strcmp( EOT, get_token() ) ) {
        flush_tokens();
      }
      return 1;
    } else {
      logger_printf( "msg_decode failed: %s\n", strerror( com_error ) );
    }
  }
  return 0;
}


static int msg_encode( char *buf, int len, void *data, struct msg_com_fmt *fmt, 
                       void *prv_dt ) {
  int i, j, num;
  char *s;
  char *prev;

  for( i = 0; fmt[i].typ; i++ ) {
    s = fmt[i].off + (char *) data;
    if( prv_dt && ( fmt[i].num > 0 ) && ( fmt[i].typ != MSG_FIELD_STRUCT ) ) {
      prev = fmt[i].off + (char *) prv_dt;
      if( !memcmp( s, prev, fmt[i].num * fmt[i].siz ) ) {
        continue;
      }
      memcpy( prev, s, fmt[i].num * fmt[i].siz );
    }

    len += snprintf( &buf[len], BUF_SIZ - len, "%s ", fmt[i].fld );

    if( fmt[i].num < 0 ) {
      num = *(uint32_t *)(data + fmt[i - 1].off);
      s = *(char **)s;
    } else {
      num = fmt[i].num;
    }

    for( j = 0; j < num; j++ ) {
      switch( fmt[i].typ ) {
      case MSG_FIELD_SINT16:
        len += snprintf( &buf[len], BUF_SIZ - len, "%d ", *(int16_t *)s );
        break;
      case MSG_FIELD_CHAR:
        len += snprintf( &buf[len], BUF_SIZ - len, "%d ", *s );
        break;
      case MSG_FIELD_UINT32:
        len += snprintf( &buf[len], BUF_SIZ - len, "%u ", *(uint32_t *)s );
        break;
      case MSG_FIELD_ANGLE:
        len += snprintf( &buf[len], BUF_SIZ - len, "%f ", *(tw_angle *)s );
        break;
      case MSG_FIELD_COORD:
        len += snprintf( &buf[len], BUF_SIZ - len, "%d ", *(tw_coord *)s );
        break;
      case MSG_FIELD_VECT:
        len += snprintf( &buf[len], BUF_SIZ - len, "%d %d ", ((tw_vect *)s)->x,
                                                           ((tw_vect *)s)->y );
        break;
      case MSG_FIELD_STRUCT:
        len += msg_encode( buf, len, s, fmt[i].fmt, NULL );
        break;
      }
      s += fmt[i].siz;
    }
    len += snprintf( &buf[len], BUF_SIZ - len, "%s ", FLDT );
  }

  len += snprintf( &buf[len], BUF_SIZ - len, "%s ", EOS );

  return len;
}

void msg_com_old_send( uint16_t src, void *data, struct msg_com_fmt *fmt, 
                   void *prv_dt ) {
  static char packet[BUF_SIZ];
  int len;

  len = msg_encode( packet, 0, data, fmt, prv_dt );
  len += snprintf( &packet[len], BUF_SIZ - len, "%s\n", EOT );

  if( len >= BUF_SIZ ) {
    logger_printf( "Buffer too small for encoded message" );
    len = BUF_SIZ - 1;
  } 

  buffer_ob_send( src, (uint8_t *)packet, len );
}


static char buffer[BUF_SIZ];
static int bufpos;
static int last_src;

void msg_com_send( uint16_t src, void *data, struct msg_com_fmt *fmt, 
                   void *prv_dt ) {
  static char packet[BUF_SIZ];
  int len;

  len = msg_encode( packet, 0, data, fmt, prv_dt );
  len += snprintf( &packet[len], BUF_SIZ - len, "%s\n", EOT );

  if( len >= BUF_SIZ ) {
    logger_printf( "Buffer too small for encoded message" );
    len = BUF_SIZ - 1;
  } 

  if( ( bufpos + len ) >= BUF_SIZ ) {
    buffer_ob_send( src, (uint8_t *)buffer, bufpos );
    bufpos = 0;
  }

  strncpy( &buffer[bufpos], packet, len );
  bufpos += len;
  last_src = src;
}


void msg_com_flush() {
  if( bufpos > 0 ) {
    buffer_ob_send( last_src, (uint8_t *)buffer, bufpos );
    bufpos = 0;
  }
}
