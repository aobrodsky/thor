#include <stdarg.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include "logger.h"

static FILE *log;
static int con_log;
static char *process = "thor";

static char *prefix( char *buffer ) {
  time_t c = time( NULL );
  snprintf( buffer, 80, "%s%s[%d]: ", ctime( &c ), process, getpid() );
  buffer[24] = ' ';
  return buffer;
}


extern int logger_init( char *name, int console ) {
  con_log = console;

  if( name ) {
    log = fopen( name, "a" );
    if( log ) {
      logger_printf( "Log opened" );
    }
  }
  return !name || log;
}
 

extern void logger_setname( char *proc ) {
  process = proc;
}


static void output( FILE *f, char *pref, char *fmt, va_list ap ) {
  fprintf( f, "%s", pref );
  vfprintf( f, fmt, ap );
  fprintf( f, "\n" );
  va_end( ap );
  fflush( f );
}


extern void logger_printf( char *fmt, ... ) {
  char buffer[80];
  va_list ap;

  prefix( buffer );
  
  if( con_log ) {
    va_start( ap, fmt );
    output( stdout, buffer, fmt, ap );
  }

  if( log ) {
    va_start( ap, fmt );
    output( log, buffer, fmt, ap );
    fsync( fileno( log ) );
  }
}

