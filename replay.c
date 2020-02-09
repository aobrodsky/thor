#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/select.h>
#include <sys/time.h>

#include "logger.h"
#include "replay.h"

#define HEADER_SIZE ( sizeof( uint32_t ) + sizeof( uint32_t ) )
#define SIZE 65536 + HEADER_SIZE

static int logged;
static int replay_fd = -1;
static char *replay_name;
struct timeval prev;
static char buffer[SIZE];
static uint32_t offset = HEADER_SIZE;
static char replay_template[20] = "replay_log.XXXXXX";

extern void byebye( char *where );

static void replay_close( void ) {
  if( ( replay_name ) && ( replay_fd > 0 ) ) {
    close( replay_fd );
    unlink( replay_name );
  }
}

extern void replay_start_log() {
  replay_name = mktemp( replay_template );
  if( replay_name ) {
    replay_fd = open( replay_name, O_APPEND | O_CREAT | O_WRONLY, 0600 );
    if( replay_fd > 0 ) {
      atexit( &replay_close );
      gettimeofday( &prev, NULL );
    }
  }
}

extern void replay_append( void *data, int len ) {
  if( replay_fd > 0 ) {
    if( ( offset + len ) < SIZE ) {
      memcpy( &buffer[offset], data, len );
      offset += len;
    } else if( !logged ) {
      logger_printf( "Relpay buffer exceeded!" );
      logged = 1;
    }
  }
}

extern void replay_checkpoint() {
  struct timeval curr;
  uint32_t diff;

  if( replay_fd > 0 ) {
    gettimeofday( &curr, NULL );
    diff = (curr.tv_sec - prev.tv_sec) * 1000000 + curr.tv_usec - prev.tv_usec;
    memcpy( buffer, &diff, sizeof( uint32_t ) );
    memcpy( &buffer[sizeof( uint32_t )], &offset, sizeof( uint32_t ) );
    write( replay_fd, buffer, offset );
    offset = HEADER_SIZE;
    prev = curr;
    logged = 0;
  }
}

extern void replay_flush() {
  if( replay_fd > 0 ) {
    fsync( replay_fd );
  }
}

static void replay_loop( int fd, int sock, int hispeed ) {
  uint32_t delay;
  uint32_t len;

  while( read( fd, &delay, sizeof( uint32_t ) ) == sizeof( uint32_t ) ) {
    switch( read( sock, buffer, SIZE ) ) {
    case -1: 
      if( errno == EAGAIN ) {
        break;
      }
    case 0:
      return;
    }

    if( !hispeed ) { 
      logger_printf( "Sleeping %fs", delay / 1000000.0 );
      usleep( delay );
    }

    if( read( fd, &len, sizeof( uint32_t ) ) == sizeof( uint32_t ) ) {
      len -= HEADER_SIZE;
      if( ( len <= SIZE ) && ( read( fd, buffer, len ) == len ) ) {
        logger_printf( "Replaying packet of length %u bytes", len );
        if( write( sock, buffer, len ) == len ) {
          continue;
        }
      }
    }
    break;
  }

  logger_printf( "Replay done, waiting" );
  for( ;; ) {
    switch( read( sock, buffer, SIZE ) ) {
    case -1: 
      if( errno == EAGAIN ) {
        break;
      }
    case 0:
      return;
    }
    usleep( 100000 );
  }
} 

extern void replay_start_rerun( int fd, int hispeed ) {
  struct sockaddr_in thor;
  static struct timeval t;    
  int sock;
  int pid;

  pid = fork();
  if( pid < 0 ) {
    byebye( "fork()" );
  } else if( pid ) {
    return;
  }

  thor.sin_family = AF_INET;
  thor.sin_addr.s_addr = htonl( INADDR_LOOPBACK ); 
  thor.sin_port = htons( THOR_REPLAY_PORT );

  sock = socket( PF_INET, SOCK_STREAM, 0 );

  logger_setname( "replay" );

  if( !connect( sock, (struct sockaddr *)&thor, sizeof( thor ) ) ) {
    t.tv_usec = 1; 
    setsockopt( sock, SOL_SOCKET, SO_RCVTIMEO, &t, sizeof( t ) );
    logger_printf( "Replay server connected" );
    replay_loop( fd, sock, hispeed );
  }

  close( sock );
  logger_printf( "Done\n" );
  exit( 0 );
}
