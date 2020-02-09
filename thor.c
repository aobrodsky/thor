#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/select.h>
#include <signal.h>

#include "thor.h"
#include "robot.h"
#include "thymio.h"
#include "logger.h"
#include "buffer.h"
#include "replay.h"
#include "global.h"

#define THOR_V1_PORT 33333
#define THOR_V4_PORT 34333

static void thor( int sock );
static unsigned char *frm = (unsigned char *) "\0\0\0\0";
static unsigned short frmp;
static char *logname = "thor.log";
static int replay_fd;

extern void byebye( char *s ) {
  if( s ) {
    logger_printf( "%s failed: %s", s, strerror( errno ) );
  }
  logger_printf( "Connection closed to %d.%d.%d.%d:%d", frm[0], frm[1], frm[2],
                 frm[3], frmp );

  exit( errno );
}


static void thor_main_loop( int ss ) {
  struct sockaddr_in client;
  int len;
  pid_t pid;
  int local;
  int sock;

  frm = (unsigned char *) &client.sin_addr.s_addr;

  for( ;; ) {
    while( waitpid( -1, NULL, WNOHANG ) > 0 );

    len = sizeof( client );
    sock = accept( ss, (struct sockaddr *)&client, (socklen_t *)&len );
    frmp = ntohs( client.sin_port );

    local = ntohl( client.sin_addr.s_addr ) == INADDR_LOOPBACK;
    if( sock < 0 ) {
      byebye( "accept()" );
    } else if( !ISSET_FLAG( GLOBAL_ALLOW_REMOTE ) && !local ) {
      logger_printf( "Oops: only local commections allowed." );
      close( sock );
    } else {
      pid = 0;
      if( !ISSET_FLAG( GLOBAL_DEBUG ) ) {
        pid = fork(); 
      }
      if( pid < 0 ) {
        byebye( "fork()" );
      } else if( !pid ) {
        close( ss );
        thor( sock );
        break;
      } else {
        close( sock );
      }
    }
  }
}


static void usage() {
  printf( "usage: ./thor [options]\n" );
  printf( "options:\n" );
  printf( "  -d            : debug mode (don't fork servers), used with GDB\n");
  printf( "  -h            : print this help nessage\n" );
  printf( "  -l <log file> : log message to specified log file \n" );
  printf( "  -p <secs>     : output performance stats every secs seconds\n" );
  printf( "  -q            : quiet mode, do not output logs\n" );
  printf( "  -r            : allow remote connections\n" );
  printf( "  -D            : create a replay log for later debugging\n" );
  printf( "  -R <replay file> : replay debug mode \n" );
  printf( "  -H            : replay at high speed\n" );
  printf( "  -v1           : accept version 1 protocl connections\n" );
  printf( "  -L            : Log inbound out-of-band packets\n" );
}

static void parseArgs( int argc, char **argv ) {
  int i;

  for( i = 1; i < argc; i++ ) {
    if( !strcmp( "-r", argv[i] ) ) {
      SET_FLAG( GLOBAL_ALLOW_REMOTE );
    } else if( !strcmp( "-v1", argv[i] ) ) {
      SET_FLAG( GLOBAL_V1_PORT );
    } else if( !strcmp( "-d", argv[i] ) ) {
      SET_FLAG( GLOBAL_DEBUG );
    } else if( !strcmp( "-p", argv[i] ) ) {
      i++;
      if( i < argc ) {
        performance = atoi( argv[i] );
      }
    } else if( !strcmp( "-l", argv[i] ) ) {
      i++;
      if( i < argc ) {
        logname = argv[i];
      } else {
        logname = NULL;
      }
    } else if( !strcmp( "-q", argv[i] ) ) {
      SET_FLAG( GLOBAL_QUIET );
    } else if( !strcmp( "-R", argv[i] ) && ( ( i + 1 ) < argc ) ) {
      i++;
      replay_fd = open( argv[i], O_RDONLY );
      if( replay_fd < 1 ) {
        printf( "Could not open replay file\n" );
        exit( 1 );
      }
      SET_FLAG( GLOBAL_DEBUG );
    } else if( !strcmp( "-h", argv[i] ) ) {
      usage();
      exit( 0 );
    } else if( !strcmp( "-D", argv[i] ) ) {
      SET_FLAG( GLOBAL_RECORD );
    } else if( !strcmp( "-H", argv[i] ) ) {
      SET_FLAG( GLOBAL_HISPEED );
    } else if( !strcmp( "-L", argv[i] ) ) {
      SET_FLAG( GLOBAL_LOG_MORE );
    } else {
      printf( "Unknown argument: %s\n", argv[i] );
      usage();
      exit( 1 );
    }
  }
}

static void segfault_handler( int sig, siginfo_t *si, void *unused ) {
    logger_printf("Got SIGSEGV at address: 0x%lx",(long) si->si_addr);
    abort();
}

static void busfault_handler( int sig, siginfo_t *si, void *unused ) {
    logger_printf("Got SIGBUS at address: 0x%lx",(long) si->si_addr);
    abort();
}

int main( int argc, char **argv ) {
  struct sockaddr_in self;
  int yes = 1;
  pid_t pid;
  struct sigaction sa;
  int ss;

  sa.sa_flags = SA_SIGINFO;
  sigemptyset( &sa.sa_mask );
  sa.sa_sigaction = segfault_handler;
  if( sigaction( SIGSEGV, &sa, NULL ) == -1) {
    byebye( "sigaction" );
  }
  sa.sa_sigaction = busfault_handler;
  if( sigaction( SIGBUS, &sa, NULL ) == -1) {
    byebye( "sigaction" );
  }
  signal( SIGPIPE, SIG_IGN );

  parseArgs( argc, argv );

  thymio_init_model();
  logger_init( logname, !ISSET_FLAG( GLOBAL_QUIET ) );
  logger_setname( "thor" );

  self.sin_family = AF_INET;
  if( ISSET_FLAG( GLOBAL_ALLOW_REMOTE ) ) {
    self.sin_addr.s_addr = htonl( INADDR_ANY );
  } else {
    self.sin_addr.s_addr = htonl( INADDR_LOOPBACK );
  }

  if( ISSET_FLAG( GLOBAL_V1_PORT ) ) {
    pid = fork();
  } else {
    pid = 0;
  }

  if( pid < 0 ) {
    byebye( "fork()" );
  } else if( pid > 0 ) {
    self.sin_port = htons( THOR_V1_PORT );
  } else if( replay_fd ) {
    self.sin_port = htons( THOR_REPLAY_PORT );
    RESET_FLAG( GLOBAL_V1_PORT );
  } else {
    self.sin_port = htons( THOR_V4_PORT );
    RESET_FLAG( GLOBAL_V1_PORT );
  }

  ss = socket( PF_INET, SOCK_STREAM, 0 );
  if( ss < 0 ) {
    byebye( "socket()" );
  } else if (setsockopt( ss, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int) ) < 0){
    byebye("setsockopt()");
  } else if (setsockopt( ss, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(int) ) < 0){
    byebye("setsockopt()");
  } else if( bind( ss, (struct sockaddr *)&self, sizeof( self ) ) )  {
    byebye( "bind()" );
  } else if( listen( ss, 64 ) ) {
    byebye( "listen()" );
  }

  if( replay_fd ) {
    replay_start_rerun( replay_fd, ISSET_FLAG( GLOBAL_HISPEED ) );
    close( replay_fd );
  }

  thor_main_loop( ss );

  return 0;
}


static void thor( int sock ) {
  int n;
  fd_set sel;
  fd_set err;
  struct timeval tv;

  logger_printf( "Connection from %d.%d.%d.%d:%d", frm[0], frm[1], frm[2], 
                                                   frm[3], frmp );

  srandom( getpid() ); /* Every instance is initalized with diff seed */
  /* srandomdev(); */ /* not available on linux */

  if( ISSET_FLAG( GLOBAL_RECORD ) ) {
    replay_start_log();
  }
  buffer_init( sock, !ISSET_FLAG( GLOBAL_V1_PORT ) );
  robot_init( ISSET_FLAG( GLOBAL_V1_PORT ) );

  for( ;; ) {
    if( !robot_event_pending( &tv ) ) {
      FD_ZERO( &sel );
      FD_ZERO( &err );
      FD_SET( sock, &sel );
      FD_SET( sock, &err );
      n = select( sock + 1, &sel, NULL, &err, &tv );

      if( n < 0 ) { 
        byebye( "select()" );
      } else if( FD_ISSET( sock, &err ) ) {
        break;
      } else if( FD_ISSET( sock, &sel ) ) {
        robot_incoming_event();
      }
    }
    robot_step();
  }
}


extern void AsebaPutVmToSleep( AsebaVMState *vm ) {
  AsebaVMEmitNodeSpecificError( vm, "Sleep mode not supported" );
}


extern void AsebaSendBuffer( AsebaVMState *vm, const uint8_t* data, 
                             uint16_t length ) {
  buffer_send( vm, data, length );
}


extern uint16_t AsebaGetBuffer( AsebaVMState *vm, uint8_t* data, 
                                uint16_t maxlength, uint16_t* source ) {
  return buffer_recv( vm, data, maxlength, source );
}


extern const AsebaVMDescription* AsebaGetVMDescription( AsebaVMState *vm ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  robot_model *m = rs->model;

  return m->get_vm_desc( vm );
}


extern const AsebaNativeFunctionDescription * const * 
                       AsebaGetNativeFunctionsDescriptions( AsebaVMState *vm ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  robot_model *m = rs->model;

  return m->get_func_desc( vm );
}


extern void AsebaNativeFunction( AsebaVMState *vm, uint16_t id ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  robot_model *m = rs->model;

  m->native_function( vm, id );
}


extern const AsebaLocalEventDescription * AsebaGetLocalEventsDescriptions(
        AsebaVMState *vm ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  robot_model *m = rs->model;

  return m->get_local_events_desc( vm );
}


extern void AsebaWriteBytecode( AsebaVMState *vm ) {         /* Not Supported */
  robot_state *rs = robot_get_state( vm->nodeId );
  robot_model *m = rs->model;

  if( m->write_bytecode ) {
    m->write_bytecode( vm );
  } else {
    logger_printf( "Received request to write bytecode into flash\n" );
  }
}


extern void AsebaResetIntoBootloader( AsebaVMState *vm ) {
  robot_state *rs = robot_get_state( vm->nodeId );
  robot_model *m = rs->model;

  if( m->reset_into_bootloader ) {
    m->reset_into_bootloader( vm );
  } else {
    logger_printf( "Received request to reset into bootloader\n" );
  }
}


#ifdef ASEBA_ASSERT
extern void AsebaAssert( AsebaVMState *vm, AsebaAssertReason reason )
{
	switch( reason ) {
	case ASEBA_ASSERT_UNKNOWN: 
          logger_printf( "Fatal error: undefined" );
          break;
	case ASEBA_ASSERT_UNKNOWN_BINARY_OPERATOR: 
          logger_printf( "Fatal error: unknown binary operator" );
          break;
	case ASEBA_ASSERT_UNKNOWN_BYTECODE: 
          logger_printf( "Fatal error: unknown bytecode" );
          break;
	case ASEBA_ASSERT_STACK_OVERFLOW: 
          logger_printf( "Fatal error: stack overflow" );
          break;
	case ASEBA_ASSERT_STACK_UNDERFLOW: 
          logger_printf( "Fatal error: stack underflow" );
          break;
	case ASEBA_ASSERT_OUT_OF_VARIABLES_BOUNDS: 
          logger_printf( "Fatal error: out of variables bounds" );
          break;
	case ASEBA_ASSERT_OUT_OF_BYTECODE_BOUNDS: 
          logger_printf( "Fatal error: out of bytecode bounds" );
          break;
	case ASEBA_ASSERT_STEP_OUT_OF_RUN: 
          logger_printf( "Fatal error: step out of run" );
          break;
	case ASEBA_ASSERT_BREAKPOINT_OUT_OF_BYTECODE_BOUNDS: 
          logger_printf( "Fatal error: breakpoint out of bytecode bounds" );
          break;
	default: 
          logger_printf( "Fatal error: unknown exception" );
          break;
	}
	logger_printf( "pc = %d, sp = %d", vm->pc, vm->sp );
	logger_printf( "Resetting VM" );
	AsebaVMInit( vm );
}
#endif

void AsebaVMRunCB(AsebaVMState *vm) {
  robot_state *rs = robot_get_state( vm->nodeId );
  robot_model *m = rs->model;

  if( m->vm_run_cb ) {
    m->vm_run_cb( vm );
  }
}

void AsebaVMErrorCB(AsebaVMState *vm, const char * msg) {
  robot_state *rs = robot_get_state( vm->nodeId );
  robot_model *m = rs->model;

  if( m->vm_error_cb ) {
    m->vm_error_cb( vm, msg );
  }
}

void AsebaVMResetCB(AsebaVMState *vm) {
  robot_state *rs = robot_get_state( vm->nodeId );
  robot_model *m = rs->model;

  if( m->vm_reset_cb ) {
    m->vm_reset_cb( vm );
  }
}
