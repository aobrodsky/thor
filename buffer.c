#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

#include "thor.h"
#include "buffer.h"
#include "logger.h"
#include "replay.h"

typedef struct aseba_packet {
  struct aseba_packet *next;
  int                 ref_count;
  uint16_t              size;
  uint16_t              len;
  uint16_t              source;
  uint8_t               data[1];
} aseba_pkt;

typedef struct packet_queue {
  struct packet_queue *next;
  AsebaVMState *key;
  aseba_pkt    *head;
} pkt_queue;

static int num_bots;
static aseba_pkt *free_list;
static aseba_pkt *buffer_head;
static aseba_pkt *buffer_tail;
static pkt_queue *queue_map;
static pkt_queue ob_q;
static int sock;
static int allow_ob;

extern void buffer_init( int sck, int ob_ok ) {
  sock = sck;
  allow_ob = ob_ok;
}

void print_buffers( AsebaVMState *vm, aseba_pkt *cur ) {
  aseba_pkt *p;

  for( p = buffer_head; p; p = p->next ) {
    if( ( p->ref_count ) && ( !vm || ( p->source != vm->nodeId ) ) ) {
      if( p == cur ) {
        printf( "*%d %d %d %d %x\n", p->source, p->ref_count, p->len, p->size, 
                *(short *)p->data );
      } else {
        printf( " %d %d %d %d %x\n", p->source, p->ref_count, p->len, p->size, 
                *(short *)p->data );
      }
    }
  }
}


static pkt_queue *get_packet_queue( AsebaVMState *vm ) {
  pkt_queue *pq;

  for( pq = queue_map; pq && pq->key !=vm; pq = pq->next );
  return pq;
}


extern void buffer_add_reader( AsebaVMState *vm ) {
  aseba_pkt *tmp;
  pkt_queue *pq = get_packet_queue( vm );

  if( !pq ) {
    pq = malloc( sizeof( pkt_queue ) );

    if( !pq ) {
      byebye( "buffer_add_reader" );
    } 

    pq->next = queue_map;
    queue_map = pq;

    pq->key = vm;
    pq->head = buffer_head;
    num_bots++;

    for( tmp = buffer_head; tmp; tmp = tmp->next ) {
      tmp->ref_count++;
    }
  }
}


static void safe_read( int f, void *data, int len ) {
  int i;

  for( i = 0; len > 0; len -= i ) {
    i = read( f, data, len );
    if( i < 0 ) {
      if( ( errno == EAGAIN ) || ( errno == EINTR ) ) {
        i = 0;
      } else {
        byebye( "read()" );
      }
    } else if( i == 0 ) {
      byebye( NULL );
    }
    data += i;
  }
}


static aseba_pkt *pkt_alloc( int len ) {
  aseba_pkt *pkt = NULL;
  aseba_pkt *tmp;
  
  if( len < ASEBA_MAX_PACKET_SIZE ) {
    len = ASEBA_MAX_PACKET_SIZE;
  }

  if( free_list ) {
    if( free_list->size >= len ) {
      pkt = free_list;
      free_list = free_list->next;
    } else {
      for(tmp = free_list; tmp->next && tmp->next->size < len; tmp = tmp->next);
      if( tmp->next ) {
        pkt = tmp->next;
        tmp->next = tmp->next->next;
      }
    }
  }
  
  if( !pkt ) {
    pkt = malloc( sizeof( aseba_pkt ) + len );
    if( !pkt ) {
      byebye( "pkt_alloc()" );
    }
    memset( pkt, 0, sizeof( aseba_pkt ) + len );
    pkt->size = len;
  } else {
    pkt->next = NULL;
  }

  pkt->ref_count = num_bots + 1;
  if( !buffer_head ) {
    buffer_head = pkt;
  } else {
    buffer_tail->next = pkt;
  }
  buffer_tail = pkt;

  return pkt;
}

  
extern void buffer_load( void ) {
  uint16_t temp;
  int len;
  aseba_pkt *pkt;

  safe_read( sock, &temp, 2 );
  len = bswap16( temp ) + 2;
  pkt = pkt_alloc( len );
  replay_append( &temp, 2 );

  safe_read( sock, &temp, 2 );
  pkt->source = bswap16( temp );
  replay_append( &temp, 2 );
  
  safe_read( sock, pkt->data, len );
  pkt->len = (uint16_t)len;
  replay_append( pkt->data, len );
  replay_checkpoint();
}


static void put_buffer( uint16_t length, uint16_t source, const uint8_t *data ) {
  aseba_pkt *pkt = pkt_alloc( length );
  pkt->source = source;
  memcpy( pkt->data, data, length );
  pkt->len = length;
}

static void clean() {
  aseba_pkt *tmp;
  
  while( buffer_head && !buffer_head->ref_count ) {
    tmp = buffer_head;
    buffer_head = buffer_head->next;
    tmp->next = free_list;
    free_list = tmp;
  }

  if( !buffer_head ) {
    buffer_tail = NULL;
  }
}


extern uint16_t buffer_recv( AsebaVMState *vm, uint8_t* data, uint16_t maxlength, 
                              uint16_t* source ) {
  pkt_queue *pq = get_packet_queue( vm );

  if( !pq ) {
    return 0;
  }

  clean();
  if( !pq->head ) {
    pq->head = buffer_head;
  } else {
    pq->head->ref_count--;
    pq->head = pq->head->next;
  }

  /* ignore messages to self && OB messages*/
  while( pq->head && ( ( pq->head->source == vm->nodeId ) || 
                       IS_OB_PACKET( pq->head->data ) ) ) {
    pq->head->ref_count--;
    pq->head = pq->head->next;
  }

  if( !pq->head ) {
    buffer_load();
    logger_printf( "Unexpected buffer_load().  Should NEVER happen." );
    pq->head = buffer_tail;
    if( ( pq->head->source == vm->nodeId ) || IS_OB_PACKET( pq->head->data ) ) {
      return 0;
    }
  }

  if( source ) {
    *source = pq->head->source;
  }
  
  if( pq->head->len < maxlength ) {
    maxlength = pq->head->len;
  }
  memcpy( data, pq->head->data, maxlength );
  return maxlength;
}


extern void buffer_ob_send( uint16_t source, const uint8_t* data, uint16_t length ) {
  uint16_t temp;

  if( !allow_ob ) {
    return;
  }

  temp = bswap16( length );
  if( write( sock, &temp, 2 ) != 2 ) {
    byebye( "write()" );
  }

  temp = bswap16( source );
  if( write( sock, &temp, 2 ) != 2 ) {
    byebye( "write()" );
  }

  temp = 0xffff; /* Invalid msg type, intercepted by visualizer */
  if( write( sock, &temp, 2 ) != 2 ) {
    byebye( "write()" );
  }

  if( write( sock, data, length ) != length ) {
    byebye( "write()" );
  }
}


extern void buffer_send( AsebaVMState *vm, const uint8_t* data, uint16_t length ) {
  uint16_t temp;

  temp = bswap16( length - 2 );
  if( write( sock, &temp, 2 ) != 2 ) {
    byebye( "write()" );
  }

  temp = bswap16( vm->nodeId );
  if( write( sock, &temp, 2 ) != 2 ) {
    byebye( "write()" );
  }

  if( write( sock, data, length ) != length ) {
    byebye( "write()" );
  }

  put_buffer( length, vm->nodeId, data );
}


int buffer_remaining( AsebaVMState *vm ) {
  pkt_queue *pq = get_packet_queue( vm );
  aseba_pkt *p = pq->head;
  int count = 0;

  if( !p ) {
    p = buffer_head;
  } else {
    p = p->next;
  }

  for( ; p; p = p->next ) {
    if( ( p->source != vm->nodeId ) && ( p->ref_count > 0 ) &&
        !IS_OB_PACKET( p->data ) ) {
      count++;
    }
  }

  return count;
}


extern uint16_t buffer_ob_recv( uint8_t* data, uint16_t maxlength, 
                                uint16_t* source ) {
  pkt_queue *pq = &ob_q;

  clean();
  if( !pq->head && !buffer_head ) {
    return 0;
  } else if( !pq->head ) {
    pq->head = buffer_head;
  } else if( pq->head->next ) {
    pq->head->ref_count--;
    pq->head = pq->head->next;
  } else {
    return 0;
  }

  /* ignore regular messages */
  while( !IS_OB_PACKET( pq->head->data ) ) {
    if( pq->head->next ) {
      pq->head->ref_count--;
      pq->head = pq->head->next;
    } else {
      return 0;
    }
  }

  if( source ) {
    *source = pq->head->source;
  }
  
  pq->head->len -= 2; /* no need to copy type */
  if( pq->head->len < maxlength ) {
    maxlength = pq->head->len;
  }
  memcpy( data, pq->head->data + 2, maxlength );

  replay_flush(); /* flush in case this payload causes a crash */
  return maxlength;
}
