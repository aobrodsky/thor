#ifndef BUFFER_H
#define BUFFER_H

#include "../../vm/vm.h"
#include "../../vm/natives.h"
#include "../../common/productids.h"
#include "../../common/consts.h"
#include "../../transport/buffer//vm-buffer.h"

#define IS_OB_PACKET(d) (*(short*)(d) == (short)ASEBA_MESSAGE_INVALID)

extern void buffer_init( int sck, int allow_ob_send );
extern void buffer_send(AsebaVMState *vm, const uint8_t* data, uint16_t length);
extern void buffer_ob_send( uint16_t source, const uint8_t* data, uint16_t len);
extern void buffer_add_reader( AsebaVMState *vm );
extern uint16_t buffer_recv( AsebaVMState *vm, uint8_t* data, uint16_t maxlen, 
                             uint16_t* source );
extern uint16_t buffer_ob_recv( uint8_t* data, uint16_t maxlen, uint16_t* src );
extern int buffer_remaining( AsebaVMState *vm );
extern void buffer_load();

#endif
