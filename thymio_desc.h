#ifndef THYMIO_DESC_H
#define THYMIO_DESC_H

#include <string.h>

#include "../../vm/natives.h"
#include "../../vm/vm.h"
#include "../../common/consts.h"
#include "../../common/types.h"
#include "../../transport/buffer/vm-buffer.h"

#include "thymio.h"

#define PRODUCT_ID ASEBA_PID_THYMIO2
#define FIRMWARE_VER_LOW 10
#define FIRMWARE_VER_HIGH 0

/* This is the number of "private" variable the aseba script can have */
#define VM_VARIABLES_FREE_SPACE 512

/* THis is the maximum number of argument an aseba event can recieve */
#define VM_VARIABLES_ARG_SIZE 32

/* THE nuber of opcode an aseba script can have */
#define VM_BYTECODE_SIZE (766+768)  // Put here 766+768*a, where a is >= 0
#define VM_STACK_SIZE 32


struct _vmVariables {
  // NodeID
  int16_t id;
  // source
  int16_t source;
  // args
  int16_t args[VM_VARIABLES_ARG_SIZE];
  // fwversion
  int16_t fwversion[2];
  // Product ID
  int16_t productid;

  
  int16_t buttons[5];
  
  int16_t buttons_state[5];

  int16_t buttons_mean[5];
  int16_t buttons_noise[5];
  
  int16_t prox[7];
  int16_t sensor_data[7];
  int16_t intensity[7];
  int16_t rx_data;
  int16_t ir_tx_data;

  int16_t ground_ambiant[2];
  int16_t ground_reflected[2];
  int16_t ground_delta[2];
  
  int16_t target[2];
  
  int16_t vbat[2];
  int16_t imot[2];
  
  int16_t uind[2];
  int16_t pwm[2];
  
  int16_t acc[3];
  
  int16_t ntc;
  
  int16_t rc5_address;
  int16_t rc5_command;
  
  int16_t sound_level;
  int16_t sound_tresh;
  int16_t sound_mean;
  
  int16_t timers[2];
  
  int16_t acc_tap;
  
  /*****
  ---> PUT YOUR VARIABLES HERE <---
  ******/
  
  int16_t freeSpace[VM_VARIABLES_FREE_SPACE];
};


enum Event
{
  EVENT_B_BACKWARD = 0,
  EVENT_B_LEFT,
  EVENT_B_CENTER,
  EVENT_B_FORWARD,
  EVENT_B_RIGHT,
  EVENT_BUTTONS,
  EVENT_PROX,
  EVENT_DATA,
  EVENT_TAP,
  EVENT_ACC,
  EVENT_MIC,
  EVENT_SOUND_FINISHED,
  EVENT_TEMPERATURE,
  EVENT_RC5,
  EVENT_MOTOR,
  // Must be consecutive
  EVENT_TIMER0,
  EVENT_TIMER1,
  // We have now 16 events.
  // this is the maximum. if you want to increase it
  // you have to change skel-usb.c 
  EVENT_COUNT // Do not touch
};

extern const AsebaLocalEventDescription * 
       ThymioGetLocalEventsDescriptions( AsebaVMState *vma );
extern void ThymioNativeFunction( AsebaVMState * vm, uint16_t id );
extern const AsebaVMDescription * ThymioGetVMDescription( AsebaVMState *vm );
extern const AsebaNativeFunctionDescription * const * 
                        ThymioGetNativeFunctionsDescriptions(AsebaVMState *vm );
#endif
