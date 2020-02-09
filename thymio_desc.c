#include <string.h>
#include <stdio.h>

#include "../../vm/natives.h"
#include "../../vm/vm.h"
#include "../../common/consts.h"
#include "../../common/types.h"
#include "../../transport/buffer/vm-buffer.h"

#include "thymio.h"
#include "thymio_desc.h"


// The content of this structure is implementation-specific
// The glue provide a way to store and retrive it from flash.
// The only way to write it is to do it from inside the VM (Native function)
// The native function access it as a integer array. So, use only int inside 
// this structure
struct private_settings {
  /* ADD here the settings to save into flash */
  /* The minimum size is one integer, the maximum size is 95 integer 
     (check done at compilation) */
  int sound_shift;
  int settings[89];
};
  

static struct private_settings __attribute__((aligned(2))) settings;

static AsebaNativeFunctionDescription AsebaNativeDescription__system_reboot = 
{
  "_system.reboot",
  "Reboot the microcontroller",
  {
    {0,0}
  }
};

static void AsebaNative__system_reboot(AsebaVMState *vm) {
  AsebaVMInit( vm );
}

static AsebaNativeFunctionDescription AsebaNativeDescription__system_settings_read = 
{
  "_system.settings.read",
  "Read a setting", 
  {
    {1, "address"},
    {1,  "value"},
    {0,0}
  }
};

static void AsebaNative__system_settings_read(AsebaVMState *vm) {
  uint16_t address = vm->variables[AsebaNativePopArg( vm )];
  uint16_t destidx = AsebaNativePopArg( vm );
  
  if( address > ( sizeof(settings)/2 - 1 ) ) {
    AsebaVMEmitNodeSpecificError( vm, "Address out of settings" );
    return;
  }

  vm->variables[destidx] = ((unsigned int *) &settings)[address];
}


static AsebaNativeFunctionDescription 
                                 AsebaNativeDescription__system_settings_write =
{
  "_system.settings.write",
  "Write a setting",
  {
    { 1, "address"},
    { 1, "value"},
    { 0, 0 }
  }
};


static void AsebaNative__system_settings_write( AsebaVMState *vm ) {
  uint16_t address = vm->variables[ AsebaNativePopArg( vm ) ];
  uint16_t sourceidx = AsebaNativePopArg( vm );

  if( address > ( sizeof(settings)/2 - 1 ) ) {
    AsebaVMEmitNodeSpecificError(vm, "Address out of settings");
    return;
  }
  ((unsigned int *) &settings)[address] = vm->variables[sourceidx];
}


static AsebaNativeFunctionDescription 
                                AsebaNativeDescription__system_settings_flash =
{
  "_system.settings.flash",
  "Write the settings into flash",
  {
    {0,0}
  }
};


static void AsebaNative__system_settings_flash(AsebaVMState *vm) {
}


/* Descriptors */

#include "thymio_natives.h"

static const AsebaVMDescription vmDescription = {
  "thymio-II", // Name of the microcontroller
  {
    {1, "_id"}, // Do not touch it
    {1, "event.source"}, // Nor this one
    {VM_VARIABLES_ARG_SIZE, "event.args"}, // neither this one
    {2, "_fwversion"}, // Do not event think about changing this one ...
    {1, "_productId"}, // Robot type
    
    {5, "buttons._raw"},
    {1, "button.backward"},
    {1, "button.left"},
    {1, "button.center"},
    {1, "button.forward"},
    {1, "button.right"},
      
    {5, "buttons._mean"},
    {5, "buttons._noise"},
    
    {7, "prox.horizontal"},

    {7, "prox.comm.rx._payloads"},
    {7, "prox.comm.rx._intensities"},
    {1, "prox.comm.rx"},
    {1, "prox.comm.tx"},

    {2, "prox.ground.ambiant"},
    {2, "prox.ground.reflected"},
    {2, "prox.ground.delta"},
    
    {1, "motor.left.target"},
    {1, "motor.right.target"},
    {2, "_vbat"},
    {2, "_imot"},
    {1, "motor.left.speed"},
    {1, "motor.right.speed"},
    {1, "motor.left.pwm"},
    {1, "motor.right.pwm"},
    
    {3, "acc"},
    
    {1, "temperature"},
    
    {1, "rc5.address"},
    {1, "rc5.command"},
    
    {1, "mic.intensity"},
    {1, "mic.threshold"},
    {1, "mic._mean"},
    
    {2, "timer.period"},
    
    {1, "acc._tap"},

    /******
     ---> PUT YOUR VARIABLES DESCRIPTIONS HERE <---
    first value is the number of element in the array (1 if not an array)
    second value is the name of the variable which will be displayed in aseba studio
    ******/

    {0, NULL} // Null terminated
  }
};


static const AsebaLocalEventDescription localEvents[] = {
  /*******
  ---> PUT YOUR EVENT DESCRIPTIONS HERE <---
  First value is event "name" (will be used as "onvent name" in asebastudio
  second value is the event description)
  *******/
  { "button.backward", "Backward button status changed"},
  { "button.left", "Left button status changed"},
  { "button.center", "Center button status changed"},
  { "button.forward", "Forward button status changed"},
  { "button.right", "Right button status changed"},  
  { "buttons", "Buttons values updated"},
  { "prox", "Proximity values updated"},
  { "prox.comm", "Data received on the proximity communication"},
  { "tap", "A tap is detected"},
  { "acc", "Accelerometer values updated"},
  { "mic", "Fired when microphone intensity is above threshold"},  
  { "sound.finished", "Fired when the playback of a user initiated sound is finished"},
  { "temperature", "Temperature value updated"},
  { "rc5", "RC5 message received"},
  { "motor", "Motor timer"},
  { "timer0", "Timer 0"},
  { "timer1", "Timer 1"},
  { NULL, NULL }
};


static const AsebaNativeFunctionDescription AsebaNativeDescription_poweroff = {
  "_poweroff",
  "Poweroff",
  {
   {0,0}
  }
};

static void power_off(AsebaVMState *vm) {
}


static const AsebaNativeFunctionDescription* nativeFunctionsDescription[] = {
  &AsebaNativeDescription__system_reboot,
  &AsebaNativeDescription__system_settings_read,
  &AsebaNativeDescription__system_settings_write,
  &AsebaNativeDescription__system_settings_flash,
  
  ASEBA_NATIVES_STD_DESCRIPTIONS,
  
  THYMIO_NATIVES_DESCRIPTIONS,
  
  &AsebaNativeDescription_poweroff,
  0       // null terminated
};


static AsebaNativeFunctionPointer nativeFunctions[] = {
  AsebaNative__system_reboot,
  AsebaNative__system_settings_read,
  AsebaNative__system_settings_write,
  AsebaNative__system_settings_flash,
  
  ASEBA_NATIVES_STD_FUNCTIONS,
  
  THYMIO_NATIVES_FUNCTIONS,
  
  power_off
};


const AsebaLocalEventDescription * 
       ThymioGetLocalEventsDescriptions( AsebaVMState *vma ) {
  return localEvents;
}


void ThymioNativeFunction( AsebaVMState * vm, uint16_t id ) {
  nativeFunctions[id]( vm );
}


const AsebaVMDescription * ThymioGetVMDescription( AsebaVMState *vm ) {
  return &vmDescription;
}


const AsebaNativeFunctionDescription * const * 
                        ThymioGetNativeFunctionsDescriptions(AsebaVMState *vm) {
  return nativeFunctionsDescription;
}
