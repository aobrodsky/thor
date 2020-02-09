/*
        Thymio-II Firmware

        Copyright (C) 2011 Philippe Retornaz <philippe dot retornaz at epfl dot ch>,
        Mobots group (http://mobots.epfl.ch), Robotics system laboratory (http://lsro.epfl.ch)
        EPFL Ecole polytechnique federale de Lausanne (http://www.epfl.ch)

        See authors.txt for more details about other contributors.

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU Lesser General Public License as published
        by the Free Software Foundation, version 3 of the License.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU Lesser General Public License for more details.

        You should have received a copy of the GNU Lesser General Public License
        along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

// #include <types/types.h>

#include "thymio.h"
#include "thysim.h"

#define WAVEFORM_SIZE 142

AsebaNativeFunctionDescription AsebaNativeDescription_set_led = {
  "_leds.set",
  "Set the led",
  {
    {1,"led"},
    {1,"brightness"},
    {0,0}
  }
};

void set_led(AsebaVMState *vm) { /* NYI: just load from variables */
  AsebaNativePopArg( vm );
  AsebaNativePopArg( vm );
}

AsebaNativeFunctionDescription AsebaNativeDescription_record = {
  "sound.record",
  "Start recording of rN.wav",
  {
    {1,"N"},
    {0,0},
  }
};

AsebaNativeFunctionDescription AsebaNativeDescription_play = {
  "sound.play",
  "Start playback of pN.wav",
  {
    {1,"N"},
    {0,0},
  }
};


AsebaNativeFunctionDescription AsebaNativeDescription_replay = {
  "sound.replay",
  "Start playback of rN.wav",
  {
    {1,"N"},
    {0,0},
  }
};

AsebaNativeFunctionDescription AsebaNativeDescription_sound_system = {
  "sound.system",
  "Start playback of system sound N",
  {
    {1,"N"},
    {0,0},
  }
};


void sound_playback(AsebaVMState *vm) {/* NYI: just load from variables */
  AsebaNativePopArg( vm );
}

void sound_record(AsebaVMState *vm) {
  AsebaNativePopArg( vm );
}

void sound_replay(AsebaVMState *vm) {
  AsebaNativePopArg( vm );
}

void sound_system(AsebaVMState *vm) {
  AsebaNativePopArg( vm );
}

AsebaNativeFunctionDescription AsebaNativeDescription_set_led_circle = {
  "leds.circle",
  "Set circular ring leds",
  {
    {1,"led 0"},
    {1,"led 1"},
    {1,"led 2"},
    {1,"led 3"},
    {1,"led 4"},
    {1,"led 5"},
    {1,"led 6"},
    {1,"led 7"},
    {0,0},
  }
};

void set_led_circle(AsebaVMState *vm) {
  int16_t c[8];
  int i;

  for( i = 0; i < 8; i++ ) {
    c[i] = vm->variables[AsebaNativePopArg( vm )];
  }
  thy_sim_set_leds_circle( vm, c );
}

AsebaNativeFunctionDescription AsebaNativeDescription_set_led_rgb_top = {
  "leds.top",
  "Set RGB top led",
  {
    {1,"red"},
    {1,"green"},
    {1,"blue"},
    {0,0},
  }
};

void set_rgb_top(AsebaVMState *vm) {
  int r = vm->variables[AsebaNativePopArg( vm )];
  int g = vm->variables[AsebaNativePopArg( vm )];
  int b = vm->variables[AsebaNativePopArg( vm )];

  thy_sim_set_leds_top( vm, r, g, b );
}

AsebaNativeFunctionDescription AsebaNativeDescription_set_led_rgb_br = {
  "leds.bottom.right",
  "Set RGB botom right led",
  {
    {1,"red"},
    {1,"green"},
    {1,"blue"},
    {0,0},
  }
};

void set_rgb_br(AsebaVMState *vm) {
  int r = vm->variables[AsebaNativePopArg( vm )];
  int g = vm->variables[AsebaNativePopArg( vm )];
  int b = vm->variables[AsebaNativePopArg( vm )];

  thy_sim_set_leds_bot_right( vm, r, g, b );
}

AsebaNativeFunctionDescription AsebaNativeDescription_set_led_rgb_bl = {
  "leds.bottom.left",
  "Set RGB botom left led",
  {
    {1,"red"},
    {1,"green"},
    {1,"blue"},
    {0,0},
  }
};

void set_rgb_bl(AsebaVMState *vm) {
  int r = vm->variables[AsebaNativePopArg( vm )];
  int g = vm->variables[AsebaNativePopArg( vm )];
  int b = vm->variables[AsebaNativePopArg( vm )];

  thy_sim_set_leds_bot_left( vm, r, g, b );
}

AsebaNativeFunctionDescription AsebaNativeDescription_set_led_buttons = {
  "leds.buttons",
  "Set buttons leds",
  {
    {1,"led 0"},
    {1,"led 1"},
    {1,"led 2"},
    {1,"led 3"},
    {0,0},
  }
};

void set_buttons_leds(AsebaVMState *vm) {
  int t = vm->variables[AsebaNativePopArg( vm )];
  int r = vm->variables[AsebaNativePopArg( vm )];
  int b = vm->variables[AsebaNativePopArg( vm )];
  int l = vm->variables[AsebaNativePopArg( vm )];

  thy_sim_set_leds_buttons( vm, t, r, b, l );
}

AsebaNativeFunctionDescription AsebaNativeDescription_set_hprox_leds = {
  "leds.prox.h",
  "Set horizontal proximity leds",
  {
    {1,"led 0"},
    {1,"led 1"},
    {1,"led 2"},
    {1,"led 3"},
    {1,"led 4"},
    {1,"led 5"},
    {1,"led 6"},
    {1,"led 7"},
    {0,0},
  }
};

void set_hprox_leds(AsebaVMState *vm) {
  int16_t c[8];
  int i;

  for( i = 0; i < 8; i++ ) {
    c[i] = vm->variables[AsebaNativePopArg( vm )];
  }
  thy_sim_set_leds_prox_horiz( vm, c );
}

AsebaNativeFunctionDescription AsebaNativeDescription_set_vprox_leds = {
  "leds.prox.v",
  "Set vertical proximity leds",
  {
    {1,"led 0"},
    {1,"led 1"},
    {0,0},
  }
};

void set_vprox_leds(AsebaVMState *vm) {
  int l = vm->variables[AsebaNativePopArg( vm )];
  int r = vm->variables[AsebaNativePopArg( vm )];

  thy_sim_set_leds_prox_grnd( vm, l, r );
}


AsebaNativeFunctionDescription AsebaNativeDescription_set_rc_leds = {
  "leds.rc",
  "Set rc led",
  {
    {1,"led"},
    {0,0},
  }
};

void set_rc_leds(AsebaVMState *vm) {
  thy_sim_set_leds_rc( vm, vm->variables[AsebaNativePopArg( vm )] );
}


AsebaNativeFunctionDescription AsebaNativeDescription_set_sound_leds = {
  "leds.sound",
  "Set sound led",
  {
    {1,"led"},
    {0,0},
  }
};


void set_sound_leds(AsebaVMState *vm) {
  thy_sim_set_leds_sound( vm, vm->variables[AsebaNativePopArg( vm )] );
}  


AsebaNativeFunctionDescription AsebaNativeDescription_set_ntc_leds = {
  "leds.temperature",
  "Set ntc led",
  {
    {1,"red"},
    {1,"blue"},
    {0,0},
  }
};


void set_ntc_leds(AsebaVMState *vm) {
  int r = vm->variables[AsebaNativePopArg( vm )];
  int b = vm->variables[AsebaNativePopArg( vm )];

  thy_sim_set_leds_temp( vm, r, b );
}  


AsebaNativeFunctionDescription AsebaNativeDescription_play_freq = {
  "sound.freq",
  "Play frequency",
  {
    {1,"Hz"},
    {1,"ds"},
    {0,0},
  }
};

void play_freq(AsebaVMState * vm) {
  AsebaNativePopArg( vm );
  AsebaNativePopArg( vm );
}

AsebaNativeFunctionDescription AsebaNativeDescription_set_wave = {
  "sound.wave",
  "Set the primary wave of the tone generator",
  {
    {WAVEFORM_SIZE, "wave"},
    {0,0},
  }
};

void set_wave( AsebaVMState * vm ) {
  AsebaNativePopArg( vm );
}

AsebaNativeFunctionDescription AsebaNativeDescription_prox_network = {
  "prox.comm.enable",
  "Enable or disable the proximity communication",
  {
    {1, "state"},
    {0,0},
  }
};
                
void prox_network(AsebaVMState * vm) {
  AsebaNativePopArg(vm);
}

AsebaNativeFunctionDescription AsebaNativeDescription_sd_open = {
  "sd.open",
  "Open a file on the SD card",
  {
    {1, "number"},
    {1, "status"},
    {0,0},
  }
};

void thymio_native_sd_open(AsebaVMState * vm) {
  AsebaNativePopArg(vm);
  AsebaNativePopArg(vm);
}

AsebaNativeFunctionDescription AsebaNativeDescription_sd_write = {
  "sd.write",
  "Write data to the opened file",
  {
    {-1, "data"},
    {1, "written"},
    {0,0},
  }
};


void thymio_native_sd_write(AsebaVMState * vm) {
  // variable pos
  AsebaNativePopArg(vm);
  AsebaNativePopArg(vm);
  AsebaNativePopArg(vm);
}               
   
     
AsebaNativeFunctionDescription AsebaNativeDescription_sd_read = {
  "sd.read",    
  "Read data from the opened file",
  {             
    {-1, "data"},
    {1, "read"},
    {0,0},      
  }     
};


void thymio_native_sd_read(AsebaVMState * vm) {
  AsebaNativePopArg(vm);
  AsebaNativePopArg(vm);
  AsebaNativePopArg(vm);
}


AsebaNativeFunctionDescription AsebaNativeDescription_sd_seek = {
  "sd.seek",
  "Seek the opened file",
  {
    {1, "position"},
    {1, "status"},
    {0,0},
  }
};


void thymio_native_sd_seek(AsebaVMState * vm) {
  AsebaNativePopArg(vm);
  AsebaNativePopArg(vm);
}


AsebaNativeFunctionDescription AsebaNativeDescription_rf_nodeid = {
  "_rf.nodeid",
  "Set Wireless Node ID",
  {
    {1, "nodeID"},
    {0,0},
  }
};


void set_rf_nodeid(AsebaVMState * vm) {
  AsebaNativePopArg(vm);
}
