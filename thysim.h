#ifndef THY_SIM_H
#define THY_SIM_H

#include "world.h"
#include "robot.h"

#include "../../vm/natives.h"
#include "../../vm/vm.h"
#include "../../common/consts.h"
#include "../../common/types.h"
#include "../../transport/buffer/vm-buffer.h"

extern void thy_sim_init( robot_model *m );

extern void thy_sim_init_state( robot_state *rs );
extern void thy_sim_delete( robot_state *rs );
extern int thy_sim_set_position( robot_state *rs, tw_vect *pos, tw_angle dir );
extern void thy_sim_user_action( robot_state *rs, void *input );

extern int thy_sim_set_speed( robot_state *rs, int16_t *target );
extern int thy_sim_get_speed( robot_state *rs, int16_t *uind );
extern int thy_sim_update_pos( robot_state *rs, unsigned long dt );

extern int thy_sim_get_buttons( robot_state *rs, int16_t *buttons );
extern int thy_sim_get_prox_horiz( robot_state *rs, int16_t *horiz );
extern int thy_sim_get_prox_grnd( robot_state *rs, int16_t *amb, int16_t *ref, 
                           int16_t *delta  );
extern int thy_sim_get_acc( robot_state *rs, int16_t *acc );
extern int thy_sim_get_temp( robot_state *rs, int16_t *temp );
extern int thy_sim_tap( robot_state *rs );

extern int thy_sim_set_leds_circle( AsebaVMState *vm, int16_t *leds );
extern int thy_sim_set_leds_top( AsebaVMState *vm, int16_t r, int16_t g, 
                                 int16_t b );

extern int thy_sim_set_leds_bot_right( AsebaVMState *vm, int16_t r, int16_t g, 
                                       int16_t b);
extern int thy_sim_set_leds_bot_left( AsebaVMState *vm, int16_t r, int16_t g, 
                                      int16_t b );
extern int thy_sim_set_leds_prox_horiz( AsebaVMState *vm, int16_t *leds );
extern int thy_sim_set_leds_prox_grnd( AsebaVMState *vm, int16_t left, 
                                       int16_t right );
extern int thy_sim_set_leds_buttons( AsebaVMState *vm, int16_t top, 
                                     int16_t right, int16_t back, int16_t left);
extern int thy_sim_set_leds_rc( AsebaVMState *vm, int16_t led );
extern int thy_sim_set_leds_temp( AsebaVMState *vm, int16_t red, int16_t blue );
extern int thy_sim_set_leds_sound( AsebaVMState *vm, int16_t led );

#endif
