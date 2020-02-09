#ifndef THY_STATE_H
#define THY_STATE_H

#include "thysim.h"
#include "world.h"
#include "bot_err.h"

/* Constants */
#define NUM_WHEELS 2
#define LEFT_WHEEL 0
#define RIGHT_WHEEL 1
#define NUM_PROX_HORIZ 7
#define NUM_PROX_GRND  2
#define NUM_BUTTONS 5
#define AMB_LIGHT 0
#define MAX_LIGHT 1023
#define TABLE_LIGHT 850

enum {
  LEFT_MOTOR_BIAS,
  RIGHT_MOTOR_BIAS,
  LEFT_MOTOR_VAR,
  RIGHT_MOTOR_VAR,
  GROUND_SENSOR_BIAS,
  GROUND_SENSOR_VAR,
  HORIZ_SENSOR_BIAS,
  HORIZ_SENSOR_VAR,
  NUM_NOISE
};

#define ARC_REP_GRAN 10  /* we represent arcs by 1 line per 10 degs */
/*                       corners   front arc degs   rep by 10 deg lines */
#define NUM_THY_VERTICES  ( 4 + ( ( 45 - ( -45 ) ) / ARC_REP_GRAN ) - 1)
#define NUM_THY_CONTACTS  3 /* left wheel, right wheel, front pivot */

#define MAX_SPEED ( (double) 200 / 1000000 ) /* mm per usec */
#define MAX_SETTING 500 /* max motor setting */
#define SPEED_CONV ((double) MAX_SPEED / MAX_SETTING )

#define MAX_PROX_DIST 100 /* mm */
#define PROX_MAX_VAL 5000
#define PROX_C_PARAM 2500
#define PROX_VAL(d,c,m) ( (double) ( (m) * (c) ) / ( ( (d) * (d) ) + (c) ) )

#define NUM_CIRCLE_LEDS 8
#define NUM_PROX_LEDS   8 

#define MIN_SEND_INTERVAL  40000 /* 1/25th of a second */
#define MIN_POLL_INTERVAL 100000 /* 1/10th of a second */
#define HEARTBEAT_INTERVAL 1000000 /* 1 second */

#define PATH_WIDTH 9 /* 9 mm on each side, tape width = 18mm */
#define OBSTACLE_COLOUR 256 /* In ver 1, obstacle colour is fixed. */
#define ROBOT_COLOUR 1023 /* maximum response of sensors */

/* thymio state */
struct thymio_state {
  int16_t ver;
  int16_t id;
  int16_t error;
  int16_t model;
  int16_t target[NUM_WHEELS];
  int16_t speed[NUM_WHEELS];
  tw_vect position;
  tw_angle direction;
  int16_t button_state[NUM_BUTTONS];
  int16_t tap;
  int16_t circle_leds[NUM_CIRCLE_LEDS];
  int16_t leds_top[3];
  int16_t leds_bot_left[3];
  int16_t leds_bot_right[3];
  int16_t leds_prox[NUM_PROX_LEDS];
  int16_t leds_prox_grnd[2];
  int16_t leds_buttons[4];
  int16_t leds_rc;
  int16_t leds_temp[2];
  int16_t leds_sound;

  tw_angle noise[NUM_NOISE]; /* never sent */
};

#endif
