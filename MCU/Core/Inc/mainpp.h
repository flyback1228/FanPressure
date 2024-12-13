#ifndef _MAINPP_H_
#define _MAINPP_H_

#include "main.h"


#ifdef __cplusplus
 extern "C" {
#endif

typedef enum {
 LOCK_SCREEN = 0,
 UNLOCK_SCREEN,
 SET_VOLTAGE,
 INQUIRE_DATA,
 NONE
} BK1697_Lable;

typedef  struct __attribute__((packed)) {
	char header[3];
    uint8_t bk1697_screen_lock;
    float bk1697_setpoint;
    char tailer[3];
} Command_t;


typedef  struct __attribute__((packed)) {
	char header[3];
    uint8_t bk1697_screen_locked;
    float bk1697_voltage;
    float bk1697_current;
    char tailer[3];
} Sensor_t;

#define DEFAULT_COMMAND { {'S','Y','R'}, 0,0.0,{'C','O','E'} }
#define DEFAULT_SENSOR { {'S','Y','R'}, 0,0.0,0.0,{'C','O','E'} }

void setup();

void loop();

#ifdef __cplusplus
}
#endif

#endif
