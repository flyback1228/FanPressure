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

typedef  struct  {
	char header[3];
	//bit 0: start; bit 1: bk front panel lock; bit 2: read ads1256; bit 3: control mode;
	uint8_t bool_register;

	//XX.X
    //uint16_t bk1697_set_voltage;
	uint16_t voltage_range;
    uint16_t fan_speed_setpoint;

    uint16_t voltage;
    uint16_t backup;


    char tailer[4];
} Command_t;

typedef  struct  {
	char header[4];
	float kp;
	float ki;
	float kd;
    uint32_t fan_speed_half;
    uint32_t fan_speed_full;
    char tailer[4];
} Parameter_t;


typedef  struct  {
	char header[2];
	//bit 0: bk connected; bit 1: request a new command
    uint8_t bool_register;
    uint8_t error_code;

    //XX.X
    uint16_t bk1697_voltage;

    //X.XX
    uint16_t bk1697_current;

    //X.XXX
    int16_t ads1256_channel_1;
    int16_t ads1256_channel_2;
    int16_t ads1256_channel_3;
    int16_t ads1256_channel_4;


    uint16_t fan_voltage;
    uint16_t fan_current;
    uint16_t fan_power;
    uint16_t fan_shunt_voltage;


    uint16_t fan_speed;
    uint16_t pwm;
    float kp;
    float ki;
    float kd;
    uint32_t speed_50;
    uint32_t speed_100;
    char tailer[4];
} Sensor_t;

#define DEFAULT_SENSOR { {'S','U'}, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0f,0.0f,0.0f,0,0,{'C','O','E','\n'} }
#define DEFAULT_COMMAND { {'S','Y','R'}, 0,0,0,0,0,{'C','O','E','\n'} }
#define DEFAULT_PARAMETER { {'C','O','E','\n'}, 0.0f,0.0f,0.0f,0,0, {'S','Y','R','\n'}}

void setup();

void loop();

#ifdef __cplusplus
}
#endif

#endif
