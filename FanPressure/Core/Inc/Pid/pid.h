/*
 * pid.h

 *
 *  Created on: 2023/11/17
 *      Author: xli185
 */
#ifndef __PID_H_
#define __PID_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>
#include <string.h>
#include "main.h"
//#include "stm32h4xx_hal.h"

#define ERROR_HISTORY_LENGTH 10
/* --------------------------------------------------------- */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* PID Mode */
typedef enum
{
	PID_MODE_MANUAL    = 0,
	PID_MODE_AUTOMATIC = 1

}PIDMode_TypeDef;

/* PID P On x */
typedef enum
{
	PID_P_ON_M = 0, /* Proportional on Measurement */
	PID_P_ON_E = 1
	
}PIDPON_TypeDef;

/* PID Control direction */
typedef enum
{
	PID_CD_DIRECT  = 0,
	PID_CD_REVERSE = 1

}PidControlDirection_TypeDef;

template<typename T>
class PID
{
	
private:
	
//	PIDPON_TypeDef  _pOnE;
//	PIDMode_TypeDef mode_;

//	PIDPON_TypeDef  _pOn;

	//const int error_history_length = 10;

	uint8_t add_input_error_to_proportional_;

//	PIDCD_TypeDef   _controllerDirection;

	uint32_t        last_time_;

//	T          _dispKp;
//	T          _dispKi;
//	T          _dispKd;

	T          kp_;
	T          ki_;
	T          kd_;

	T          *input_;
	T          *output_;
	T          *setpoint_;

	T          output_sum_;
	T          last_input_;

	T          out_min_;
	T          out_max_;
	
	T error_history[ERROR_HISTORY_LENGTH];
	int error_index = 0;

	public:
	
	/* :::::::::: Constructor :::::::::: */
	PID();
	PID(T *input, T *output, T *setpoint, T kp, T ki, T kd, uint8_t add_input = 0);
//	PID(T *Input, T *Output, T *Setpoint, T Kp, T Ki, T Kd, PIDCD_TypeDef ControllerDirection);

	/* :::::::::::::: Init ::::::::::::: */
//	void Init(void);
//	void Init(T *Input, T *Output, T *Setpoint, T Kp, T Ki, T Kd, PIDPON_TypeDef POn, PIDCD_TypeDef ControllerDirection);
//	void Init(T *Input, T *Output, T *Setpoint, T Kp, T Ki, T Kd, PIDCD_TypeDef ControllerDirection);

	/* ::::::::::: Computing ::::::::::: */
	void compute(void);

	/* ::::::::::: PID Mode :::::::::::: */
//	void            Set_Mode(PIDMode_TypeDef Mode);
//	PIDMode_TypeDef GetMode(void);

	/* :::::::::: PID Limits ::::::::::: */
	void set_output_limits(T min, T max);

	/* :::::::::: PID Tunings :::::::::: */
//	void Set_Tunings(T Kp, T Ki, T Kd);
	void set_tunings(T kp, T ki, T kd, uint8_t add_input = 0);

	void reset();

	/* ::::::::: PID Direction ::::::::: */
//	void          Set_Controller_Direction(PIDCD_TypeDef Direction);
//	PIDCD_TypeDef GetDirection(void);

	/* ::::::::: PID Sampling :::::::::: */
//	void SetSampleTime(int32_t NewSampleTime);

//	/* ::::::: Get Tunings Param ::::::: */
//	T GetKp(void);
//	T GetKi(void);
//	T GetKd(void);
	
};

template<typename T>
void clip(T* value,T min,T max){
	if(*value>max)*value=max;
	else if(*value<min)*value=min;
}

//template void clip<float>(float* value,float min,float max);

#endif /* __PID_H_ */
