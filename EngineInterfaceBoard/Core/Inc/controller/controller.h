/*
 * controller.h
 *
 *  Created on: May 15, 2022
 *      Author: mined
 */

#ifndef INC_CONTROLLER_CONTROLLER_H_
#define INC_CONTROLLER_CONTROLLER_H_

#include "message/message.h"

enum _pid_modes{
	P = 1,
	PI,
	PD,
	PID
};

typedef struct _pid_controller_gain{
	float p;
	float i;
	float d;
}PID_GAIN;

typedef struct _pid_controller_block{
	int mode;

	PID_GAIN k;

	float pre_error;
	float integrated_error;

	float loop_time;
	float loop_start;
}PID_BLOCK;

extern volatile unsigned long gTick;

//PID_BLOCK pid_list[20];

int regPID(int mode, float gain_p, float gain_i, float gain_d);

float PIDoutput(int id, float state, float target);

void resetState(int id);

#endif /* INC_CONTROLLER_CONTROLLER_H_ */
