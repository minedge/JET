/*
 * output.h
 *
 *  Created on: May 15, 2022
 *      Author: mined
 */

#ifndef INC_OUTPUT_OUTPUT_H_
#define INC_OUTPUT_OUTPUT_H_

typedef struct output_data{
	unsigned int start_motor_pwm;
	uint8_t glow_plug;
	uint8_t valve;
	unsigned int pump_pwm;
} OUTPUT;

void setOutput(OUTPUT out);

#endif /* INC_OUTPUT_OUTPUT_H_ */
