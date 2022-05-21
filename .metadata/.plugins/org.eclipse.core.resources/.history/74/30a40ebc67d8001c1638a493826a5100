/*
 * message.h
 *
 *  Created on: May 15, 2022
 *      Author: mined
 */

#ifndef INC_MESSAGE_MESSAGE_H_
#define INC_MESSAGE_MESSAGE_H_

#include "sensor/sensor.h"
#include "output/output.h"
#include "commander/commander.h"

typedef struct message_manager_structure{
	uint8_t from;
	uint8_t request; // in or out
	uint8_t type; // which structure
	COMMAND cmd;
	ENGINE_STATE state;
	OUTPUT out;
} MMS;

MMS getMessage();
void putMessage();

#endif /* INC_MESSAGE_MESSAGE_H_ */
