/*
 * commander.h
 *
 *  Created on: May 15, 2022
 *      Author: mined
 */

#ifndef INC_COMMANDER_COMMANDER_H_
#define INC_COMMANDER_COMMANDER_H_

typedef struct ecu_command {
	uint8_t start_motor;
	uint8_t glow_plug;
	uint8_t valve;
	uint8_t pump;
}COMMAND;

typedef union command_packet{
	COMMAND cmd;
	uint8_t byte_array[sizeof(COMMAND)];
}COMM_PACK;

COMMAND readCommand();
void writeStatuse();

#endif /* INC_COMMANDER_COMMANDER_H_ */
