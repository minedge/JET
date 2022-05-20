/*
 * sensor.h
 *
 *  Created on: May 15, 2022
 *      Author: mined
 */

#ifndef INC_SENSOR_SENSOR_H_
#define INC_SENSOR_SENSOR_H_

#include "sensor/DFR0558.h"
#include "sensor/WSH231.h"
#include "message/message.h"

typedef struct engine_state{
	float rpm;
	float temperature;
	float atmosphere_pressure;
}ENGINE_STATE;

typedef union engine_state_packet{
	ENGINE_STATE cmd;
	uint8_t byte_array[sizeof(ENGINE_STATE)];
}STATE_PACK;

#endif /* INC_SENSOR_SENSOR_H_ */