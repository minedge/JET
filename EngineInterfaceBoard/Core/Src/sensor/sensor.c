/*
 * sensor.c
 *
 *  Created on: May 21, 2022
 *      Author: mined
 */


#include "sensor/sensor.h"

ENGINE_STATE getState(){
	ENGINE_STATE state;

	state.temperature = readCelsius();

	return state;
}
