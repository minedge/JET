/*
 * DFR0558.c
 *
 *  Created on: May 20, 2022
 *      Author: mined
 */
#include "sensor/DFR0558.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;

unsigned char rxbuf[4] = {0};

float readCelsius(){

  HAL_I2C_Mem_Read(&hi2c1, MAX31855_Addr, 0x00, I2C_MEMADD_SIZE_8BIT, rxbuf, 4, HAL_TIMEOUT);

  if(rxbuf[3]&0x7){
	return 0;
  }

  if(rxbuf[0]&0x80){
	rxbuf[0] = 0xff - rxbuf[0];
	rxbuf[1] = 0xff - rxbuf[1];
	float temp =  -((((rxbuf[0] << 8)|(rxbuf[1] & 0xfc)) >> 2) + 1) * 0.25;
	return temp;
  }

  float temp =(((rxbuf[0] << 8 )| (rxbuf[1] & 0xfc)) >> 2)*0.25;

  return temp;
}
