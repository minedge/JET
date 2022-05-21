/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RC/spektrum.h"
#include "Sensor/as5147.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct _telemetry_TX_data_structure{
	uint8_t header;
	float angle;
	float mode;
	float throttle;
}Telem_struct;

typedef struct _telemetry_RX_data_structure{
	uint8_t header;
	uint8_t comm;
}Comm_struct;

typedef union _telemetry_data_union{
	uint8_t array[sizeof(Telem_struct)];
	Telem_struct tx;
	Comm_struct rx;
}TELEM;


typedef enum{
	ECU_OK			= 0x00,
	ECU_STANDBY		= 0x01,
	ECU_READY		= 0x02,
	ECU_IGNITE		= 0x03,
	ECU_RUNNING		= 0x04,
	ECU_COLLING		= 0x05,
	ECU_Error		= 0x06
}ECU_Mode;

typedef struct _engine_sensor_state{
	MAGNET mag;
	float temperature;
}Sensors;

typedef struct _engine_output_state{
	uint8_t glow_plug;
	float fuel_pump_pwm;
	float start_motor_pwm;
}OUTPUT;

typedef struct _engine_control_state{
	uint8_t safty_arm;

	ECU_Mode state;

	Sensors sensor;

	OUTPUT out;
}ECS;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SPI2_CS_Pin GPIO_PIN_10
#define SPI2_CS_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
