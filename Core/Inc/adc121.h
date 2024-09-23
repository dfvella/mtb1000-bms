#ifndef __ADC121_H__
#define __ADC121_H__

#include "stm32l0xx_hal.h"

#define ADC121_I2C_ADDR 0x55

#define ADC121_REG_RES 0x00
#define ADC121_REG_STS 0x01
#define ADC121_REG_CFG 0x02

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint32_t timeout_ms;
	uint16_t data;
} ADC121_inst_S;

HAL_StatusTypeDef ADC121_init(ADC121_inst_S * inst, I2C_HandleTypeDef *hi2c, uint32_t timeout_ms);
HAL_StatusTypeDef ADC121_update(ADC121_inst_S *inst);
uint16_t ADC121_read(ADC121_inst_S *inst);
HAL_StatusTypeDef ADC121_shutdown(ADC121_inst_S *inst);

#endif // __ADC121_H__
