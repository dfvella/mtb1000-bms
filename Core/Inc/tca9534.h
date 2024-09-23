#ifndef __TCA9534_H__
#define __TCA9534_H__

#include "stm32l0xx_hal.h"

#define TCA9534_I2C_ADDR 0x20

#define TCA9534_REG_INP 0x00
#define TCA9534_REG_OUT 0x01
#define TCA9534_REG_POL 0x02
#define TCA9534_REG_CFG 0x03

typedef enum
{
	TCA9534_CHANNEL_1,
	TCA9534_CHANNEL_2,
	TCA9534_CHANNEL_3,
	TCA9534_CHANNEL_4,
	TCA9534_CHANNEL_5,
	TCA9534_CHANNEL_6,
	TCA9534_CHANNEL_7,
	TCA9534_CHANNEL_8,

	TCA9534_CHANNEL_COUNT
} TCA9534_channel_E;

typedef enum
{
	TCA9534_OUTPUT = 0,
	TCA9534_INPUT = 1,
} TCA9534_pinDirection_E;

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint32_t timeout_ms;
	uint8_t input_reg;
	uint8_t output_reg;
	uint8_t polarity_reg;
	uint8_t config_reg;
} TCA9534_inst_S;

HAL_StatusTypeDef TCA9534_init(TCA9534_inst_S *inst, I2C_HandleTypeDef *hi2c, uint32_t timeout_ms);
void TCA9534_setPinDirection(TCA9534_inst_S *inst, TCA9534_channel_E channel, TCA9534_pinDirection_E dir);
HAL_StatusTypeDef TCA9534_update(TCA9534_inst_S *inst);
GPIO_PinState TCA9534_readPin(TCA9534_inst_S *inst, TCA9534_channel_E channel);
void TCA9534_writePin(TCA9534_inst_S *inst, TCA9534_channel_E channel, GPIO_PinState state);
HAL_StatusTypeDef TCA9534_shutdown(TCA9534_inst_S *inst);

#endif // __TCA9534_H__
