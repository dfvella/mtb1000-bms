#include "tca9534.h"

#define TCA9534_SET_BIT(bits, bit, value) ((bits & ~(1 << bit)) | (value << bit))
#define TCA9534_GET_BIT(bits, bit) (bits & (1 << bit))

static HAL_StatusTypeDef TCA9534_readReg(TCA9534_inst_S * inst, uint8_t regAddr, uint8_t *data, uint8_t size)
{
	return HAL_I2C_Mem_Read(inst->hi2c, TCA9534_I2C_ADDR << 1, regAddr, 1, data, size, inst->timeout_ms);
}

static HAL_StatusTypeDef TCA9534_writeReg(TCA9534_inst_S * inst, uint8_t regAddr, uint8_t *data, uint8_t size)
{
	return HAL_I2C_Mem_Write(inst->hi2c, TCA9534_I2C_ADDR << 1, regAddr, 1, data, size, inst->timeout_ms);
}

HAL_StatusTypeDef TCA9534_init(TCA9534_inst_S *inst, I2C_HandleTypeDef *hi2c, uint32_t timeout_ms)
{
	inst->hi2c = hi2c;
	inst->timeout_ms = timeout_ms;
	inst->polarity_reg = 0;

	HAL_StatusTypeDef status;

	status = TCA9534_readReg(inst, TCA9534_REG_INP, &inst->input_reg, sizeof(inst->input_reg));

	if (status != HAL_OK)
	{
		return status;
	}

	status = TCA9534_readReg(inst, TCA9534_REG_OUT, &inst->output_reg, sizeof(inst->output_reg));

	if (status != HAL_OK)
	{
		return status;
	}

	status = TCA9534_writeReg(inst, TCA9534_REG_POL, &inst->polarity_reg, sizeof(inst->polarity_reg));

	if (status != HAL_OK)
	{
		return status;
	}

	return TCA9534_readReg(inst, TCA9534_REG_CFG, &inst->config_reg, sizeof(inst->config_reg));
}

void TCA9534_setPinDirection(TCA9534_inst_S *inst, TCA9534_channel_E channel, TCA9534_pinDirection_E dir)
{
	inst->config_reg = TCA9534_SET_BIT(inst->config_reg, channel, dir);
}

HAL_StatusTypeDef TCA9534_update(TCA9534_inst_S *inst)
{
	HAL_StatusTypeDef status;

	status = TCA9534_readReg(inst, TCA9534_REG_INP, &inst->input_reg, sizeof(inst->input_reg));

	if (status != HAL_OK)
	{
		return status;
	}

	status = TCA9534_writeReg(inst, TCA9534_REG_OUT, &inst->output_reg, sizeof(inst->output_reg));

	if (status != HAL_OK)
	{
		return status;
	}

	status = TCA9534_writeReg(inst, TCA9534_REG_POL, &inst->polarity_reg, sizeof(inst->polarity_reg));

	if (status != HAL_OK)
	{
		return status;
	}

	return TCA9534_writeReg(inst, TCA9534_REG_CFG, &inst->config_reg, sizeof(inst->config_reg));
}

GPIO_PinState TCA9534_readPin(TCA9534_inst_S *inst, TCA9534_channel_E channel)
{
	return TCA9534_GET_BIT(inst->input_reg, channel) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void TCA9534_writePin(TCA9534_inst_S *inst, TCA9534_channel_E channel, GPIO_PinState state)
{
	inst->output_reg = TCA9534_SET_BIT(inst->output_reg, channel, state);
}

HAL_StatusTypeDef TCA9534_shutdown(TCA9534_inst_S *inst)
{
	inst->output_reg = 1;
	inst->polarity_reg = 0;
	inst->config_reg = 1;

	return TCA9534_update(inst);
}
