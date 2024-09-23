#include "adc121.h"

static HAL_StatusTypeDef ADC121_readReg(ADC121_inst_S * inst, uint8_t regAddr, uint8_t *data, uint8_t size)
{
	return HAL_I2C_Mem_Read(inst->hi2c, ADC121_I2C_ADDR << 1, regAddr, 1, data, size, inst->timeout_ms);
}

static HAL_StatusTypeDef ADC121_writeReg(ADC121_inst_S * inst, uint8_t regAddr, uint8_t *data, uint8_t size)
{
	return HAL_I2C_Mem_Write(inst->hi2c, ADC121_I2C_ADDR << 1, regAddr, 1, data, size, inst->timeout_ms);
}

HAL_StatusTypeDef ADC121_init(ADC121_inst_S * inst, I2C_HandleTypeDef *hi2c, uint32_t timeout_ms)
{
	inst->hi2c = hi2c;
	inst->timeout_ms = timeout_ms;

	uint8_t data = 0b11100000;

	return ADC121_writeReg(inst, ADC121_REG_CFG, &data, sizeof(data));
}

HAL_StatusTypeDef ADC121_update(ADC121_inst_S *inst)
{
	uint8_t data[2];

	HAL_StatusTypeDef status = ADC121_readReg(inst, ADC121_REG_RES, data, sizeof(data));

	if (status == HAL_OK)
	{
		inst->data = (uint16_t)(data[0] << 8) | data[1];
		inst->data &= 0x0FFF;
	}

	return status;
}

uint16_t ADC121_read(ADC121_inst_S *inst)
{
	return inst->data;
}

HAL_StatusTypeDef ADC121_shutdown(ADC121_inst_S *inst)
{
	uint8_t data = 0b00000000;

	return ADC121_writeReg(inst, ADC121_REG_CFG, &data, sizeof(data));
}
