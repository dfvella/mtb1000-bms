#include "bq76930.h"
#include <string.h>
#include <stdio.h>

#define BQ76930_SET_BIT(bits, bit, value) ((bits & ~(1 << bit)) | (value << bit))
#define BQ76930_GET_BIT(bits, bit) (bits & (1 << bit))

static const uint8_t crc8_table[256] = {
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
	0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
	0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
	0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
	0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
	0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
	0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
	0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
	0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
	0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
	0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3,
};

#define THERMISTOR_TABLE_LEN 17

static const uint16_t thermistor_resistance_table[THERMISTOR_TABLE_LEN] = {
	35820,
	27340,
	21020,
	16290,
	12720,
	10000,
	7921,
	6315,
	5067,
	4090,
	3319,
	2709,
	2222,
	1832,
	1518,
};

static const uint16_t thermistor_temperature_table[THERMISTOR_TABLE_LEN] = {
	0,
	5,
	10,
	15,
	20,
	25,
	30,
	35,
	40,
	45,
	50,
	55,
	60,
	65,
	70
};

static uint8_t crc8(uint8_t *data, uint32_t len)
{
	uint8_t crc = 0x00;

	for (uint8_t i = 0; i < len; i++)
	{
		uint8_t ind = data[i] ^ crc;
		crc = crc8_table[ind];
	}

	return crc;
}

static int64_t interpolate(int64_t x, int64_t x1, int64_t x2, int64_t y1, int64_t y2)
{
	return (((y2 - y1) * (x - x1)) / (x2 - x1)) + y1;
}

static uint16_t BQ76930_adc2Volt(BQ76930_inst_S *inst, uint16_t adc)
{
	return ((inst->adc_gain * (uint32_t)adc) / 1000) + inst->adc_offset;
}

static uint16_t BQ76930_volt2Adc(BQ76930_inst_S *inst, uint16_t v)
{
	return (1000 * ((int32_t)v - inst->adc_offset)) / inst->adc_gain;
}

static uint8_t BQ76930_adc2Temp(BQ76930_inst_S *inst, uint16_t adc)
{
	uint32_t mv = BQ76930_adc2Volt(inst, adc);
	uint32_t r = (10000 * mv) / (3300 - mv);

	uint32_t i = 1;
	while (i < THERMISTOR_TABLE_LEN)
	{
		if (r > thermistor_resistance_table[i])
		{
			break;
		}

		i++;
	}

	int32_t r1 = thermistor_resistance_table[i];
	int32_t r2 = thermistor_resistance_table[i + 1];
	int32_t t1 = 1000000 * thermistor_temperature_table[i];
	int32_t t2 = 1000000 * thermistor_temperature_table[i + 1];

	return (interpolate(r, r1, r2, t1, t2) / 1000000);
}

//static uint8_t BQ76930_adc2TempInternal(BQ76930_inst_S *inst, uint16_t adc)
//{
//	uint32_t mv = BQ76930_adc2Volt(inst, adc);
//
//	return 25 - ((1000 * (int32_t)(mv - 1200)) / 4200);
//}

static HAL_StatusTypeDef BQ76930_writeReg(BQ76930_inst_S *inst, uint8_t addr, uint8_t *byte)
{
	uint8_t data[4];

	data[0] = (BQ76930_I2C_ADDR << 1) | 0b0;
	data[1] = addr;
	data[2] = *byte;
	data[3] = crc8(&data[0], 3);

	return HAL_I2C_Mem_Write(inst->hi2c, BQ76930_I2C_ADDR << 1, addr, 1, &data[2], 2, inst->timeout_ms);
}

static HAL_StatusTypeDef BQ76930_readReg(BQ76930_inst_S *inst, uint8_t addr, uint8_t *byte)
{
	uint8_t data[3];

	data[0] = (BQ76930_I2C_ADDR << 1) | 0b1;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(inst->hi2c, BQ76930_I2C_ADDR << 1, addr, 1, &data[1], 2, inst->timeout_ms);

	if (status != HAL_OK)
	{
		return status;
	}

	uint8_t expected_crc = crc8(&data[0], 2);

	if (expected_crc != data[2])
	{
		return HAL_ERROR;
	}

	*byte = data[1];

	return status;
}

static HAL_StatusTypeDef BQ76930_readRegMultiple(BQ76930_inst_S *inst, uint8_t addr, uint8_t *data, uint8_t len)
{
	uint8_t buf[258];

	buf[0] = (BQ76930_I2C_ADDR << 1) | 0b1;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(inst->hi2c, BQ76930_I2C_ADDR << 1, addr, 1, &buf[1], len + 1, inst->timeout_ms);

	if (status != HAL_OK)
	{
		return status;
	}

	// TODO
//	uint8_t expected_crc = crc8(&buf[0], len + 1);
//
//	if (expected_crc != buf[len + 1])
//	{
//		return HAL_ERROR;
//	}

	memcpy(data, &buf[1], len);

	return status;
}

HAL_StatusTypeDef BQ76930_clearFaults(BQ76930_inst_S *inst)
{
	uint8_t data = 0xFF;

	return BQ76930_writeReg(inst, BQ76930_REG_SYS_STAT, &data);
}

HAL_StatusTypeDef BQ76930_init(BQ76930_inst_S *inst, I2C_HandleTypeDef *hi2c, BQ76930_config_S *config, uint32_t timeout_ms)
{
	memset(inst, 0, sizeof(BQ76930_inst_S));

	inst->hi2c = hi2c;
	inst->timeout_ms = timeout_ms;

	inst->adc_gain = 377;
	inst->adc_offset = 46;

	HAL_StatusTypeDef status = HAL_OK;

	uint8_t buf = 0;

	// write configuration
	buf = config->scd_thresh & 0x07;

	status = BQ76930_writeReg(inst, BQ76930_REG_PROTECT1, &buf);

	if (status != HAL_OK)
	{
		return status;
	}

	buf = config->ocd_thresh & 0xF;

	status = BQ76930_writeReg(inst, BQ76930_REG_PROTECT2, &buf);

	if (status != HAL_OK)
	{
		return status;
	}

	buf = 0;

	status = BQ76930_writeReg(inst, BQ76930_REG_PROTECT3, &buf);

	if (status != HAL_OK)
	{
		return status;
	}

	buf = (BQ76930_volt2Adc(inst, config->ov_thresh) >> 4) & 0xFF;

	status = BQ76930_writeReg(inst, BQ76930_REG_OV_TRIP, &buf);

	if (status != HAL_OK)
	{
		return status;
	}

	buf = (BQ76930_volt2Adc(inst, config->uv_thresh) >> 4) & 0xFF;

	status = BQ76930_writeReg(inst, BQ76930_REG_UV_TRIP, &buf);

	if (status != HAL_OK)
	{
		return status;
	}

	// enable adc
	buf = 0;
	buf = BQ76930_SET_BIT(buf, BQ76930_REG_SYS_CTRL1_ADC_EN, 1);
	buf = BQ76930_SET_BIT(buf, BQ76930_REG_SYS_CTRL1_TEMP_SEL, 0);

	status = BQ76930_writeReg(inst, BQ76930_REG_SYS_CTRL1, &buf);

	if (status != HAL_OK)
	{
		return status;
	}

	buf = 0;

	status = BQ76930_writeReg(inst, BQ76930_REG_SYS_CTRL2, &buf);

	if (status != HAL_OK)
	{
		return status;
	}

	// clear faults
	return BQ76930_clearFaults(inst);
}

HAL_StatusTypeDef BQ76930_update(BQ76930_inst_S *inst)
{
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t buf[256];

	// read faults
	status = BQ76930_readReg(inst, BQ76930_REG_SYS_STAT, buf);

	if (status != HAL_OK)
	{
		return status;
	}

	inst->faults = buf[0] & 0xF;
	inst->faults = BQ76930_SET_BIT(inst->faults, BQ76930_FAULT_INTERNAL, (buf[0] >> BQ76930_REG_SYS_STAT_DEVICE_XREADY) & 1);

	// read volt data
	for (uint32_t i = 0; i < BQ76930_CELL_COUNT; i++)
	{
		status = BQ76930_readRegMultiple(inst, BQ76930_REG_VC1_HI + (2 * i), buf, 2);

		if (status != HAL_OK)
		{
			return status;
		}

		uint16_t raw = ((uint16_t)(buf[0] << 8) | buf[1]) & 0x3FFF;
		uint16_t raw_volt = BQ76930_adc2Volt(inst, raw);
		if ((2500 < raw_volt) && (raw_volt < 4200)) // TODO
		{
			inst->data_volts[i] = BQ76930_adc2Volt(inst, raw);
		}
//		printf("%d  ", raw);
	}
//	printf("\n");

	// read temperature data
	for (uint32_t i = 0; i < 3; i++)
	{
		status = BQ76930_readRegMultiple(inst, BQ76930_REG_TS1_HI + (2 * i), buf, 2);

		if (status != HAL_OK)
		{
			return status;
		}

		uint16_t raw = ((uint16_t)(buf[0] << 8) | buf[1]) & 0x3FFF;
		inst->data_temps[i] = BQ76930_adc2Temp(inst, raw);
	}

	// set balance control
	buf[0] = inst->cb & 0x1F;

	status = BQ76930_writeReg(inst, BQ76930_REG_CELLBAL1, buf);

	if (status != HAL_OK)
	{
		return status;
	}

	buf[0] = (inst->cb >> 5) & 0x1F;

	status = BQ76930_writeReg(inst, BQ76930_REG_CELLBAL2, buf);

	if (status != HAL_OK)
	{
		return status;
	}

	buf[0] = (inst->cb >> 10) & 0x1F;

	status = BQ76930_writeReg(inst, BQ76930_REG_CELLBAL3, buf);

	if (status != HAL_OK)
	{
		return status;
	}

	// set mosfet state
	status = BQ76930_readReg(inst, BQ76930_REG_SYS_CTRL2, buf);

	if (status != HAL_OK)
	{
		return status;
	}

	buf[0] = BQ76930_SET_BIT(buf[0], BQ76930_REG_SYS_CTRL2_CHG_ON, inst->chg);
	buf[0] = BQ76930_SET_BIT(buf[0], BQ76930_REG_SYS_CTRL2_DSG_ON, inst->dsg);

	return BQ76930_writeReg(inst, BQ76930_REG_SYS_CTRL2, buf);
}

uint16_t BQ76930_getVoltage(BQ76930_inst_S *inst, BQ76930_cell_E cell)
{
	return inst->data_volts[cell];
}

uint16_t BQ76930_getTemp(BQ76930_inst_S *inst, BQ76930_temp_E temp)
{
	return inst->data_temps[temp];
}

uint8_t BQ76930_getFault(BQ76930_inst_S *inst, BQ76930_fault_E fault)
{
	return (inst->faults >> fault) & 1;
}

void BQ76930_setBalance(BQ76930_inst_S *inst, BQ76930_cell_E cell, BQ76930_fetState_E state)
{
	inst->cb = BQ76930_SET_BIT(inst->cb, cell, state);
}

void BQ76930_setCharge(BQ76930_inst_S *inst, BQ76930_fetState_E state)
{
	inst->chg = state;
}

void BQ76930_setDischarge(BQ76930_inst_S *inst, BQ76930_fetState_E state)
{
	inst->dsg = state;
}

HAL_StatusTypeDef BQ76930_shutdown(BQ76930_inst_S *inst)
{
	uint8_t buf = 0;

	HAL_StatusTypeDef status = BQ76930_writeReg(inst, BQ76930_REG_SYS_CTRL1, &buf);

	if (status != HAL_OK)
	{
		return status;
	}

	return BQ76930_writeReg(inst, BQ76930_REG_SYS_CTRL2, &buf);
}
