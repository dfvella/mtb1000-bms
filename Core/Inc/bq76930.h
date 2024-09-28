#ifndef __BQ76930_H__
#define __BQ76930_H__

#include "stm32l0xx_hal.h"

#define BQ76930_I2C_ADDR 0x08

#define BQ76930_REG_SYS_STAT 0x00
#define BQ76930_REG_CELLBAL1 0x01
#define BQ76930_REG_CELLBAL2 0x02
#define BQ76930_REG_CELLBAL3 0x03
#define BQ76930_REG_SYS_CTRL1 0x04
#define BQ76930_REG_SYS_CTRL2 0x05
#define BQ76930_REG_PROTECT1 0x06
#define BQ76930_REG_PROTECT2 0x07
#define BQ76930_REG_PROTECT3 0x08
#define BQ76930_REG_OV_TRIP 0x09
#define BQ76930_REG_UV_TRIP 0x0A
#define BQ76930_REG_CC_CFG 0x0B
#define BQ76930_REG_VC1_HI 0x0C
#define BQ76930_REG_VC1_LO 0x0D
#define BQ76930_REG_VC2_HI 0x0E
#define BQ76930_REG_VC2_LO 0x0F
#define BQ76930_REG_VC3_HI 0x10
#define BQ76930_REG_VC3_LO 0x11
#define BQ76930_REG_VC4_HI 0x12
#define BQ76930_REG_VC4_LO 0x13
#define BQ76930_REG_VC5_HI 0x14
#define BQ76930_REG_VC5_LO 0x15
#define BQ76930_REG_VC6_HI 0x16
#define BQ76930_REG_VC6_LO 0x17
#define BQ76930_REG_VC7_HI 0x18
#define BQ76930_REG_VC7_LO 0x19
#define BQ76930_REG_VC8_HI 0x1A
#define BQ76930_REG_VC8_LO 0x1B
#define BQ76930_REG_VC9_HI 0x1C
#define BQ76930_REG_VC9_LO 0x1D
#define BQ76930_REG_VC10_HI 0x1E
#define BQ76930_REG_VC10_LO 0x1F
#define BQ76930_REG_VC11_HI 0x20
#define BQ76930_REG_VC11_LO 0x21
#define BQ76930_REG_VC12_HI 0x22
#define BQ76930_REG_VC12_LO 0x23
#define BQ76930_REG_VC13_HI 0x24
#define BQ76930_REG_VC13_LO 0x25
#define BQ76930_REG_VC14_HI 0x26
#define BQ76930_REG_VC14_LO 0x27
#define BQ76930_REG_VC15_HI 0x28
#define BQ76930_REG_VC15_LO 0x29
#define BQ76930_REG_BAT_HI 0x2A
#define BQ76930_REG_BAT_LO 0x2B
#define BQ76930_REG_TS1_HI 0x2C
#define BQ76930_REG_TS1_LO 0x2D
#define BQ76930_REG_TS2_HI 0x2E
#define BQ76930_REG_TS2_LO 0x2F
#define BQ76930_REG_TS3_HI 0x30
#define BQ76930_REG_TS3_LO 0x31
#define BQ76930_REG_CC_HI 0x32
#define BQ76930_REG_CC_LO 0x33
#define BQ76930_REG_ADCGAIN1 0x50
#define BQ76930_REG_ADCOFFSET 0x51
#define BQ76930_REG_ADCGAIN2 0x59

#define BQ76930_REG_SYS_STAT_OCD 0
#define BQ76930_REG_SYS_STAT_SCD 1
#define BQ76930_REG_SYS_STAT_OV 2
#define BQ76930_REG_SYS_STAT_UV 3
#define BQ76930_REG_SYS_STAT_OVRD_ALERT 4
#define BQ76930_REG_SYS_STAT_DEVICE_XREADY 5
#define BQ76930_REG_SYS_STAT_CC_READY 7

#define BQ76930_REG_SYS_CTRL1_TEMP_SEL 3
#define BQ76930_REG_SYS_CTRL1_ADC_EN 4

#define BQ76930_REG_SYS_CTRL2_CHG_ON 0
#define BQ76930_REG_SYS_CTRL2_DSG_ON 1

typedef enum
{
	BQ76930_CELL_1,
	BQ76930_CELL_2,
	BQ76930_CELL_3,
	BQ76930_CELL_4,
	BQ76930_CELL_5,
	BQ76930_CELL_6,
	BQ76930_CELL_7,
	BQ76930_CELL_8,
	BQ76930_CELL_9,
	BQ76930_CELL_10,
	BQ76930_CELL_11,
	BQ76930_CELL_12,
	BQ76930_CELL_13,
	BQ76930_CELL_14,
	BQ76930_CELL_15,

	BQ76930_CELL_COUNT,
} BQ76930_cell_E;

typedef enum
{
	BQ76930_TEMP_1,
	BQ76930_TEMP_2,
	BQ76930_TEMP_3,

	BQ76930_TEMP_INTERNAL1,
	BQ76930_TEMP_INTERNAL2,
	BQ76930_TEMP_INTERNAL3,

	BQ76930_TEMP_COUNT,
} BQ76930_temp_E;

typedef enum
{
	BQ76930_FAULT_OCD,
	BQ76930_FAULT_SCD,
	BQ76930_FAULT_OV,
	BQ76930_FAULT_UV,
	BQ76930_FAULT_INTERNAL,

	BQ76930_FAULT_COUNT,
} BQ76930_fault_E;

typedef enum
{
	BQ76930_FET_STATE_OFF,
	BQ76930_FET_STATE_ON,
} BQ76930_fetState_E;

typedef struct
{
	uint8_t scd_thresh;
	uint8_t ocd_thresh;
	uint16_t ov_thresh;
	uint16_t uv_thresh;
} BQ76930_config_S;

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint32_t timeout_ms;

	BQ76930_config_S config;

	uint32_t faults;

	BQ76930_fetState_E dsg;
	BQ76930_fetState_E chg;

	uint16_t cb;

	uint16_t adc_gain;
	uint16_t adc_offset;

	uint16_t data_volts[BQ76930_CELL_COUNT];
	uint16_t data_temps[BQ76930_TEMP_COUNT];

} BQ76930_inst_S;

HAL_StatusTypeDef BQ76930_init(BQ76930_inst_S *inst, I2C_HandleTypeDef *hi2c, BQ76930_config_S *config, uint32_t timeout_ms);
HAL_StatusTypeDef BQ76930_update(BQ76930_inst_S *inst);
HAL_StatusTypeDef BQ76930_clearFaults(BQ76930_inst_S *inst);
uint16_t BQ76930_getVoltage(BQ76930_inst_S *inst, BQ76930_cell_E cell);
uint16_t BQ76930_getTemp(BQ76930_inst_S *inst, BQ76930_temp_E temp);
uint8_t BQ76930_getFault(BQ76930_inst_S *inst, BQ76930_fault_E fault);
void BQ76930_setBalance(BQ76930_inst_S *inst, BQ76930_cell_E cell, BQ76930_fetState_E state);
void BQ76930_setCharge(BQ76930_inst_S *inst, BQ76930_fetState_E state);
void BQ76930_setDischarge(BQ76930_inst_S *inst, BQ76930_fetState_E state);
HAL_StatusTypeDef BQ76930_shutdown(BQ76930_inst_S *inst);


#endif // __BQ76930_H__
