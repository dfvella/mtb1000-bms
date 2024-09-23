#ifndef __BATTERY_H__
#define __BATTERY_H__

#include <stdint.h>

#include "stm32l0xx_hal.h"

typedef enum
{
	CELL_1,
	CELL_2,
	CELL_3,
	CELL_4,
	CELL_5,
	CELL_6,
	CELL_7,
	CELL_8,
	CELL_9,
	CELL_10,
	CELL_11,
	CELL_12,
	CELL_13,

	CELL_COUNT,

	CELL_MIN,
	CELL_MAX,
	CELL_AVG,
	CELL_SUM,
} batt_cell_E;

typedef enum
{
	FET_PCH,
	FET_CHG,
	FET_DSG,
} batt_fet_E;

typedef enum
{
	FET_OFF,
	FET_ON,
} batt_fetState_E;

typedef enum
{
	THERMISTOR_1,
	THERMISTOR_2,
	THERMISTOR_3,
	BQ_1,
	BQ_2,
	BQ_3,

	TEMP_COUNT,

	TEMP_MAX,
} batt_temp_E;

typedef enum
{
	FAULT_OV,
	FAULT_UV,
	FAULT_SC,
	FAULT_OC,
	FAULT_BQ,
	FAULT_OT,
	FAULT_COMMS,

	FAULT_COUNT,
} batt_fault_E;

void batt_init(void);
void batt_update(void);
uint16_t batt_getCellVoltage(batt_cell_E cell);
uint16_t batt_getPackVoltage(void);
int32_t batt_getPackCurrent(void);
uint8_t batt_getTemp(batt_temp_E temp);
uint8_t batt_getFault(batt_fault_E fault);
uint8_t batt_getFaultMask(void);
void batt_setFetState(batt_fet_E fet, batt_fetState_E state);
void batt_setBalance(batt_cell_E cell, batt_fetState_E state);
void batt_shutdown(void);

#endif // __BATTERY_H__
