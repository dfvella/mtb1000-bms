#include "controller.h"

#include "stm32l0xx_hal.h"

#include "battery.h"
#include "display.h"

#include <stdio.h>

#define PRECHARGE_THRESHOLD_MV 8000
#define PRECHARGE_TIMEOUT_MS 5000
#define IDLE_TIMEOUT_MS 30000
#define IDLE_CURRENT_HYST_MA 100

#define CHARGE_LIMIT_MV 4100
#define DISCHARGE_LIMIT_MV 3000
#define BALANCE_HYST_MV 100
#define CHARGE_HYST_MV 100

#define BALANCE_GROUP_A 0b1010101010101
#define BALANCE_GROUP_B 0b0101010101010

#define BALANCE_GROUP_TIME_MS 5000

#define LOOP_PERIOD_MS 100

#define SOC_TABLE_SIZE 12

extern UART_HandleTypeDef hlpuart1;

typedef struct
{
	uint32_t code;
	uint32_t capacity;
	uint16_t soc;
	int16_t state;
	uint16_t volt[CELL_COUNT];
	uint16_t temp[TEMP_COUNT];
	uint16_t fet;
	uint16_t pack_voltage;
	int16_t pack_current;
	uint16_t faults;
	uint16_t loop_time;
} controller_data_S;

static controller_data_S controller_data;

static controller_state_E controller_state;
static uint32_t precharge_start_time;
static uint32_t idle_start_time;
static uint32_t off_start_time;
static uint32_t last_balance_group_change;
static uint32_t active_balance_group;
static uint32_t last_controller_run;
static uint32_t capacity_remaining;
static uint8_t display_soc;

static uint32_t soc_table_voltage[SOC_TABLE_SIZE] =
{
	2500,
	3000,
	3300,
	3500,
	3600,
	3700,
	3800,
	3900,
	4000,
	4050,
	4100,
	4200,
};

static uint32_t soc_table_capacity[SOC_TABLE_SIZE] =
{
	3000 - 3000,
	3000 - 2880,
	3000 - 2560,
	3000 - 2240,
	3000 - 1920,
	3000 - 1600,
	3000 - 1280,
	3000 - 960,
	3000 - 640,
	3000 - 320,
	3000 - 150,
	3000 - 0,
};

static int64_t interpolate(int64_t x, int64_t x1, int64_t x2, int64_t y1, int64_t y2)
{
	return (((y2 - y1) * (x - x1)) / (x2 - x1)) + y1;
}

static uint32_t voltage2capacity(uint16_t v)
{
	uint32_t i = 1;
	while (i < SOC_TABLE_SIZE)
	{
		if (v < soc_table_voltage[i])
		{
			break;
		}
		i++;
	}

	return interpolate(v, soc_table_voltage[i - 1], soc_table_voltage[i], 4 * 3600 * soc_table_capacity[i - 1], 4 * 3600 * soc_table_capacity[i]);
}

static uint8_t capacity2soc(uint32_t capacity)
{
	uint32_t max_usable_capacity = voltage2capacity(CHARGE_LIMIT_MV);
	uint32_t min_usable_capacity = voltage2capacity(DISCHARGE_LIMIT_MV);

	int32_t soc = interpolate(capacity, min_usable_capacity, max_usable_capacity, 0, 100);

	if (soc < 0)
	{
		soc = 0;
	}

	if (soc > 100)
	{
		soc = 100;
	}

	return soc;
}

const char *state2str(controller_state_E state)
{
	switch (state)
	{
	case STATE_PRECHARGE: return "PRECHARGE";
	case STATE_IDLE: return "IDLE";
	case STATE_DISCHARGE: return "DISCHARGE";
	case STATE_CHARGE: return "CHARGE";
	case STATE_OFF: return "OFF";
	case STATE_BALANCE: return "BALANCE";
	case STATE_FAULT: return "FAULT";
	case STATE_SHUTDOWN: return "SHUTDOWN";
	default: return "";
	}
}

static uint32_t controller_updateCapacityRemaining(controller_state_E state, uint32_t cap)
{
	switch (state)
	{
	case STATE_OFF:
	case STATE_IDLE:
	case STATE_BALANCE:
		return voltage2capacity(batt_getCellVoltage(CELL_AVG));

	case STATE_PRECHARGE:
	case STATE_DISCHARGE:
	case STATE_CHARGE:
		return cap + ((batt_getPackCurrent() * LOOP_PERIOD_MS) / 1000);

	case STATE_FAULT:
	case STATE_SHUTDOWN:
	default:
		return cap;
	}
}

static controller_state_E controller_getNextState(controller_state_E state)
{
	if (batt_getFaultMask())
	{
		return STATE_FAULT;
	}

	switch (state)
	{
	case STATE_OFF:
		if ((HAL_GetTick() - off_start_time) >= 1000)
		{
			return STATE_PRECHARGE;
		}
		else
		{
			return STATE_OFF;
		}

	case STATE_PRECHARGE:
		if ((batt_getCellVoltage(CELL_SUM) - batt_getPackVoltage()) < PRECHARGE_THRESHOLD_MV)
		{
			return STATE_IDLE;
		}
		else if ((HAL_GetTick() - precharge_start_time) >= PRECHARGE_TIMEOUT_MS)
		{
			return STATE_IDLE; // TODO
		}
		else
		{
			return STATE_PRECHARGE;
		}

	case STATE_IDLE:
		if (display_getButtonLongPress())
		{
			return STATE_SHUTDOWN;
		}
		else if (batt_getPackCurrent() > IDLE_CURRENT_HYST_MA)
		{
			return STATE_DISCHARGE;
		}
		else if (batt_getPackCurrent() < -IDLE_CURRENT_HYST_MA)
		{
			return STATE_CHARGE;
		}
		else if ((HAL_GetTick() - idle_start_time) >= IDLE_TIMEOUT_MS)
		{
			return STATE_SHUTDOWN;
		}
		else
		{
			return STATE_IDLE;
		}

	case STATE_DISCHARGE:
		if (batt_getPackCurrent() < IDLE_CURRENT_HYST_MA)
		{
			return STATE_IDLE;
		}
		else
		{
			return STATE_DISCHARGE;
		}

	case STATE_CHARGE:
		if (batt_getCellVoltage(CELL_MAX) >= CHARGE_LIMIT_MV)
		{
			if ((batt_getCellVoltage(CELL_MAX) - batt_getCellVoltage(CELL_MIN)) > BALANCE_HYST_MV)
			{
				return STATE_BALANCE;
			}
			else
			{
				return STATE_SHUTDOWN;
			}
		}
		else if (batt_getPackCurrent() > -IDLE_CURRENT_HYST_MA)
		{
			return STATE_IDLE;
		}
		else
		{
			return STATE_CHARGE;
		}

	case STATE_BALANCE:
		if ((batt_getCellVoltage(CELL_MAX) - batt_getCellVoltage(CELL_MIN)) < BALANCE_HYST_MV)
		{
			if (batt_getCellVoltage(CELL_MAX) < (CHARGE_LIMIT_MV - CHARGE_HYST_MV))
			{
				return STATE_CHARGE;
			}
			else
			{
				return STATE_SHUTDOWN;
			}
		}
		else
		{
			return STATE_BALANCE;
		}

	case STATE_FAULT:
		if (display_getButtonLongPress())
		{
			return STATE_OFF;
		}
		else
		{
			return STATE_FAULT;
		}

	case STATE_SHUTDOWN:
	default:
		return STATE_SHUTDOWN;
	}
}

static void controller_entryAction(controller_state_E state)
{
	switch (state)
	{
	case STATE_OFF:
		batt_init();
		off_start_time = HAL_GetTick();
		break;

	case STATE_PRECHARGE:
		precharge_start_time = HAL_GetTick();
		break;

	case STATE_IDLE:
		idle_start_time = HAL_GetTick();
		break;

	case STATE_DISCHARGE:
	case STATE_CHARGE:
	case STATE_BALANCE:
	case STATE_FAULT:
	case STATE_SHUTDOWN:
	default:
		break;
	}
}

static void controller_setFetState(controller_state_E state)
{
	switch (state)
	{
	case STATE_PRECHARGE:
		batt_setFetState(FET_PCH, FET_ON);
		batt_setFetState(FET_CHG, FET_ON);
		batt_setFetState(FET_DSG, FET_OFF);
		break;

	case STATE_IDLE:
	case STATE_DISCHARGE:
	case STATE_CHARGE:
		batt_setFetState(FET_PCH, FET_OFF);
		batt_setFetState(FET_CHG, FET_ON);
		batt_setFetState(FET_DSG, FET_ON);
		break;

	case STATE_OFF:
	case STATE_BALANCE:
	case STATE_FAULT:
	case STATE_SHUTDOWN:
	default:
		batt_setFetState(FET_PCH, FET_OFF);
		batt_setFetState(FET_CHG, FET_OFF);
		batt_setFetState(FET_DSG, FET_OFF);
		break;
	}
}

static void controller_setBalanceState(controller_state_E state)
{
	if (HAL_GetTick() - last_balance_group_change > BALANCE_GROUP_TIME_MS)
	{
		last_balance_group_change = HAL_GetTick();

		if (active_balance_group == BALANCE_GROUP_A)
		{
			active_balance_group = BALANCE_GROUP_B;
		}
		else
		{
			active_balance_group = BALANCE_GROUP_A;
		}
	}

	for (uint32_t i = 0; i < CELL_COUNT; i++)
	{
		if (state == STATE_BALANCE)
		{
			uint8_t in_group = (active_balance_group >> i) & 1;
			uint8_t above_min = batt_getCellVoltage(i) > batt_getCellVoltage(CELL_MIN);
//			uint8_t above_min = i == 0;
			batt_setBalance(i, (in_group && above_min) ? FET_ON : FET_OFF);
		}
		else
		{
			batt_setBalance(i, FET_OFF);
		}
	}
}

static void controller_packData(void)
{
	controller_data.code = 0xDEADBEEF;
	controller_data.capacity = capacity_remaining;
	controller_data.soc = display_soc;
	controller_data.state = controller_state;
	for (uint32_t i = 0; i < CELL_COUNT; i++)
	{
		controller_data.volt[i] = batt_getCellVoltage(i);
	}
	for (uint32_t i = 0; i < TEMP_COUNT; i++)
	{
		controller_data.temp[i] = batt_getTemp(i);
	}
	controller_data.fet = 0;
	for (uint32_t i = 0; i < CELL_COUNT; i++)
	{
		controller_data.fet |= (batt_getBalanceState(i) == FET_ON ? 1 : 0) << i;
	}
	controller_data.fet |= (batt_getFetState(FET_PCH) == FET_ON ? 1 : 0) << 13;
	controller_data.fet |= (batt_getFetState(FET_CHG) == FET_ON ? 1 : 0) << 14;
	controller_data.fet |= (batt_getFetState(FET_DSG) == FET_ON ? 1 : 0) << 15;
	controller_data.pack_voltage = batt_getPackVoltage();
	controller_data.pack_current = batt_getPackCurrent();
	controller_data.faults = batt_getFaultMask();
	controller_data.loop_time = HAL_GetTick() - last_controller_run;
}

void controller_init(void)
{
	off_start_time = HAL_GetTick();
	last_balance_group_change = HAL_GetTick();
	active_balance_group = BALANCE_GROUP_A;
	controller_state = STATE_OFF;

	last_controller_run = 0;

	display_init();
	batt_init();
}

void controller_run(void)
{
	if ((HAL_GetTick() - last_controller_run) >= LOOP_PERIOD_MS)
	{
		last_controller_run = HAL_GetTick();

		batt_update();

		controller_state_E desired_state = controller_getNextState(controller_state);

		if (controller_state != desired_state)
		{
			controller_state = desired_state;
			controller_entryAction(controller_state);
		}

		capacity_remaining = controller_updateCapacityRemaining(controller_state, capacity_remaining);
		display_soc = capacity2soc(capacity_remaining);

		controller_setFetState(controller_state);
		controller_setBalanceState(controller_state);

		display_setFault(batt_getFaultMask());
		display_setSOC(display_soc);
		display_update(controller_state);

		HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t*)&controller_data, sizeof(controller_data_S));

		if (controller_state == STATE_SHUTDOWN)
		{
			batt_shutdown();
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
			HAL_PWR_EnterSTANDBYMode();
		}

		controller_packData();
	}
}
