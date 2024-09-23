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

static controller_state_E state;
static uint32_t precharge_start_time;
static uint32_t idle_start_time;
static uint32_t off_start_time;
static uint32_t last_balance_group_change;
static uint32_t active_balance_group;
static uint32_t last_controller_run;

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
			batt_setBalance(i, (in_group && above_min) ? FET_ON : FET_OFF);
		}
		else
		{
			batt_setBalance(i, FET_OFF);
		}
	}
}

void controller_init(void)
{
	printf("initializing controller ...\n");

	off_start_time = HAL_GetTick();
	last_balance_group_change = HAL_GetTick();
	active_balance_group = BALANCE_GROUP_A;
	state = STATE_OFF;

	last_controller_run = 0;

	display_init();
	batt_init();
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

void controller_run(void)
{
	if ((HAL_GetTick() - last_controller_run) >= LOOP_PERIOD_MS)
	{
		last_controller_run = HAL_GetTick();

		batt_update();

		controller_state_E desired_state = controller_getNextState(state);

		if (state != desired_state)
		{
			state = desired_state;
			controller_entryAction(state);
		}

		controller_setFetState(state);
		controller_setBalanceState(state);
		display_setFault(batt_getFaultMask());
		display_setSOC(100);
		display_update(state);

		printf("state:%s  button:%d  long:%d\n", state2str(state), HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin), display_getButtonLongPress());

		if (state == STATE_SHUTDOWN)
		{
			batt_shutdown();
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);
			HAL_PWR_EnterSTANDBYMode();
		}

//		printf("ov:%d  ", batt_getFault(FAULT_OV));
//		printf("uv:%d  ", batt_getFault(FAULT_UV));
//		printf("oc:%d  ", batt_getFault(FAULT_OC));
//		printf("sc:%d  ", batt_getFault(FAULT_SC));
//		printf("bq:%d  ", batt_getFault(FAULT_BQ));
//		printf("ot:%d  ", batt_getFault(FAULT_OT));
//		printf("comms:%d\n", batt_getFault(FAULT_COMMS));
//
//		printf("vc1:%d  ", batt_getCellVoltage(CELL_1));
//		printf("vc2:%d  ", batt_getCellVoltage(CELL_2));
//		printf("vc3:%d  ", batt_getCellVoltage(CELL_3));
//		printf("vc4:%d  ", batt_getCellVoltage(CELL_4));
//		printf("vc5:%d  ", batt_getCellVoltage(CELL_5));
//		printf("vc6:%d  ", batt_getCellVoltage(CELL_6));
//		printf("vc7:%d  ", batt_getCellVoltage(CELL_7));
//		printf("vc8:%d  ", batt_getCellVoltage(CELL_8));
//		printf("vc9:%d  ", batt_getCellVoltage(CELL_9));
//		printf("vc10:%d  ", batt_getCellVoltage(CELL_10));
//		printf("vc11:%d  ", batt_getCellVoltage(CELL_11));
//		printf("vc12:%d  ", batt_getCellVoltage(CELL_12));
//		printf("vc13:%d\n", batt_getCellVoltage(CELL_13));
//
//		printf("sum:%d  ", batt_getCellVoltage(CELL_SUM));
//		printf("pv:%d  ", batt_getPackVoltage());
//		printf("pc:%ld\n", batt_getPackCurrent());
//
//		printf("ts1:%d  ", batt_getTemp(THERMISTOR_1));
//		printf("ts2:%d  ", batt_getTemp(THERMISTOR_2));
//		printf("ts3:%d  ", batt_getTemp(THERMISTOR_3));
//		printf("bq1:%d  ", batt_getTemp(BQ_1));
//		printf("bq2:%d  ", batt_getTemp(BQ_2));
//		printf("bq3:%d\n", batt_getTemp(BQ_3));
	}
}
