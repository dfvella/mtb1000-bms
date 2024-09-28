#include "battery.h"

#include "adc121.h"
#include "bq76930.h"
#include "tca9534.h"

#define BATT_SET_BIT(bits, bit, value) ((bits & ~(1 << bit)) | (value << bit))
#define BATT_GET_BIT(bits, bit) (bits & (1 << bit))

#define I2C_TIMEOUT_MS 10

#define OV_THRESH_MV 4200
#define UV_THRESH_MV 2000
#define OC_THRESH 0x5 // 22A // bq datasheet pg 33
#define SC_THRESH 0x3 // 56A
#define OT_THRESH_C 60

#define CHANNEL_PCHG_EN TCA9534_CHANNEL_1
#define CHANNEL_PMON_EN TCA9534_CHANNEL_2
#define CHANNEL_CP_EN TCA9534_CHANNEL_3
#define CHANNEL_TMUX_SEL_0 TCA9534_CHANNEL_4
#define CHANNEL_TMUX_SEL_1 TCA9534_CHANNEL_5
#define CHANNEL_SNS_EN TCA9534_CHANNEL_6
#define CHANNEL_TMUX_EN TCA9534_CHANNEL_7
#define CHANNEL_BQ_ALERT TCA9534_CHANNEL_8

extern I2C_HandleTypeDef hi2c1;

static ADC121_inst_S adc;
static BQ76930_inst_S bq;
static TCA9534_inst_S tca;

static uint16_t pack_voltage;
static int32_t pack_current;
static uint8_t adc_select;
static batt_fetState_E pch_state;
static batt_fetState_E chg_state;
static batt_fetState_E dsg_state;
static batt_fetState_E bal_state[CELL_COUNT];
static uint8_t faults;
static uint16_t v_min;
static uint16_t v_max;
static uint16_t v_avg;
static uint16_t v_sum;
static uint8_t t_max;

static BQ76930_cell_E getBQCell(batt_cell_E cell)
{
	switch (cell)
	{
	case CELL_1: return BQ76930_CELL_1;
	case CELL_2: return BQ76930_CELL_2;
	case CELL_3: return BQ76930_CELL_3;
	case CELL_4: return BQ76930_CELL_4;
	case CELL_5: return BQ76930_CELL_5;
	case CELL_6: return BQ76930_CELL_6;
	case CELL_7: return BQ76930_CELL_7;
	case CELL_8: return BQ76930_CELL_8;
	case CELL_9: return BQ76930_CELL_10;
	case CELL_10: return BQ76930_CELL_11;
	case CELL_11: return BQ76930_CELL_12;
	case CELL_12: return BQ76930_CELL_13;
	case CELL_13: return BQ76930_CELL_15;
	default: return 0;
	}
}

void batt_init(void)
{
	BQ76930_config_S config =
	{
		.ocd_thresh = OC_THRESH,
		.ov_thresh = OV_THRESH_MV,
		.scd_thresh = SC_THRESH,
		.uv_thresh = UV_THRESH_MV,
	};

	HAL_StatusTypeDef status = HAL_OK;

	pack_voltage = 0;
	pack_current = 0;
	adc_select = 1;
	pch_state = FET_OFF;
	chg_state = FET_OFF;
	dsg_state = FET_OFF;
	faults = 0;
	v_min = 0;
	v_max = 0;
	v_avg = 0;
	v_sum = 0;
	t_max = 0;

	for (uint32_t i = 0; i < CELL_COUNT; i++)
	{
		bal_state[i] = FET_OFF;
	}

	status |= ADC121_init(&adc, &hi2c1, I2C_TIMEOUT_MS);
	status |= BQ76930_init(&bq, &hi2c1, &config, I2C_TIMEOUT_MS);
	status |= TCA9534_init(&tca, &hi2c1, I2C_TIMEOUT_MS);

    TCA9534_setPinDirection(&tca, CHANNEL_PCHG_EN, TCA9534_OUTPUT);
    TCA9534_setPinDirection(&tca, CHANNEL_PMON_EN, TCA9534_OUTPUT);
    TCA9534_setPinDirection(&tca, CHANNEL_CP_EN, TCA9534_OUTPUT);
    TCA9534_setPinDirection(&tca, CHANNEL_TMUX_SEL_0, TCA9534_OUTPUT);
    TCA9534_setPinDirection(&tca, CHANNEL_TMUX_SEL_1, TCA9534_OUTPUT);
    TCA9534_setPinDirection(&tca, CHANNEL_SNS_EN, TCA9534_OUTPUT);
    TCA9534_setPinDirection(&tca, CHANNEL_TMUX_EN, TCA9534_OUTPUT);
    TCA9534_setPinDirection(&tca, CHANNEL_BQ_ALERT, TCA9534_INPUT);

    TCA9534_writePin(&tca, CHANNEL_PCHG_EN, GPIO_PIN_RESET);
    TCA9534_writePin(&tca, CHANNEL_PMON_EN, GPIO_PIN_SET);

    TCA9534_writePin(&tca, CHANNEL_CP_EN, GPIO_PIN_SET);

    TCA9534_writePin(&tca, CHANNEL_TMUX_SEL_0, GPIO_PIN_RESET);
    TCA9534_writePin(&tca, CHANNEL_TMUX_SEL_1, GPIO_PIN_SET);

    TCA9534_writePin(&tca, CHANNEL_SNS_EN, GPIO_PIN_SET);
    TCA9534_writePin(&tca, CHANNEL_TMUX_EN, GPIO_PIN_SET);

    status |= TCA9534_update(&tca);

    faults = BATT_SET_BIT(faults, FAULT_COMMS, (status != HAL_OK));
}

void batt_update(void)
{
	HAL_StatusTypeDef status = HAL_OK;

	status |= ADC121_update(&adc);

	uint32_t adc_mv = (3300 * ADC121_read(&adc)) / 4095;

	if (adc_select)
	{
		pack_voltage = (18647 * adc_mv) / 1000;

	    TCA9534_writePin(&tca, CHANNEL_TMUX_SEL_0, GPIO_PIN_RESET);
	    TCA9534_writePin(&tca, CHANNEL_TMUX_SEL_1, GPIO_PIN_RESET);

	    TCA9534_writePin(&tca, CHANNEL_PMON_EN, GPIO_PIN_RESET);

		adc_select = 0;
	}
	else
	{
		pack_current = ((62500 * ((int32_t)adc_mv - 1650)) / 1000) + 937;

	    TCA9534_writePin(&tca, CHANNEL_TMUX_SEL_0, GPIO_PIN_RESET);
	    TCA9534_writePin(&tca, CHANNEL_TMUX_SEL_1, GPIO_PIN_SET);

	    TCA9534_writePin(&tca, CHANNEL_PMON_EN, GPIO_PIN_SET);

		adc_select = 1;
	}

	status |= BQ76930_update(&bq);
	status |= TCA9534_update(&tca);

	v_min = batt_getCellVoltage(CELL_1);
	v_max = v_min;
	v_sum = v_min;

	for (uint32_t i = 1; i < CELL_COUNT; i++)
	{
		uint16_t v = batt_getCellVoltage(i);

		if (v < v_min)
		{
			v_min = v;
		}

		if (v > v_max)
		{
			v_max = v;
		}

		v_sum += v;
	}

	v_avg = v_sum / CELL_COUNT;

	t_max = batt_getTemp(THERMISTOR_1);

	for (uint32_t i = 1; i < TEMP_COUNT; i++)
	{
		uint8_t t = batt_getTemp(i);

		if (t > t_max)
		{
			t_max = t;
		}
	}

	uint8_t fault_ov = BQ76930_getFault(&bq, BQ76930_FAULT_OV);
	uint8_t fault_uv = BQ76930_getFault(&bq, BQ76930_FAULT_UV);
	uint8_t fault_oc = BQ76930_getFault(&bq, BQ76930_FAULT_OCD);
	uint8_t fault_sc = BQ76930_getFault(&bq, BQ76930_FAULT_SCD);
	uint8_t fault_bq = BQ76930_getFault(&bq, BQ76930_FAULT_INTERNAL);
	uint8_t fault_ot = 0;

	fault_ov |= (v_max > OV_THRESH_MV);
	fault_uv |= (v_min < UV_THRESH_MV);
	fault_ot |= (t_max > OT_THRESH_C);

	faults = BATT_SET_BIT(faults, FAULT_OV, fault_ov);
	faults = BATT_SET_BIT(faults, FAULT_UV, fault_uv);
	faults = BATT_SET_BIT(faults, FAULT_OC, fault_oc);
	faults = BATT_SET_BIT(faults, FAULT_SC, fault_sc);
	faults = BATT_SET_BIT(faults, FAULT_BQ, fault_bq);
	faults = BATT_SET_BIT(faults, FAULT_OT, fault_ot);
	faults = BATT_SET_BIT(faults, FAULT_COMMS, (status != HAL_OK));
}

uint16_t batt_getCellVoltage(batt_cell_E cell)
{
	switch (cell)
	{
	case CELL_1:
	case CELL_2:
	case CELL_3:
	case CELL_4:
	case CELL_5:
	case CELL_6:
	case CELL_7:
	case CELL_8:
	case CELL_9:
	case CELL_10:
	case CELL_11:
	case CELL_12:
	case CELL_13:
		return BQ76930_getVoltage(&bq, getBQCell(cell));

	case CELL_MAX:
		return v_max;

	case CELL_MIN:
		return v_min;

	case CELL_AVG:
		return v_avg;

	case CELL_SUM:
		return v_sum;

	case CELL_COUNT:
	default:
		return 0;
	}
}

uint16_t batt_getPackVoltage(void)
{
	return pack_voltage;
}

int32_t batt_getPackCurrent(void)
{
	return pack_current;
}

uint8_t batt_getTemp(batt_temp_E temp)
{
	switch (temp)
	{
	case THERMISTOR_1:
	case THERMISTOR_2:
	case THERMISTOR_3:
	case BQ_1:
	case BQ_2:
		return BQ76930_getTemp(&bq, (BQ76930_temp_E)temp);

	case TEMP_MAX:
		return t_max;

	case TEMP_COUNT:
	default:
		return 0;
	}
}

uint8_t batt_getFault(batt_fault_E fault)
{
	return (faults >> fault) & 1;
}

uint8_t batt_getFaultMask(void)
{
	return faults;
}

batt_fetState_E batt_getFetState(batt_fet_E fet)
{
	switch (fet)
	{
	case FET_PCH:
		return pch_state;

	case FET_CHG:
		return chg_state;

	case FET_DSG:
		return dsg_state;

	default:
		return FET_OFF;
	}
}

batt_fetState_E batt_getBalanceState(batt_cell_E cell)
{
	return bal_state[cell];
}

void batt_setFetState(batt_fet_E fet, batt_fetState_E state)
{
	switch (fet)
	{
	case FET_PCH:
		pch_state = state;
		TCA9534_writePin(&tca, CHANNEL_PCHG_EN, (state == FET_ON) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		break;

	case FET_CHG:
		chg_state = state;
		BQ76930_setCharge(&bq, (state == FET_ON) ? BQ76930_FET_STATE_ON : BQ76930_FET_STATE_OFF);
		break;

	case FET_DSG:
		dsg_state = state;
		BQ76930_setDischarge(&bq, (state == FET_ON) ? BQ76930_FET_STATE_ON : BQ76930_FET_STATE_OFF);
		break;

	default:
		break;
	}
}

void batt_setBalance(batt_cell_E cell, batt_fetState_E state)
{
	bal_state[cell] = state;
	BQ76930_setBalance(&bq, getBQCell(cell), (state == FET_ON) ? BQ76930_FET_STATE_ON : BQ76930_FET_STATE_OFF);
}

void batt_shutdown(void)
{
    TCA9534_writePin(&tca, CHANNEL_PCHG_EN, GPIO_PIN_RESET);
    TCA9534_writePin(&tca, CHANNEL_PMON_EN, GPIO_PIN_RESET);
    TCA9534_writePin(&tca, CHANNEL_CP_EN, GPIO_PIN_RESET);
    TCA9534_writePin(&tca, CHANNEL_TMUX_SEL_0, GPIO_PIN_RESET);
    TCA9534_writePin(&tca, CHANNEL_TMUX_SEL_1, GPIO_PIN_RESET);
    TCA9534_writePin(&tca, CHANNEL_SNS_EN, GPIO_PIN_RESET);
    TCA9534_writePin(&tca, CHANNEL_TMUX_EN, GPIO_PIN_RESET);

    (void)TCA9534_update(&tca);

    (void)TCA9534_shutdown(&tca);
    (void)BQ76930_shutdown(&bq);
    (void)ADC121_shutdown(&adc);
}
