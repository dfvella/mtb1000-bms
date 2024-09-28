#include "display.h"

#define BUTTON_LONG_PRESS_TIME_MS 2000

static uint8_t display_soc;
static batt_fault_E display_fault;
static uint8_t buttonPress;
static uint8_t buttonLongPress;

static uint32_t last_button_state_change;
static uint8_t button_state;
static uint8_t long_button_state;

static controller_state_E controller_state;
static uint32_t last_controller_state_change;

static uint8_t rotate1;
static uint8_t rotate2;
static uint32_t last_rotate_update;

void display_init(void)
{
	display_soc = 0;
	display_fault = 0;

	buttonPress = 0;
	buttonLongPress = 0;

	button_state = 0;
	long_button_state = 0;
	last_button_state_change = 0;

	last_rotate_update = 0;

	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
}

void display_update(controller_state_E state)
{
	uint8_t blink = ((HAL_GetTick() - last_controller_state_change) % 1000) < 500;
	uint8_t blink2 = ((HAL_GetTick() - last_controller_state_change) % 500) < 250;

	if (HAL_GetTick() - last_rotate_update > 250)
	{
		last_rotate_update = HAL_GetTick();

		if (rotate1 == 0xF)
		{
			rotate1 = 0;
		}
		else
		{
			rotate1 = (rotate1 << 1) | 1;
		}

		if (rotate2 == 0)
		{
			rotate2 = 1;
		}
		else
		{
			rotate2 = (rotate2 << 1) & 0xF;
		}
	}

	switch (state)
	{
	case STATE_OFF:
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, last_controller_state_change > 0);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, last_controller_state_change > 250);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, last_controller_state_change > 500);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, last_controller_state_change > 750);
		break;

	case STATE_PRECHARGE:
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, blink);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, blink);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, blink);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, blink);
		break;

	case STATE_IDLE:
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, display_soc > 20 ? 1 : blink2);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, display_soc > 40);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, display_soc > 60);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, display_soc > 80);
		break;

	case STATE_DISCHARGE:
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, display_soc > 20 ? (display_soc > 40 ? 1 : blink) : blink2);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, display_soc > 40 ? (display_soc > 60 ? 1 : blink) : 0);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, display_soc > 60 ? (display_soc > 80 ? 1 : blink) : 0);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, display_soc > 80 ? blink : 0);
		break;

	case STATE_CHARGE:
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, (rotate1 & 1 ? 1 : 0));
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, display_soc > 40 ? (rotate1 & 2 ? 1 : 0) : 0);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, display_soc > 60 ? (rotate1 & 4 ? 1 : 0) : 0);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, display_soc > 80 ? (rotate1 & 8 ? 1 : 0) : 0);
		break;

	case STATE_BALANCE:
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, (~rotate2 & 1 ? 1 : 0));
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, (~rotate2 & 2 ? 1 : 0));
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, (~rotate2 & 4 ? 1 : 0));
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, (~rotate2 & 8 ? 1 : 0));
		break;

	case STATE_FAULT:
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, display_fault & 8);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, display_fault & 4);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, display_fault & 2);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, display_fault & 1);
		break;

	case STATE_SHUTDOWN:
	default:
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
		break;
	}

	if (controller_state != state)
	{
		last_controller_state_change = HAL_GetTick();

		controller_state = state;
	}

	uint8_t current_button_state = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin);

	if (current_button_state != button_state)
	{
		last_button_state_change = HAL_GetTick();

		button_state = current_button_state;
	}

	uint32_t time_since_last_state_change = HAL_GetTick() - last_button_state_change;

	uint8_t current_button_long_state = button_state && (time_since_last_state_change >= BUTTON_LONG_PRESS_TIME_MS);

	buttonPress = current_button_state && !button_state;

	buttonLongPress = current_button_long_state && !long_button_state;

	long_button_state = current_button_long_state;
}

void display_setSOC(uint8_t soc)
{
	display_soc = soc;
}

void display_setFault(batt_fault_E fault)
{
	display_fault = 0;
	for (uint32_t i = 0; i < FAULT_COUNT; i++)
	{
		if ((fault >> i) & 1)
		{
			display_fault = i + 1;
			break;
		}
	}
}

uint8_t dislpay_getButtonPress(void)
{
	return buttonPress;
}

uint8_t display_getButtonLongPress(void)
{
	return buttonLongPress;
}
