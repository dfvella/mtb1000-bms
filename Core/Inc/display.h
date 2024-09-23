#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "stm32l0xx_hal.h"

#include "main.h"

#include "battery.h"
#include "controller.h"

void display_init(void);
void display_update(controller_state_E state);

void display_setSOC(uint8_t soc);
void display_setFault(batt_fault_E fault);

uint8_t dislpay_getButtonPress(void);
uint8_t display_getButtonLongPress(void);

#endif // __DISPLAY_H__

