#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

typedef enum
{
	STATE_OFF,
	STATE_PRECHARGE,
	STATE_IDLE,
	STATE_DISCHARGE,
	STATE_CHARGE,
	STATE_BALANCE,
	STATE_FAULT,
	STATE_SHUTDOWN,
} controller_state_E;

void controller_init(void);
void controller_run(void);

#endif // __CONTROLLER_H__
