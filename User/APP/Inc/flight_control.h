#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

/**
 * @file     flight_control.h
 * @brief
 * @author   Mr.Five
 * @date     2025-08-07 11:48:40
 */
#include "main.h"
#include "cmsis_os2.h"
#include "light.h"
#include "attitude.h"
#include "pid.h"
#include "bmi088.h"
#include "spl06.h"
#include "state_machine.h"
quadcopter_state_enum quadcopter_state_get(void);
volatile quadcopter_state_enum *quadcopter_state_get_ptr(void);
void quadcopter_state_set(quadcopter_state_enum state);
void flight_control_run(bmi088_data_struct *bmi088_data, spl06_data_struct *spl06_data, pid_data_struct *pid_data, state_machine_event_enum queue_sm_evt);

#endif /* FLIGHT_CONTROL_H */