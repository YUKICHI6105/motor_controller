/*
 *  motor_pid.h
 *
 *  Created on: 2023/09/07
 *      Author: ykc
 */
#include <stdint.h>

typedef struct
{
    uint32_t velKp;
    uint32_t velKi;
    int32_t vel_e_pre;
    int32_t vel_du_pre;
    uint32_t posKp;
    uint16_t control_period;
} PIDParameter;

void PIDParam_Init(PIDParameter *pid_param, uint32_t velkp, uint32_t velki,
                    uint32_t poskp, uint16_t control_period);
int32_t PIDParam_CalculateVel(PIDParameter *pid_param, int32_t target, int32_t velocity);
int32_t PIDParam_CalculatePos(PIDParameter *pid_param, int32_t target, int32_t angle);
void PIDParam_SetVelKp(PIDParameter *pid_param, uint32_t velkp);
void PIDParam_SetVelKi(PIDParameter *pid_param, uint32_t velki);
void PIDParam_SetPosKp(PIDParameter *pid_param, uint32_t poskp);
void PIDParam_SetControlPeriod(PIDParameter *pid_param, uint16_t control_period);
void PIDParam_Reset(PIDParameter *pid_param);