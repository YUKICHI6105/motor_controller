/*
 *  motor_control.h
 *
 *  Created on: 2023/09/07
 *      Author: ykc
 */
#include <motor_pid.h>
#include <stdint.h>
#include <math.h>

typedef enum
{
    MOTOR_MODE_DISABLE = 0,
    MOTOR_MODE_CURRENT,
    MOTOR_MODE_VELOCITY,
    MOTOR_MODE_POSITION,
} MotorMode;

typedef struct
{
    int32_t angle;
    int32_t velocity;
    int32_t current;
} MotorParameter;

typedef struct
{
    MotorParameter motor_param;
    MotorParameter target_param;
    MotorMode motor_mode;
    PIDParameter pid_param;
} MotorControlParam;

void MotorControl_Init(MotorControlParam *motor_param, uint16_t init_angle,
                       uint32_t velkp, uint32_t velki, uint32_t poskp, uint16_t control_period);
void MotorControl_SetAngle(MotorControlParam *motor_param, int32_t angle);
void MotorControl_SetVelocity(MotorControlParam *motor_param, int32_t velocity);
void MotorControl_SetCurrent(MotorControlParam *motor_param, int32_t current);
void MotorControl_SetMode(MotorControlParam *motor_param, MotorMode mode);
void MotorControl_setTargetAngle(MotorControlParam *motor_param, int32_t target_angle);
void MotorControl_setTargetVelocity(MotorControlParam *motor_param, int32_t target_velocity);
void MotorControl_setTargetCurrent(MotorControlParam *motor_param, int32_t target_current);
int32_t MotorControl_Update(MotorControlParam *motor_param);
void MotorControl_Reset(MotorControlParam *motor_param);
void MotorControl_SetPID(MotorControlParam *motor_param, uint32_t velkp, uint32_t velki,
                          uint32_t poskp, uint16_t control_period);
void MotorControl_GetStatus(MotorControlParam *motor_param, int32_t *angle,
                              int32_t *velocity, int32_t *current, MotorMode *mode);
void MotorControl_GetTarget(MotorControlParam *motor_param, int32_t *target_angle,
                              int32_t *target_velocity, int32_t *target_current);