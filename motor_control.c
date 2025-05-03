#include <motor_control.h>
#include <motor_pid.h>
#include <stdint.h>

void MotorControl_Init(MoterControlParam *motor_param, uint16_t init_angle,
                        uint32_t velkp, uint32_t velki, uint32_t poskp, uint16_t control_period) {
    motor_param->motor_param.angle = init_angle;
    motor_param->motor_param.velocity = 0;
    motor_param->motor_param.current = 0;
    motor_param->motor_mode = MOTOR_MODE_DISABLE;
    PIDParam_Init(&motor_param->pid_param, velkp, velki, poskp, control_period);
}

void MotorControl_SetAngle(MoterControlParam *motor_param, int32_t angle) {
    motor_param->motor_param.angle = angle;
}

void MotorControl_SetVelocity(MoterControlParam *motor_param, int32_t velocity) {
    motor_param->motor_param.velocity = velocity;
}

void MotorControl_SetCurrent(MoterControlParam *motor_param, int32_t current) {
    motor_param->motor_param.current = current;
}

void MotorControl_SetMode(MoterControlParam *motor_param, MotorMode mode) {
    motor_param->motor_mode = mode;
}

void MotorControl_setTargetAngle(MoterControlParam *motor_param, int32_t target_angle) {
    motor_param->target_param.angle = target_angle;
}
void MotorControl_setTargetVelocity(MoterControlParam *motor_param, int32_t target_velocity) {
    motor_param->target_param.velocity = target_velocity;
}
void MotorControl_setTargetCurrent(MoterControlParam *motor_param, int32_t target_current) {
    motor_param->target_param.current = target_current;
}

int32_t MotorControl_Update(MoterControlParam *motor_param) {
    switch (motor_param->motor_mode) {
        case MOTOR_MODE_CURRENT:
            break;
        case MOTOR_MODE_VELOCITY:
            motor_param->target_param.current = PIDParam_CalculateVel(&motor_param->pid_param,
                motor_param->target_param.velocity, motor_param->motor_param.velocity);
            break;
        case MOTOR_MODE_POSITION:
            motor_param->target_param.current = PIDParam_CalculatePos(&motor_param->pid_param,
                motor_param->target_param.angle, motor_param->motor_param.angle);
            break;
        case MOTOR_MODE_DISABLE:
        default:
            motor_param->target_param.current = 0;
            break;
    }
    return motor_param->target_param.current;
}

void MotorControl_Reset(MoterControlParam *motor_param) {
    motor_param->motor_param.angle = 0;
    motor_param->motor_param.velocity = 0;
    motor_param->motor_param.current = 0;
    motor_param->target_param.angle = 0;
    motor_param->target_param.velocity = 0;
    motor_param->target_param.current = 0;
    motor_param->motor_mode = MOTOR_MODE_DISABLE;
    PIDParam_Reset(&motor_param->pid_param);
}

void MotorControl_SetPID(MoterControlParam *motor_param, uint32_t velkp, uint32_t velki,
                           uint32_t poskp, uint16_t control_period) {
    PIDParam_SetVelKp(&motor_param->pid_param, velkp);
    PIDParam_SetVelKi(&motor_param->pid_param, velki);
    PIDParam_SetPosKp(&motor_param->pid_param, poskp);
    PIDParam_SetControlPeriod(&motor_param->pid_param, control_period);
}

void MotorControl_GetStatus(MoterControlParam *motor_param, int32_t *angle,
                               int32_t *velocity, int32_t *current, MotorMode *mode) {
    *angle = motor_param->motor_param.angle;
    *velocity = motor_param->motor_param.velocity;
    *current = motor_param->motor_param.current;
    *mode = motor_param->motor_mode;
}
void MotorControl_GetTarget(MoterControlParam *motor_param, int32_t *target_angle,
                               int32_t *target_velocity, int32_t *target_current) {
    *target_angle = motor_param->target_param.angle;
    *target_velocity = motor_param->target_param.velocity;
    *target_current = motor_param->target_param.current;
}