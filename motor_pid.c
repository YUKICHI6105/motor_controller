#include <motor_pid.h>

void PIDParam_Init(PIDParameter *pid_param, uint32_t velkp, uint32_t velki,
                     uint32_t poskp, uint16_t control_period) {
    pid_param->velKp = velkp;
    pid_param->velKi = velki;
    pid_param->posKp = poskp;
    pid_param->control_period = control_period;
    pid_param->vel_e_pre = 0;
    pid_param->vel_du_pre = 0;
}
int32_t PIDParam_CalculateVel(PIDParameter *pid_param, int32_t target, int32_t velocity) {
    int32_t e = target - velocity;
    int32_t de = (e - pid_param->vel_e_pre) / pid_param->control_period;
    int32_t du = pid_param->velKp * de + pid_param->velKi * e;
    int32_t output = (du + pid_param->vel_du_pre) / 2 * pid_param->control_period;
    pid_param->vel_du_pre = du;
    pid_param->vel_e_pre = e;
    return output;
}
int32_t PIDParam_CalculatePos(PIDParameter *pid_param, int32_t target, int32_t angle) {
    return PIDParam_CalculateVel(pid_param, target, pid_param->posKp * (target - angle));
}
void PIDParam_SetVelKp(PIDParameter *pid_param, uint32_t velkp) {
    pid_param->velKp = velkp;
}
void PIDParam_SetVelKi(PIDParameter *pid_param, uint32_t velki) {
    pid_param->velKi = velki;
}
void PIDParam_SetPosKp(PIDParameter *pid_param, uint32_t poskp) {
    pid_param->posKp = poskp;
}
void PIDParam_SetControlPeriod(PIDParameter *pid_param, uint16_t control_period) {
    pid_param->control_period = control_period;
}

void PIDParam_Reset(PIDParameter *pid_param) {
    pid_param->vel_e_pre = 0;
    pid_param->vel_du_pre = 0;
}