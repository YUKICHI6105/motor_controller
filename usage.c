#include <motor_control.h>
#include <motor_pid.h>
#include <stdint.h>


MotorControlParam motor_param;

int main(void) {
    MotorControl_Init(&motor_param, 0, 1000, 1000, 1000, 10);
    MotorControl_SetMode(&motor_param, MOTOR_MODE_POSITION);
    MotorControl_setTargetAngle(&motor_param, 90);
    while (1)
    {
        
    }
    return 0;
}

void timer_callback(uint32_t angle) {
    MotorControl_SetAngle(&motor_param, angle);
    SetCurrent(MotorControl_Update(&motor_param));//SetCurrent is difinded by users
}