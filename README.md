# モーター制御ライブラリ
想定:Arduino,stm32

## usage

```C
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

void timer_callback(uint32_t angle) {//timer_callback is defined by users.
    MotorControl_SetAngle(&motor_param, angle);
    SetCurrent(MotorControl_Update(&motor_param));//SetCurrent is defined by users
}
```

## Attention
MotorControlParamはグローバル空間に置かなければならない
制御周期は単位が決まっていないが、msを推奨する(1msなど)