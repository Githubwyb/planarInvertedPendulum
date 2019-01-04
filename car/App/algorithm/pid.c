#include "pid.h"

pid_struct motor_pwm[3];
pid_struct location_pid;
pid_struct balance_pid;
// pid_struct rollRatePid;
// pid_struct pitchRatePid;
// pid_struct rollPid;
// pid_struct pitchPid;

void PID_param_Init(void)
{
    PID_param_Reset(&motor_pwm[0]);
    PID_param_Reset(&motor_pwm[1]);
    PID_param_Reset(&motor_pwm[2]);
    PID_param_Reset(&location_pid);
    PID_param_Reset(&balance_pid);
    // PID_param_Reset(&rollRatePid);
    // PID_param_Reset(&pitchRatePid);
    // PID_param_Reset(&rollPid);
    // PID_param_Reset(&pitchPid);

    motor_pwm[0].kp = 2000;
    motor_pwm[0].ki = 0;
    motor_pwm[0].kd = 100;
    motor_pwm[0].iLimit = 1;
    motor_pwm[0].oLimit = 2000;

    motor_pwm[1].kp = 2000;
    motor_pwm[1].ki = 0;
    motor_pwm[1].kd = 100;
    motor_pwm[1].iLimit = 1;
    motor_pwm[1].oLimit = 2000;

    motor_pwm[2].kp = 2000;
    motor_pwm[2].ki = 0;
    motor_pwm[2].kd = 100;
    motor_pwm[2].iLimit = 1;
    motor_pwm[2].oLimit = 2000;

    location_pid.kp = 0.03;
    location_pid.ki = 0;
    location_pid.kd = 0.008;
    location_pid.iLimit = 100;
    location_pid.oLimit = 0.5;

    balance_pid.kp = 1;
    balance_pid.ki = 0;
    balance_pid.kd = 0.6;
    balance_pid.iLimit = 30;
    balance_pid.oLimit = 1.7;

    // rollPid.kp = 7;
    // rollPid.ki = 0;
    // rollPid.kd = 0;
    // rollPid.iLimit = 20;
    // rollPid.oLimit = 200;

    // pitchPid.kp = 7;
    // pitchPid.ki = 0;
    // pitchPid.kd = 0;
    // pitchPid.iLimit = 20;
    // pitchPid.oLimit = 300;

    // rollRatePid.kp = 0.12;
    // rollRatePid.ki = 0;
    // rollRatePid.kd = 0.06;
    // rollRatePid.iLimit = 20;
    // rollRatePid.oLimit = 1.7;

    // pitchRatePid.kp = 0.12;
    // pitchRatePid.ki = 0;
    // pitchRatePid.kd = 0.06;
    // pitchRatePid.iLimit = 20;
    // pitchRatePid.oLimit = 1.7;
}

/**
*@breif:pid计算
*@param desired:期望
*@param measured:实际测量值
*/
double PID_Compulate(pid_struct* pid, double desired, double measured)
{
    pid->error = desired - measured;
    pid->integ += (pid->error * 0.001f);                 /*integ update*/

    if (pid->integ > pid->iLimit)                            /*limit the integ*/
        pid->integ = pid->iLimit;
    else if (pid->integ < -pid->iLimit)
        pid->integ = -pid->iLimit;

    pid->deriv = pid->error - pid->lastError;

    pid->output = pid->kp * pid->error + pid->ki * pid->integ + pid->kd * pid->deriv;

    if (pid->output > pid->oLimit)                           /*limit the output*/
        pid->output = pid->oLimit;
    else if (pid->output < -pid->oLimit)
        pid->output = -pid->oLimit;

    pid->lastError = pid->error;

    return pid->output;
}


/*****************************************************************************/
/**
*@breif:复位pid参数，将pid参数全部置为0
*/
void PID_param_Reset(pid_struct* pid)
{
    pid->error = 0;
    pid->lastError = 0;
    pid->integ = 0;
    pid->deriv = 0;
    pid->iLimit = 0;
    pid->kp = 0;
    pid->ki = 0;
    pid->kd = 0;
    pid->output = 0;
    pid->oLimit = 0;
}
