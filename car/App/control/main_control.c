#include "main_control.h"
#include "math.h"

#define PI 3.1415926535

extern double mpu_rotate[3];
extern double mpu_acce[3];
extern uint8_t mode;               //模式选择
extern uint8_t start;
extern int32_t motor_pwm_n[3];       //当前电机0的pwm
extern double rotate_motor_exp[3];   //电机期望角速度
extern double rotate_motor[3];       //电机0当前角速度
extern pid_struct motor_pwm[3];
extern pid_struct location_pid;
extern pid_struct balance_pid;
// extern pid_struct rollRatePid;
// extern pid_struct pitchRatePid;
// extern pid_struct rollPid;
// extern pid_struct pitchPid;
extern double rotate_f;            //转速力度
extern double angle;               //角度
extern uint8_t key_usart;
extern uint8_t clear_flag;     //清屏标志位
extern double location_n[2];
extern double location_exp[2];
double angle_balance[2];
double angle_erro = -8;

void Control_4ms(void)
{
    double rotate_angle;
    double gyro_g[3];
    // double data_0[3];
    // double data_1[3];
    MPU_Get_param(mpu_rotate, mpu_acce);

    gyro_g[0] = mpu_rotate[0] / 180 * PI; //角度换算成弧度
    gyro_g[1] = mpu_rotate[1] / 180 * PI; //角度换算成弧度
    gyro_g[2] = mpu_rotate[2] / 180 * PI; //角度换算成弧度

    switch(mode)
    {
        case 0:
            break;
        case 1:
            GET_rotate_By_direction(angle, rotate_motor_exp);
            break;
        case 2:
            angle = GET_degree(location_exp);
            rotate_f = PID_Compulate(&location_pid ,0 , -GET_d2(location_exp));
            GET_rotate_By_direction(angle, rotate_motor_exp);
            break;
        case 3:
            Get_angle(gyro_g, mpu_acce, angle_balance);

            if(angle_balance[0] > 30 || angle_balance[0] < -30 || angle_balance[1] > 30 || angle_balance[1] < -30)
                start = 0;

            if(angle_balance[0] >= 0 && angle_balance[1] >= 0)
                angle = atan(tan(angle_balance[0] / 180 * PI) / tan(angle_balance[1] / 180 * PI)) / PI * 180 - 180;
            if(angle_balance[0] >= 0 && angle_balance[1] <= 0)
                angle = -atan(tan(angle_balance[0] / 180 * PI) / tan(-angle_balance[1] / 180 * PI)) / PI * 180;
            if(angle_balance[0] <= 0 && angle_balance[1] <= 0)
                angle = atan(tan(-angle_balance[0] / 180 * PI) / tan(-angle_balance[1] / 180 * PI)) / PI * 180;
            if(angle_balance[0] <= 0 && angle_balance[1] >= 0)
                angle = 180 - atan(-tan(angle_balance[0] / 180 * PI) / tan(angle_balance[1] / 180 * PI)) / PI * 180;

            angle -= angle_erro;
            if(angle < -180)
                angle += 360;
            else if(angle > 180)
                angle -= 360;

            rotate_angle = acos(1 / sqrt(tan(angle_balance[0] / 180 * PI) * tan(angle_balance[0] / 180 * PI) + tan(angle_balance[1] / 180 * PI) * tan(angle_balance[1] / 180 * PI) + 1)) / PI * 180;
            rotate_f = PID_Compulate(&balance_pid ,0 , -rotate_angle * rotate_angle / 4);
            GET_rotate_By_direction(angle, rotate_motor_exp);
            break;
        default:
            break;
    }

    if(start)
    {
        motor_pwm_n[0] += PID_Compulate(&motor_pwm[0], rotate_motor_exp[0], rotate_motor[0]);
        motor_pwm_n[1] += PID_Compulate(&motor_pwm[1], rotate_motor_exp[1], rotate_motor[1]);
        motor_pwm_n[2] += PID_Compulate(&motor_pwm[2], rotate_motor_exp[2], rotate_motor[2]);
 //         + 86.60254 * pitchRatePid.output
 // - 86.60254 * pitchRatePid.output
        if(motor_pwm_n[0] >= 3800)
            motor_pwm_n[0] = 3800;
        else if(motor_pwm_n[0] <= -3800)
            motor_pwm_n[0] = -3800;

        if(motor_pwm_n[1] >= 3800)
            motor_pwm_n[1] = 3800;
        else if(motor_pwm_n[1] <= -3800)
            motor_pwm_n[1] = -3800;

        if(motor_pwm_n[2] >= 3800)
            motor_pwm_n[2] = 3800;
        else if(motor_pwm_n[2] <= -3800)
            motor_pwm_n[2] = -3800;

        MOTOR0_Set(motor_pwm_n[0]);
        MOTOR1_Set(motor_pwm_n[1]);
        MOTOR2_Set(motor_pwm_n[2]);
        if(mode)
        {
            location_n[0] += sin(angle) * GET_speed() * 0.004;
            location_n[1] += cos(angle) * GET_speed() * 0.004;
        }
    }
    else
    {
        MOTOR0_Set(0);
        MOTOR1_Set(0);
        MOTOR2_Set(0);
    }



    LED_Change(LED_0);
}

void Control_main(void)
{
}
