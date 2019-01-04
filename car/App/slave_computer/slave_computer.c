#include "slave_computer.h"
#include "lcd.h"
#include "touch.h"
#include "math.h"
#include "string.h"
#include "pid.h"

#define PAGE_MAX 3
#define DATA_X 120
#define NAME_X 20

uint8_t page_now = 1;       //当前页数
uint8_t line_now = 1;       //当前列数
uint8_t data_now = 1;       //当前操作数字位数
uint8_t touch_key = 0;      //当前按下的键值
uint8_t touch_flag = 0;     //触屏检测
uint8_t clear_flag = 1;     //清屏标志位
uint8_t is_touch = 0;       //触屏是否被按下
uint8_t start = 0;          //开始信号
uint8_t data_line[14] = {5,20,35,50,65,80,95,110,125,140,155,170,185,200};  //行y值
uint8_t clear = 0;          //清楚标志位
uint8_t name[13][255];      //变量名数组

uint8_t mode = 0;               //模式选择
int32_t motor_pwm_n[3] = {0,0,0};       //当前电机的pwm
double mpu_rotate[3];           //mpu6050角速度
double mpu_acce[3];             //mpu6050加速度
double rotate_motor_exp[3] = {0,0,0};   //电机期望角速度
double rotate_f = 0.5;          //转速力度
double angle = 30;               //角度
uint32_t time_start = 0;
double location_n[2] = {0,0};
double location_exp[2] = {0,0};

extern double rotate_motor[3];    //电机当前角速度
extern int32_t encoder0_count;  //编码器0总码数
extern int32_t encoder1_count;  //编码器1总码数
extern int32_t encoder2_count;  //编码器2总码数
extern pid_struct motor_pwm[3];
extern pid_struct location_pid;
extern pid_struct balance_pid;
extern uint32_t time_4ms;
extern double angle_balance[2];

void Page_1(void)
{
    static uint8_t n_data = 7, n_sum = 10;
    int32_t data[13][3];

    strcpy((char*)name[0], "mode:");
    strcpy((char*)name[1], "start:");
    strcpy((char*)name[2], "clear:");
    strcpy((char*)name[3], "x_exp(mm):");
    strcpy((char*)name[4], "y_exp(mm):");
    strcpy((char*)name[5], "angle:");
    strcpy((char*)name[6], "rotate_f:");
    strcpy((char*)name[7], "x_n(mm):");
    strcpy((char*)name[8], "y_n(mm):");
    strcpy((char*)name[9], "time(ms):");

    if(mode >= 2)
        n_data = 5;
    else
        n_data = 7;

    if(start)
        time_start = time_4ms;
    else
        time_4ms = 0;

    data[0][0] = mode;
    data[0][1] = 3;
    data[0][2] = 0;

    data[1][0] = start;
    data[1][1] = 1;
    data[1][2] = 0;

    data[2][0] = clear;
    data[2][1] = 1;
    data[2][2] = 0;

    data[3][0] = location_exp[0] * 10;
    data[3][1] = 10000;
    data[3][2] = -10000;

    data[4][0] = location_exp[1] * 10;
    data[4][1] = 10000;
    data[4][2] = -10000;

    data[5][0] = angle;
    data[5][1] = 180;
    data[5][2] = -180;

    data[6][0] = rotate_f * 1000;
    data[6][1] = 30000;
    data[6][2] = -30000;

    data[7][0] = location_n[0] * 10;
    data[8][0] = location_n[1] * 10;
    data[9][0] = time_start * 4;

    Page(name, data, n_data, n_sum - n_data);

    mode = data[0][0];

    start = data[1][0];

    clear = data[2][0];

    location_exp[0] = data[3][0] / 10.0;

    location_exp[1] = data[4][0] / 10.0;

    angle = data[5][0];

    rotate_f = data[6][0] / 1000.0;

    if(clear)
    {
        location_n[0] = 0;
        location_n[1] = 0;
    }

    if(!start)
    {
        motor_pwm[0].integ = 0;
        motor_pwm[1].integ = 0;
        motor_pwm[2].integ = 0;
    }

    if(line_now != 2)
        start = 0;

    if(line_now != 3)
        clear = 0;
}

void Page_2(void)
{
    uint8_t n = 3;
    int32_t data[11][3];

    strcpy((char*)name[0], "balance_p:");
    strcpy((char*)name[1], "balance_i:");
    strcpy((char*)name[2], "balance_d:");
    strcpy((char*)name[3], "acce_x:");
    strcpy((char*)name[4], "acce_y:");
    strcpy((char*)name[5], "acce_z:");
    strcpy((char*)name[6], "gyro_x:");
    strcpy((char*)name[7], "gyro_y:");
    strcpy((char*)name[8], "gyro_z:");
    strcpy((char*)name[9], "angle_0:");
    strcpy((char*)name[10], "angle_1:");

    data[0][0] = balance_pid.kp * 1000;
    data[0][1] = 1000000;
    data[0][2] = -1000000;

    data[1][0] = balance_pid.ki * 1000;
    data[1][1] = 1000000;
    data[1][2] = -1000000;

    data[2][0] = balance_pid.kd * 1000;
    data[2][1] = 1000000;
    data[2][2] = -1000000;

    data[3][0] = mpu_acce[0] * 1000;
    data[4][0] = mpu_acce[1] * 1000;
    data[5][0] = mpu_acce[2] * 1000;
    data[6][0] = mpu_rotate[0] * 1000;
    data[7][0] = mpu_rotate[1] * 1000;
    data[8][0] = mpu_rotate[2] * 1000;
    data[9][0] = angle_balance[0] * 1000;
    data[10][0] = angle_balance[1] * 1000;

    Page(name, data, n, 11 - n);

    balance_pid.kp = data[0][0] / 1000.0;
    balance_pid.ki = data[1][0] / 1000.0;
    balance_pid.kd = data[2][0] / 1000.0;
}

void Page_3(void)
{
    static uint8_t n_data = 6, n_sum = 12;
    int32_t data[13][3];

    strcpy((char*)name[0], "mode:");
    strcpy((char*)name[1], "start:");
    strcpy((char*)name[2], "clear:");
    strcpy((char*)name[3], "angle:");
    strcpy((char*)name[4], "rotate_f:");
    strcpy((char*)name[5], "rotate0_exp:");
    strcpy((char*)name[6], "rotate1_exp:");
    strcpy((char*)name[7], "rotate2_exp:");
    strcpy((char*)name[8], "rotate0_n:");
    strcpy((char*)name[9], "rotate1_n:");
    strcpy((char*)name[10], "rotate2_n:");
    strcpy((char*)name[11], "time:");

    if(start)
        time_start = time_4ms;
    else
        time_4ms = 0;

    data[0][0] = mode;
    data[0][1] = 3;
    data[0][2] = 0;

    data[1][0] = start;
    data[1][1] = 1;
    data[1][2] = 0;

    data[2][0] = clear;
    data[2][1] = 1;
    data[2][2] = 0;

    data[3][0] = angle;
    data[3][1] = 180;
    data[3][2] = -180;

    data[4][0] = rotate_f * 1000;
    data[4][1] = 30000;
    data[4][2] = -30000;

    data[5][0] = rotate_motor_exp[0] * 1000;
    data[5][1] = 3000;
    data[5][2] = -3000;

    data[6][0] = rotate_motor_exp[1] * 1000;
    data[6][1] = 3000;
    data[6][2] = -3000;

    data[7][0] = rotate_motor_exp[2] * 1000;
    data[7][1] = 3000;
    data[7][2] = -3000;

    data[8][0] = rotate_motor[0] * 1000;
    data[9][0] = rotate_motor[1] * 1000;
    data[10][0] = rotate_motor[2] * 1000;

    data[11][0] = time_start;

    Page(name, data, n_data, n_sum - n_data);

    mode = data[0][0];

    start = data[1][0];

    clear = data[2][0];

    angle = data[3][0];

    rotate_f = data[4][0] / 1000.0;

    rotate_motor_exp[0] = data[5][0] / 1000.0;

    rotate_motor_exp[1] = data[6][0] / 1000.0;

    rotate_motor_exp[2] = data[7][0] / 1000.0;

    if(mode == 0)
        n_data = 8;
    else if(mode == 1)
        n_data = 5;
    else if(mode == 2)
        n_data = 3;

    if(clear)
        ;

    if(!start)
    {
        motor_pwm[0].integ = 0;
        motor_pwm[1].integ = 0;
        motor_pwm[2].integ = 0;
    }

    if(line_now != 2)
        start = 0;

    if(line_now != 3)
        clear = 0;
}

void SHOW_Pages(void)
{
    touch_key = GET_touch_key();

    if(is_touch && !touch_flag)
    {
        clear_flag = 1;
        LCD_Clear(WHITE);
    }
    
    is_touch = touch_flag;

    switch(touch_key)
    {
        case 1:
            if(page_now != 1)
                page_now--;
            line_now = 1;
            data_now = 1;
            start = 0;
            break;
        case 3:
            if(page_now != PAGE_MAX)
                page_now++;
            line_now = 1;
            data_now = 1;
            start = 0;
            break;
        case 5:
            if(start == 0)
                start = 1;
            else
                start = 0;
            break;
        default:
            break;
    }

    switch(page_now)
    {
        case 1:
            Page_1();
            break;
        case 2:
            Page_2();
            break;
        case 3:
            Page_3();
            break;
        // case 4:
        //     Page_4();
        //     break;
        // case 5:
        //     Page_5();
        //     break;
        // case 6:
        //     Page_6();
        //     break;
        default:
            break;
    }

    if(clear_flag)
    {
        clear_flag = 0;
        // LCD_Fill(5, 205, 75, 235, YELLOW);
        // LCD_Fill(85, 205, 155, 235, YELLOW);
        // LCD_Fill(165, 205, 235, 235, YELLOW);
        // LCD_Fill(5, 245, 75, 275, YELLOW);
        // LCD_Fill(85, 245, 155, 275, YELLOW);
        // LCD_Fill(165, 245, 235, 275, YELLOW);
        // LCD_Fill(5, 285, 75, 315, YELLOW);
        // LCD_Fill(85, 285, 155, 315, YELLOW);
        // LCD_Fill(165, 285, 235, 315, YELLOW);

        LCD_DrawRectangle(5, 205, 75, 235);
        LCD_DrawRectangle(85, 205, 155, 235);
        LCD_DrawRectangle(165, 205, 235, 235);
        LCD_DrawRectangle(5, 245, 75, 275);
        LCD_DrawRectangle(85, 245, 155, 275);
        LCD_DrawRectangle(165, 245, 235, 275);
        LCD_DrawRectangle(5, 285, 75, 315);
        LCD_DrawRectangle(85, 285, 155, 315);
        LCD_DrawRectangle(165, 285, 235, 315);

        LCD_ShowString(8, 212, 70, 16, 16, "Page_pre");
        LCD_ShowString(88, 212, 70, 16, 16, "line_pre");
        LCD_ShowString(168, 212, 70, 16, 16, "Page_nex");
        LCD_ShowString(8, 252, 70, 16, 16, "data_pre");
        LCD_ShowString(112, 252, 70, 16, 16, "OK");
        LCD_ShowString(168, 252, 70, 16, 16, "data_nex");
        LCD_ShowString(37, 292, 70, 16, 16, "-");
        LCD_ShowString(88, 292, 70, 16, 16, "line_nex");
        LCD_ShowString(197, 292, 70, 16, 16, "+");
        
        LCD_ShowString(NAME_X,data_line[0],70,12,12,"Page:");
        LCD_ShowNum(DATA_X, data_line[0], page_now, 2, 12);
        LCD_ShowString(NAME_X,data_line[1],70,12,12,"data_plus:");
        LCD_ShowNum(DATA_X, data_line[1], data_now, 2, 12);
        LCD_ShowChar(5, data_line[line_now + 1], '>', 12, 0);
    }
}

void Page(uint8_t name[][255], int32_t data[][3], uint8_t n_data, uint8_t n_show)
{
    uint8_t i = 0;

    for(i = 0;i < n_show;i++)
        LCD_ShowInt(DATA_X, data_line[i + 2 + n_data], data[n_data + i][0], 10, 12);

    switch(touch_key)
    {
        case 2:
            if(line_now != 1)
                line_now--;
            break;
        case 8:
            if(line_now != n_data)
                line_now++;
            break;
        default:
            break;
    }

    data_op(data[line_now - 1], data[line_now - 1][1], data[line_now - 1][2]);

    if(clear_flag)
    {
        for(i = 0;i < n_data + n_show;i++)
            LCD_ShowString(NAME_X,data_line[i + 2],70,12,12,name[i]);

        for(i = 0;i < n_data;i++)
            LCD_ShowInt(DATA_X, data_line[i + 2], data[i][0], 10, 12);
    }
}

uint8_t GET_touch_key(void)
{
//    static uint8_t touch_up = 1;
    static uint8_t key;
    uint16_t location[2];
    touch_flag = GET_TP_XY(location);
    if(!is_touch && touch_flag)
    {
        key = 0;
        if(location[0] > 5 && location[0] < 75 && location[1] > 205 && location[1] < 235)
        {
            key = 1;
            LCD_Fill(5, 205, 75, 235, RED);
        }
        if(location[0] > 85 && location[0] < 155 && location[1] > 205 && location[1] < 235)
        {
            key = 2;
            LCD_Fill(85, 205, 155, 235, RED);
        }
        if(location[0] > 165 && location[0] < 235 && location[1] > 205 && location[1] < 235)
        {
            key = 3;
            LCD_Fill(165, 205, 235, 235, RED);
        }
        if(location[0] > 5 && location[0] < 75 && location[1] > 245 && location[1] < 275)
        {
            key = 4;
            LCD_Fill(5, 245, 75, 275, RED);
        }
        if(location[0] > 85 && location[0] < 155 && location[1] > 245 && location[1] < 275)
        {
            key = 5;
            LCD_Fill(85, 245, 155, 275, RED);
        }
        if(location[0] > 165 && location[0] < 235 && location[1] > 245 && location[1] < 275)
        {
            key = 6;
            LCD_Fill(165, 245, 235, 275, RED);
        }
        if(location[0] > 5 && location[0] < 75 && location[1] > 285 && location[1] < 315)
        {
            key = 7;
            LCD_Fill(5, 285, 75, 315, RED);
        }
        if(location[0] > 85 && location[0] < 155 && location[1] > 285 && location[1] < 315)
        {
            key = 8;
            LCD_Fill(85, 285, 155, 315, RED);
        }
        if(location[0] > 165 && location[0] < 235 && location[1] > 285 && location[1] < 315)
        {
            key = 9;
            LCD_Fill(165, 285, 235, 315, RED);
        }
    }

    if(is_touch && !touch_flag)
        return key;

    return 0;
}

void data_op(int32_t* data, int32_t data_max, int32_t data_min)
{
    int32_t plus;
    plus = pow(10, data_now - 1);
    switch(touch_key)
    {
        case 4:
            if(pow(10, data_now) <= data_max)
                data_now++;
            break;
        case 6:
            if(data_now != 1)
                data_now--;
            break;
        case 7:
            *data -= plus;
            break;
        case 9:
            *data += plus;
            break;
        default:
            break;
    }
    if(*data >= data_max)
        *data = data_max;
    else if(*data <= data_min)
        *data = data_min;
}
