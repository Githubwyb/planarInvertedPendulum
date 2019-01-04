#ifndef _HANDLE_H_
#define _HANDLE_H_
#include <stm32f4xx.h>

void GET_rotate_By_direction(double angle, double* rotate);
void GET_rotate_by_balance(double angle, double rotate_f0, double* rotate);
double GET_speed(void);
double GET_degree(double* location_exp);
double GET_d2(double* location);
void GET_balance_para(double* acce, double* gyro, double* angle, double* rotate_angle);
void Get_angle(double *gyro, double *acce, double *angle);

#endif
