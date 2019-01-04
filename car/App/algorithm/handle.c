#include "handle.h"
#include "math.h"

#define PI 3.1415926535
#define SPEED 20.466

extern double rotate_f;
extern double location_n[2];

#define Kp          2.0f
#define Ki          0.005f
#define halfT       0.002f      //100Hz,10ms
#define FACTOR      0.002f

double q0 = 1;
double q1 = 0;
double q2 = 0;
double q3 = 0;

void GET_rotate_By_direction(double angle, double* rotate)
{
    double angle_d = angle / 180 * PI;
    rotate[0] = rotate_f * sin(angle_d);
    rotate[1] = rotate_f * sin(PI / 3 - angle_d);
    rotate[2] = -rotate_f * sin(PI / 3 + angle_d);
}

void GET_rotate_by_balance(double angle, double rotate_f0, double* rotate)
{
    double angle_d = angle / 180 * PI;
    rotate[0] = rotate_f0 * sin(angle_d);
    rotate[1] = rotate_f0 * sin(PI / 3 - angle_d);
    rotate[2] = -rotate_f0 * sin(PI / 3 + angle_d);
}

double GET_speed(void)
{
    return rotate_f * SPEED;
}

double GET_degree(double* location_exp)
{
    double angle_exp = 0;
    angle_exp = atan2(location_exp[0] - location_n[0], location_exp[1] - location_n[1]) / PI * 180;
    return angle_exp;
}

double GET_d2(double* location)
{
    return (location_n[0] - location[0]) * (location_n[0] - location[0]) + (location_n[1] - location[1]) * (location_n[1] - location[1]);
}

void GET_balance_para(double* acce, double* gyro, double* angle, double* rotate_angle)
{
    *angle = -atan2(acce[1], acce[0]) / PI * 180;
    if(acce[3] >= 9.590)
        *rotate_angle = 0;
    else
        *rotate_angle = acos(acce[3] / 9.610) / PI * 180;
}

void Get_angle(double *gyro, double *acce, double *angle)
{
    double norm;
    double vx, vy, vz;
    double ex, ey, ez;

    double delta_2 = 0;

    double ax = acce[0];
    double ay = acce[1];
    double az = acce[2];

    double gx = gyro[0];
    double gy = gyro[1];
    double gz = gyro[2];
    // normalise the measurements
    norm = sqrt(ax * ax + ay * ay + az * az);
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;

    // estimated direction of gravity
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // error is sum of cross product between reference direction of field and direction measured by sensor
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    gx = gx + ex * FACTOR / halfT;
    gy = gy + ey * FACTOR / halfT;
    gz = gz + ez * FACTOR / halfT;

    delta_2 = (2 * halfT*gx)*(2 * halfT*gx) + (2 * halfT*gy)*(2 * halfT*gy) + (2 * halfT*gz)*(2 * halfT*gz);

    q0 = (1 - delta_2 / 8) * q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = (1 - delta_2 / 8) * q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = (1 - delta_2 / 8) * q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = (1 - delta_2 / 8) * q3 + (q0*gz + q1*gy - q2*gx)*halfT;

    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    // roll
    angle[0] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.295780f;
    // pitch
    angle[1] = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.295780f;
}
