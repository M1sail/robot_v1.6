#include "stdafx.h"
#include"robot.h"

extern SMSBL sm;

extern float Pi;

int v = 200;                 //舵机转速、加速度
int a = 20;

int pth_R[8] = { 2047,2047,2047,1023,2047,2047,2047,2047 };                 // 舵机初始位置
int pth_L[8] = { 2047,2047,2047,1023,2047,2047,2047,2047 };
int pth_H[3] = { 2047,2047,2047 };
int pth_R_pre[8] = { 2047,2047,2047,1023,2047,2047,2047,2047 };
int pth_L_pre[8] = { 2047,2047,2047,1023,2047,2047,2047,2047 };
int pth_H_pre[8] = { 2047,2047,2047 };

int ptm[8];
int ptm_pre[8];

int map_R[8][4] = { {-90,90,3071,1023},    //关节角度到舵机位置的映射矩阵
                               {0,100,1023,2161},
                               {0,180,3071,1023},
                               {0,125,2047,625},
                               {-90,90,3071,1023},
                               {70,110,1477,2617},
                               {-75,75,2900,1194},
                               {0,90,2160,1510} };
int map_L[8][4] = { {-90,90,1023,3071},    //关节角度到舵机位置的映射矩阵
                              {80,180,1933,3071},
                              {0,180,3071,1023},
                              {0,125,2047,625},
                              {-90,90,3071,1023},
                              {70,110,1477,2617},
                              {-75,75,2900,1194},
                              {0,90,1950,2600} };
int map_H[3][4] = { {-90,90,3071,1023},    //关节角度到舵机位置的映射矩阵
                               {-20,20,2617,1477},
                               {-22,90,1797,3071} };
int map0[8][4];

void delay(int time)					//单位毫秒
{
    clock_t   now = clock();

    while (clock() - now < time);
}

float map(float num, float s1min, float s1max, float s2min, float s2max)            //把num从s1范围映射到s2范围
{
    float num1;
    num1 = (num - s1min) * (s2max - s2min) / (s1max - s1min) + s2min;
    return num1;
}

int insure(int p[], int isRL)                   //判定是否超出舵机转角范围
{
    int cnt = 0;
    for (int i = 0; i < 8; i++)
    {
        if (abs(p[i] - (map0[i][2] + map0[i][3]) / 2) < abs((map0[i][2] - map0[i][3]) / 2) + 2)
        {
            cnt += pow(10, i);
        }
    }

    if (cnt == 11111111)
        return 1;
    else
    {
        cout << isRL << ":" << cnt << endl;
        return 0;
    }
}


void wr(float pth[], int isRL)
{
    int p[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    int num;

    if (isRL == 0)
    {
        memcpy(map0, map_R, 128);
        num = 1;
    }
    else if (isRL == 1)
    {
        memcpy(map0, map_L, 128);
        num = 11;
    }

    for (int i = 0; i < 8; i++)
    {
        p[i] = int(map(pth[i] * 180 / Pi, map0[i][0], map0[i][1], map0[i][2], map0[i][3]));
    }

    switch (isRL)
    {
    case 0:
        for (int i = 0; i < 8; i++)
        {
            pth_R_pre[i] = pth_R[i];
            pth_R[i] = p[i];
            ptm[i] = pth_R[i];
            ptm_pre[i] = pth_R_pre[i];
        }
        break;
    case 1:
        for (int i = 0; i < 8; i++)
        {
            pth_L_pre[i] = pth_L[i];
            pth_L[i] = p[i];
            ptm[i] = pth_L[i];
            ptm_pre[i] = pth_L_pre[i];
        }
        break;
    }

    if (insure(p, isRL))
    {
        for (int i = 0; i < 8; i++)
        {
            v = 4 * abs(ptm[i] - ptm_pre[i]);    if (v > 1000) { v = 1000; };
            a = 0.1 * v;  if (a < 1) { a = 1; };
            if (v < 10) break;

            if (i == 5) sm.WritePosEx(num + i, p[i], 2 * v, 2 * a);         //舵机ID、目标位置、速度、加速度
            else sm.WritePosEx(num + i, p[i], v, a);
        }
    }
}

//ini1 - 初始化 · 抬起手
void ini1()
{
    int num = 1;

    for (int i = 0; i < 8; i++)
    {
        switch (i)
        {
        case 5:
            sm.WritePosEx(num + i, pth_R[i], 2 * v, 2 * a);
            break;
        default:
            sm.WritePosEx(num + i, pth_R[i], v, a);
            break;
        }
    }

    num = 11;

    for (int i = 0; i < 8; i++)
    {
        switch (i)
        {
        case 5:
            sm.WritePosEx(num + i, pth_L[i], 2 * v, 2 * a);
            break;
        default:
            sm.WritePosEx(num + i, pth_L[i], v, a);
            break;
        }
    }
}