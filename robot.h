#pragma once
#include "stdafx.h"
#include "SCServo/SCServo.h"
#include "Leap.h"
#include<cmath>
#include <ctime> 
#include <cstring>
#include <Windows.h>
#include <iostream>

using namespace std;
using namespace Leap;

//=================3*3矩阵相乘=================//
void matrix_multiply3(float c[][3], float a[][3], float b[][3]);

//=================3*1矩阵相乘=================//
void matrix_multiply_3x1(float C[], float A[][3], float B[]);

//=================3*3矩阵求逆=================//
void matrix_inverse3(float B[][3], float A[][3]);

float map(float num, float s1min, float s1max, float s2min, float s2max);

void delay(int time);

class myLeapmotion          //Leapmotion
{
public:
	const Controller controller;

	// leap motion数据：手指数量、抓取半径、 ；末端位置；末端姿态
	float leapdata_L[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	float leapdata_R[3][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
	//数据：pos_R; ang_R; pos_L; ang_L; 爪子右、爪子左
	float data[5][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };
		 
public:
	void ini_leap();
	void getFrame();

private:
	float rot[3][3] = { {1, 0, 0}, {0, 0, -1}, {0, 1, 0} };                      //转换坐标系的旋转矩阵及位移
	float p_move[3] = { 0, 340, -520 };
	float hand_map[4] = { 25, 150, 0, 90 };

private:
    void Init(const Controller&);
    void Connect(const Controller&);
    void Disconnect(const Controller&);
    void Exit(const Controller&);
};


class Driver
{
public:
	float Pi = 3.1415926;
	float pos82B_R[3] = { 0, 0, 0 };                     //末端全局位置
	float ang82B_R[3] = { 0, 0, 0 };                 //末端全局姿态,rz,ry,rx  
	float pos82B_L[3] = { 0, 0, 0 };                     //末端全局位置
	float ang82B_L[3] = { 0, 0, 0 };                 //末端全局姿态,rz,ry,rx 

public:
	void ini1(SMSBL &sm);			//ini1 - 初始化 · 抬起手
	void drive(float data[5][3], SMSBL &sm);

private:
	//=================公共变量=================//
	float armLen[4] = { 270, 240, 100, 160 };                  //定义各杆长

	float pos_R[3] = { 0, 0, 0 };						//左右末端位置、姿态
	float pos_L[3] = { 0, 0, 0 };
	float ang_R[3] = { 0, 0, 0 };
	float ang_L[3] = { 0, 0, 0 };

	float th_R[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };                  //各个关节的角度
	float th_L[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	float th_H[3] = { 0, 0, 0 };
	float th[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	//=================计算过程变量=================//
	float II[3][3] = { {-1, 0, 0}, {0, -1, 0}, {0, 0, -1} };			        //负单位矩阵

	float pos8[3] = { 0, 0, 0 };                   //末端相对手臂坐标{0}位置、姿态
	float rot8[3][3];

	float rotB_R[3][3] = { {0, 0, -1}, {0, 1, 0}, {1, 0, 0} };          //右臂坐标{0}对基坐标旋转矩阵及位移
	float rotB_L[3][3] = { {0, 0, -1}, {0, 1, 0}, {1, 0, 0} };
	float posB_R[3] = { 0, 0, -armLen[3] };
	float posB_L[3] = { 0, 0, armLen[3] };

	float pos5[3] = { 0, 0, 0 };                 //腕关节位置坐标
	float usw[3] = { 0, 0, 0 };                  //原点到腕关节的单位矢量

	float fai_R = 80 * Pi / 180;                    //臂形角φ
	float fai_L = 90 * Pi / 180;

	//==============================读写舵机变量==============================//
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
								   {0,120,3071,1705},
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

private:
	//=================计算、读写=================//
	void clc(float pos82B[], float ang82B[], int isRL);

	int insure(int p[], int isRL);

	void wr(float p[], int is_RL, SMSBL& sm);
};

