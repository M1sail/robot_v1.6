#include "stdafx.h"
#include"robot.h"
#include <fstream>

SMSBL sm;
Controller controller;
myLeapmotion mylp;
//ofstream ofs;

//isRL  0右1左
extern float Pi;
extern int map_R[8][4];

float th_R[] = { 0, 0, 0, 0, 0, 0, 0, 0 };                  //各个关节的角度
float th_L[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
float th_H[] = { 0, 0, 0 };
float th[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

float pos82B_R[] = { 0, 0, 0 };                     //末端全局位置
float ang82B_R[] = { 0, 0, 0 };                 //末端全局姿态,rz,ry,rx  
float pos82B_L[] = { 0, 0, 0 };                     //末端全局位置
float ang82B_L[] = { 0, 0, 0 };                 //末端全局姿态,rz,ry,rx 

// leap motion数据：手指数量、抓取半径、 ；末端位置；末端姿态
float leapdata_L[3][3] = { {0, 0, 0}, {0, 0, 0},{0, 0, 0} };
float leapdata_R[3][3] = { {0, 0, 0}, {0, 0, 0},{0, 0, 0} };

float rot[][3] = { {1, 0, 0}, {0, 0, -1}, {0, 1, 0} };                      //转换坐标系的旋转矩阵及位移
float p_move[] = { 0, 340, -570 };
float hand_map[] = { 25, 150, 0, 90 };


void lp_run_robot()
{
	int counter = 0;
	bool start = false;
	//ofs.open("test.txt", ios::out);

	while (true)
	{
		mylp.getFrame(controller, leapdata_L, leapdata_R);						//获取一帧数据
		/*for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
				cout << leapdata[i][j] << "  ";
			cout << endl;
		}*/
		if (start == false)
			cout << "抬好你的小手，比个1不要动" << endl;

		if (leapdata_R[0][0] == 1)						//假装识别手势
			counter += 1;
		else counter = 0;

		if (counter == 9)						//比“1”10帧  
		{
			start = !start;						//修改状态
			counter = 0;
			if (start)
			{
				cout << "要开始咯" << endl;
				delay(3000);
			}
			else
			{
				cout << "不玩了么￣へ￣" << endl;
				delay(3000);
				break;
			}
		}

		if (start)
		{
			if (leapdata_R[0][1] < 30) th[7] = 10 * Pi / 180;
			else if (30 <= leapdata_R[0][1] && leapdata_R[0][1] < 90) th[7] = 60 * Pi / 180;
			else if (leapdata_R[0][1] >= 90) th[7] = 80 * Pi / 180;

			for (int i = 0; i < 8; i++) { cout << th[i] << "; "; }
			cout << endl;

			int p = int(map(th[7] * 180 / Pi, map_R[7][0], map_R[7][1], map_R[7][2], map_R[7][3]));
			sm.WritePosEx(8, p, 600, 60);
		}

		delay(200);
	}
	//ofs.close();
}


int main(int argc, char** argv)
{
	if (argc < 2) {                  //检验是否设置端口
		cout << "argc error!" << endl;
		getchar();
		return 0;
	}
	cout << "serial:" << argv[1] << endl;
	if (!sm.begin(115200, argv[1])) {                  //检验波特率及端口是否正确
		cout << "Failed to init smb motor!" << endl;
		getchar();
		return 0;
	}
	//================================以上为初始化舵机================================\\

	mylp.Init(controller);

	mylp.Connect(controller);

	mylp.Disconnect(controller);
	//=============================以上为初始化leap motion=============================\\

	ini1();						//初始化机械臂
	delay(5000);

	lp_run_robot();

	return 0;
}
