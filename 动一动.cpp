#include "stdafx.h"
#include"robot.h"

SMSBL sm;
myLeapmotion mylp;
Driver dr;

//数据：pos_R; ang_R; pos_L; ang_L; 爪子右、爪子左
float data[5][3] = { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0} };

int main(int argc, char** argv)
{
	if (argc < 2) {                  //检验是否设置端口
		cout << "argc error!" << endl;
		getchar();
		return 0;
	}
	cout << "serial:" << argv[1] << endl;
	if (!sm.begin(115200, argv[1])) {                  //检验波特率及端口是否正确并初始化舵机
		cout << "Failed to init smb motor!" << endl;
		getchar();
		return 0;
	}

	dr.ini1(sm);						//初始化机械臂

	delay(5000);

	float tmax = 16;

	while (true)
	{
		for (float t = 0.00; t < tmax; t += 0.2)
		{
			cout << t << endl;                 //显示时间

			mylp.data[0][0] = 160 + 0.5 * 100 * cos(t * PI / 2) - 20;
			mylp.data[0][1] = 300 + 100 * sin(t * PI / 2);
			mylp.data[0][2] = -270;

			mylp.data[1][0] = -160 - 0.5 * 0 * cos(t * PI / 2) + 20;
			mylp.data[1][1] = 300 - 100 * sin(t * PI / 2);
			mylp.data[1][2] = -270;

			dr.drive(mylp.data, sm);                 //计算各关节角并写入舵机
		}

		delay(150);
	}

	return 0;
}


