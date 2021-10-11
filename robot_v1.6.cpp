#include "stdafx.h"
#include"robot.h"
#include <fstream>

SMSBL sm;
myLeapmotion mylp;
Driver dr;

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

	mylp.ini_leap();			//初始化Leapmotion

	dr.ini1(sm);						//初始化机械臂

	delay(5000);

	int counter = 0;
	bool start = false;

	while (true)
	{
		mylp.getFrame();						//获取一帧数据

		if (start == false)
			cout << "抬好你的小手，比个1不要动" << endl;

		if (mylp.leapdata_R[0][0] == 1)						//假装识别手势
			counter += 1;
		else counter = 0;

		if (counter == 9)						//比“1”10帧  
		{
			start = !start;						//修改状态
			counter = 0;
			if (start)
			{
				cout << "要开始咯" << endl;
				cout << "可以把手恢复自然状态啦" << endl;
				cout << "它有点笨笨的，有点耐心喔~" << endl;
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
			//for (int i = 0; i < 5; i++)
			//{
			//	for (int j = 0; j < 3; j++)
			//		cout << mylp.data[i][j] << "; ";
			//	cout << endl;
			//}
			dr.drive(mylp.data, sm);                 //计算各关节角并写入舵机
		}

		delay(150);
	}

	return 0;
}


