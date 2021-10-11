#include "stdafx.h"
#include"robot.h"
#include <fstream>

SMSBL sm;
myLeapmotion mylp;
Driver dr;

int main(int argc, char** argv)
{
    if (argc < 2) {                  //�����Ƿ����ö˿�
        cout << "argc error!" << endl;
        getchar();
        return 0;
    }
    cout << "serial:" << argv[1] << endl;
    if (!sm.begin(115200, argv[1])) {                  //���鲨���ʼ��˿��Ƿ���ȷ����ʼ�����
        cout << "Failed to init smb motor!" << endl;
        getchar();
        return 0;
    }

	mylp.ini_leap();			//��ʼ��Leapmotion

	dr.ini1(sm);						//��ʼ����е��

	delay(5000);

	int counter = 0;
	bool start = false;

	while (true)
	{
		mylp.getFrame();						//��ȡһ֡����

		if (start == false)
			cout << "̧�����С�֣��ȸ�1��Ҫ��" << endl;

		if (mylp.leapdata_R[0][0] == 1)						//��װʶ������
			counter += 1;
		else counter = 0;

		if (counter == 9)						//�ȡ�1��10֡  
		{
			start = !start;						//�޸�״̬
			counter = 0;
			if (start)
			{
				cout << "Ҫ��ʼ��" << endl;
				cout << "���԰��ָֻ���Ȼ״̬��" << endl;
				cout << "���е㱿���ģ��е������~" << endl;
				delay(3000);
			}
			else
			{
				cout << "������ô���أ�" << endl;
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
			dr.drive(mylp.data, sm);                 //������ؽڽǲ�д����
		}

		delay(150);
	}

	return 0;
}


