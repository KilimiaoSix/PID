#include <iostream>
#include <iomanip>
#include "VAGUE_PID.hpp"
//#include "PID.hpp"
//using namespace std;
//int main()
//{
//	double output = 0;
//	int count = 0;
//	PID pid_ted(0.2,0.015,0.2);
//	pid_ted.SET_Alpha(0.9);
//	pid_ted.SET_D_Incompleter(DISABLE);
//	for (count = 0; count < 1000; count++)
//	{
//		pid_ted.set_goalval(200.0);
//		output = pid_ted.get_PID_control_output();
//		pid_ted.set_realval(output + pid_ted.get_real_val());
//		cout << "output: " <<  setiosflags(ios::fixed) << setprecision(3) << output << endl;
//	}
//}
/*
ʹ�÷������ȴ���һ��PID���󣬱�֤��������ʹ��֮��
		  ����VAGUE_PID��������ERR���ֵ��DERR���ֵ��PID��ϵ���������ֵ��PID�������ֵ
		  ֮�����ù������ʹ����VAGUE_PID.hpp���洴���õ����鼴��
		   ����ģ���������ʹ��VAGUE_PID.hpp���洴���õļ���
		  ֮�����UPDATE_REAL_DATA��������ʵ�����ݣ�����Get_VAGUE_PID_OUTPUT������ȡ�������
		  ��������Ŀ��ֵ������SET_Goal_Val��������
		  main������Ϊʹ��ʵ��
		  ��Ҫ���Բ�������û��~
*/
int main(void)
{
	double realval = 0;
	double output;
	double e_mf_paras[] = { -3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3 };
	double de_mf_paras[] = { -3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3 };
	double Kp_mf_paras[] = { -3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3 };
	double Ki_mf_paras[] = { -3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3 };
	double Kd_mf_paras[] = { -3,-3,-2,-3,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,3,2,3,3 };
	PID p_test(0.01, 0.04 ,0.01);
	VAGUE_PID pt(p_test, 1500, 650, 0.3, 0.9, 0.6, 10, 10, 10);
	pt.SETRuleMatrix(deltaKpMatrix, deltaKiMatrix, deltaKdMatrix);
	pt.setMf_sub(TRIMF, e_mf_paras, TYPE_ERR);
	pt.setMf_sub(TRIMF, de_mf_paras, TYPE_DE);
	pt.setMf_sub(TRIMF, Kp_mf_paras, TYPE_KP);
	pt.setMf_sub(TRIMF, Ki_mf_paras, TYPE_KI);
	pt.setMf_sub(TRIMF, Kd_mf_paras, TYPE_KD);
	pt.showInfo();
	system("pause");
	int i = 0;
	pt.SET_Goal_Val(600);
	for (i = 0; i < 1000; i++)
	{
		pt.UPDATE_REAL_DATA(realval);
		output = pt.Get_VAGUE_PID_OUTPUT();
		realval += output;
		std::cout << "output: " << output << "realval: " << realval << std::endl;
	}
	system("pause");
	pt.showInfo();
	return 0;

}
