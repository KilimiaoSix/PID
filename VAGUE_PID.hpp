#pragma once
#include "PID.hpp"
#include "mymath.hpp"
#define N 7
extern int deltaKpMatrix[N][N];
extern int deltaKiMatrix[N][N];
extern int deltaKdMatrix[N][N];
extern double e_mf_paras[];
extern double de_mf_paras[];
extern double Kp_mf_paras[];
extern double Ki_mf_paras[];
extern double Kd_mf_paras[];
typedef int TYPE_MODE;
class VAGUE_PID
{
public:
	VAGUE_PID(PID &pid_t,double emax_t, double de_max_t,double delta_Kp_max_t, double delta_Ki_max_t, double delta_Kd_max_t,double KP_MAX_T,double KI_MAX_T,double KD_MAX_T);  //ѵ���õ�PID
	~VAGUE_PID(void);
	void SETRuleMatrix(int kp_m[N][N], int ki_m[N][N], int kd_m[N][N]);
	void setMf_sub(TYPE type, double* paras, TYPE_MODE n);  //����ѡ��ĺ������ͣ�typeΪ�������ͣ�parasΪ����������main��������д�õ��ֳɵļ��ɣ�nΪѡ�����õĶ��󣬲ο�����
	double Get_VAGUE_PID_OUTPUT(void);
	void SET_Goal_Val(double goalval_t)  //�޸�Ŀ��ֵ������ʱ��������ӿھͿ�����
	{
		goalval = goalval_t;
	}
	void UPDATE_REAL_DATA(double realval_t)  
	{
		realval = realval_t;
	}
	void showInfo(void);
protected:
	double goalval;			//����Ŀ��ֵ����Ҫ�ﵽ����������
	double realval;			//��ǰʵ��ֵ����������õ���������
	double Kp;				//Kp ����ϵ��
	double Ki;				//Ki ����ϵ��
	double Kd;				//Kd ΢��ϵ��
	double err;				//ƫ��ֵ ����ϵͳ�����Զ����㵱ǰƫ����
	double lasterr;			//ǰһ�ε�ƫ����
	double preerr;			//ǰ���ε�ƫ����
	double derr;			//ƫ������΢��
	double output;			//�����������
	double result;			//���������
	double* e_mf_paras;		//���������Ⱥ����Ĳ���
	double* de_mf_paras;	//����ƫ�������Ⱥ����Ĳ���
	double* Kp_mf_paras;	//kp�������Ⱥ����Ĳ���
	double* Ki_mf_paras;	//ki�������Ⱥ����Ĳ���
	double* Kd_mf_paras;	//kd�������Ⱥ����Ĳ���
	double Ke;				//Ke=n/emax,��������Ϊ[-3,-2,-1,0,1,2,3]
	double Kde;				//Ke=n/emax,��������Ϊ[-3,-2,-1,0,1,2,3]
	double Ku_p;			//Ku_p=Kpmax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
	double Ku_i;			//Ku_i=Kimax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
	double Ku_d;			//Ku_d=Kdmax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
	double delta_Kp_MAX;	//һ���������ӻ��߼��ٵ����Kp��
	double delta_Ki_MAX;	//һ���������ӻ��߼��ٵ����Ki��
	double delta_Kd_MAX;	//һ���������ӻ��߼��ٵ����Kd��
	double KpMAX;			//����Kp�����ֵ
	double KiMAX;			//����Ki�����ֵ
	double KdMAX;			//����Kd�����ֵ
	double A;				//����ϵ��A = Kp + Ki + Kd
	double B;				//����ϵ��B = -2 * Kp - Kd
	double C;				//����ϵ��C = Kd
private:
	TYPE mf_type_e;				//e�������Ⱥ�������
	TYPE mf_type_de;			//de�������Ⱥ�������
	TYPE mf_type_Kp;			//kp�������Ⱥ�������
	TYPE mf_type_Ki;			//ki�������Ⱥ�������
	TYPE mf_type_Kd;			//kd�������Ⱥ�������
	int Kp_rule_matrix[N][N];   //Kpģ���������
	int Ki_rule_matrix[N][N];   //Kiģ���������
	int Kd_rule_matrix[N][N];   //Kdģ���������
	double emax;				//������ƫ������ֵ
	double demax;				//������ƫ��΢�ֵ����ֵ
	void showMf(TYPE type, double* mf_paras);  //�ڲ����ýӿ�
};
enum VAGUE_GROUP
{
	NB = -3,
	NM,
	NS,
	ZO,
	PS,
	PM,
	PB
};
enum TYPE_GROUP_FOR_KPKIKD
{
	TYPE_ERR = 0, //E
	TYPE_DE,	  //DE
	TYPE_KP,	  //KP
	TYPE_KI,	  //KI
	TYPE_KD		  //KD
};
