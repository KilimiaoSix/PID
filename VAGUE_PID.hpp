#pragma once
#include "PID.hpp"
#include "mymath.hpp"
#define N 7
extern int deltaKpMatrix[N][N];
extern int deltaKiMatrix[N][N];
extern int deltaKdMatrix[N][N];
typedef int TYPE_MODE;
class VAGUE_PID
{
public:
	VAGUE_PID(PID &pid_t,double emax_t, double de_max_t,double delta_Kp_max_t, double delta_Ki_max_t, double delta_Kd_max_t);  //ѵ���õ�PID
	~VAGUE_PID(void);
	void SETRuleMatrix(int** kp_m, int** ki_m, int** kd_m);
	void setMf_sub(TYPE type, double* paras, TYPE_MODE n);
	double Get_VAGUE_PID_OUTPUT(void);
	
protected:
	double goalval;
	double realval;
	double Kp;
	double Ki;
	double Kd;
	double err;
	double lasterr;
	double preerr;
	double derr;
	double output;  //�����������
	double result;	//���������
	double* e_mf_paras; //���������Ⱥ����Ĳ���
	double* de_mf_paras;//����ƫ�������Ⱥ����Ĳ���
	double* Kp_mf_paras; //kp�������Ⱥ����Ĳ���
	double* Ki_mf_paras; //ki�������Ⱥ����Ĳ���
	double* Kd_mf_paras; //kd�������Ⱥ����Ĳ���
	double Ke;      //Ke=n/emax,��������Ϊ[-3,-2,-1,0,1,2,3]
	double Kde;     //Ke=n/emax,��������Ϊ[-3,-2,-1,0,1,2,3]
	double Ku_p;    //Ku_p=Kpmax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
	double Ku_i;    //Ku_i=Kimax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
	double Ku_d;    //Ku_d=Kdmax/n,��������Ϊ[-3,-2,-1,0,1,2,3]
	double delta_Kp_MAX;
	double delta_Ki_MAX;
	double delta_Kd_MAX;
	double A;
	double B;
	double C;
private:
	double qValue[2];
	TYPE mf_type_e;       //e�������Ⱥ�������
	TYPE mf_type_de;      //de�������Ⱥ�������
	TYPE mf_type_Kp;      //kp�������Ⱥ�������
	TYPE mf_type_Ki;      //ki�������Ⱥ�������
	TYPE mf_type_Kd;      //kd�������Ⱥ�������
	int Kp_rule_matrix[N][N];   //Kpģ���������
	int Ki_rule_matrix[N][N];   //Kiģ���������
	int Kd_rule_matrix[N][N];   //Kdģ���������
	double emax;
	double demax;
	void showMf(TYPE type, double* mf_paras);
	void showInfo(void);
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
	TYPE_ERR = 1,
	TYPE_DE,
	TYPE_KP,
	TYPE_KI,
	TYPE_KD
};
