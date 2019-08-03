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
	VAGUE_PID(PID &pid_t,double emax_t, double de_max_t,double delta_Kp_max_t, double delta_Ki_max_t, double delta_Kd_max_t);  //训练好的PID
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
	double output;  //控制器输出量
	double result;	//物理输出流
	double* e_mf_paras; //误差的隶属度函数的参数
	double* de_mf_paras;//误差的偏差隶属度函数的参数
	double* Kp_mf_paras; //kp的隶属度函数的参数
	double* Ki_mf_paras; //ki的隶属度函数的参数
	double* Kd_mf_paras; //kd的隶属度函数的参数
	double Ke;      //Ke=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
	double Kde;     //Ke=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
	double Ku_p;    //Ku_p=Kpmax/n,量化论域为[-3,-2,-1,0,1,2,3]
	double Ku_i;    //Ku_i=Kimax/n,量化论域为[-3,-2,-1,0,1,2,3]
	double Ku_d;    //Ku_d=Kdmax/n,量化论域为[-3,-2,-1,0,1,2,3]
	double delta_Kp_MAX;
	double delta_Ki_MAX;
	double delta_Kd_MAX;
	double A;
	double B;
	double C;
private:
	double qValue[2];
	TYPE mf_type_e;       //e的隶属度函数类型
	TYPE mf_type_de;      //de的隶属度函数类型
	TYPE mf_type_Kp;      //kp的隶属度函数类型
	TYPE mf_type_Ki;      //ki的隶属度函数类型
	TYPE mf_type_Kd;      //kd的隶属度函数类型
	int Kp_rule_matrix[N][N];   //Kp模糊规则矩阵
	int Ki_rule_matrix[N][N];   //Ki模糊规则矩阵
	int Kd_rule_matrix[N][N];   //Kd模糊规则矩阵
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
