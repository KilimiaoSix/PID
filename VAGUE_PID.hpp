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
	VAGUE_PID(PID &pid_t,double emax_t, double de_max_t,double delta_Kp_max_t, double delta_Ki_max_t, double delta_Kd_max_t,double KP_MAX_T,double KI_MAX_T,double KD_MAX_T);  //训练好的PID
	~VAGUE_PID(void);
	void SETRuleMatrix(int kp_m[N][N], int ki_m[N][N], int kd_m[N][N]);
	void setMf_sub(TYPE type, double* paras, TYPE_MODE n);  //设置选择的函数类型，type为函数类型，paras为参数矩阵，用main函数里面写好的现成的即可，n为选择设置的对象，参考下面
	double Get_VAGUE_PID_OUTPUT(void);
	void SET_Goal_Val(double goalval_t)  //修改目标值，运行时调用这个接口就可以了
	{
		goalval = goalval_t;
	}
	void UPDATE_REAL_DATA(double realval_t)  
	{
		realval = realval_t;
	}
	void showInfo(void);
protected:
	double goalval;			//控制目标值（想要达到的物理量）
	double realval;			//当前实际值（传感器测得的物理量）
	double Kp;				//Kp 比例系数
	double Ki;				//Ki 积分系数
	double Kd;				//Kd 微分系数
	double err;				//偏差值 控制系统将会自动计算当前偏差量
	double lasterr;			//前一次的偏差量
	double preerr;			//前两次的偏差量
	double derr;			//偏差量的微分
	double output;			//控制器输出量
	double result;			//物理输出流
	double* e_mf_paras;		//误差的隶属度函数的参数
	double* de_mf_paras;	//误差的偏差隶属度函数的参数
	double* Kp_mf_paras;	//kp的隶属度函数的参数
	double* Ki_mf_paras;	//ki的隶属度函数的参数
	double* Kd_mf_paras;	//kd的隶属度函数的参数
	double Ke;				//Ke=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
	double Kde;				//Ke=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
	double Ku_p;			//Ku_p=Kpmax/n,量化论域为[-3,-2,-1,0,1,2,3]
	double Ku_i;			//Ku_i=Kimax/n,量化论域为[-3,-2,-1,0,1,2,3]
	double Ku_d;			//Ku_d=Kdmax/n,量化论域为[-3,-2,-1,0,1,2,3]
	double delta_Kp_MAX;	//一次允许增加或者减少的最大Kp量
	double delta_Ki_MAX;	//一次允许增加或者减少的最大Ki量
	double delta_Kd_MAX;	//一次允许增加或者减少的最大Kd量
	double KpMAX;			//允许Kp的最大值
	double KiMAX;			//允许Ki的最大值
	double KdMAX;			//允许Kd的最大值
	double A;				//运算系数A = Kp + Ki + Kd
	double B;				//运算系数B = -2 * Kp - Kd
	double C;				//运算系数C = Kd
private:
	TYPE mf_type_e;				//e的隶属度函数类型
	TYPE mf_type_de;			//de的隶属度函数类型
	TYPE mf_type_Kp;			//kp的隶属度函数类型
	TYPE mf_type_Ki;			//ki的隶属度函数类型
	TYPE mf_type_Kd;			//kd的隶属度函数类型
	int Kp_rule_matrix[N][N];   //Kp模糊规则矩阵
	int Ki_rule_matrix[N][N];   //Ki模糊规则矩阵
	int Kd_rule_matrix[N][N];   //Kd模糊规则矩阵
	double emax;				//理论上偏差的最大值
	double demax;				//理论上偏差微分的最大值
	void showMf(TYPE type, double* mf_paras);  //内部调用接口
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
