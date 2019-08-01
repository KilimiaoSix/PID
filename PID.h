#pragma once
#include <math.h>
#define ENABLE true
#define DISABLE false
class PID
{
public:
	PID(double p_t, double i_t, double d_t);
	PID(PID& pid);
	~PID();
	double get_p(void);								 //获取目前的P
	double get_i(void);								 //获取目前的I
	double get_d(void);								 //获取目前的D
	void set_p(double p_t);							 //设定P
	void set_i(double i_t);							 //设定I
	void set_d(double d_t);							 //设定D
	void set_realval(double new_real_val);			 //实际值更新（传感器中获取）
	void set_goalval(double new_goal_val);			 //设定目标值
	virtual double get_PID_control_output(void);			 //获取PID运算结果
	void SET_Intergral_Limter(bool FLAG);			 //积分限幅器控制接口
	void SET_Intergral_Separater(bool FLAG);		 //积分分离器控制接口
	void SET_Limit_val(double limit_val_t);			 //设定积分限幅的值
	void SET_ERR_Limit_val(double err_limit_val_t);  //设定积分分离阈值
protected:
	double p;										 //P
	double i;										 //I
	double d;										 //D
	double goal_val;								 //目标值
	double real_val;								 //实际值
	double err;										 //误差值
	double err_last;								 //上次的误差值
	double integral;								 //积分量
	bool Intergral_Limit_Flag;						 //积分限幅器启动标志位
	double limit_val;								 //积分限幅值
	bool Intergral_separate_Flag;					 //积分分离器启动标志位
	double err_limit_val;							 //误差限制值
	double alpha;									 //不完全微分系数
	double Intergral_separate_Actuator;				 //不要动！！！
};
class PID_D_Incomplete :public PID
{
public:
	PID_D_Incomplete(double p_t, double i_t, double d_t);
	PID_D_Incomplete(PID_D_Incomplete& pid);
	~PID_D_Incomplete(void);
};