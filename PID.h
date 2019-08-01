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
	double get_p(void);								 //��ȡĿǰ��P
	double get_i(void);								 //��ȡĿǰ��I
	double get_d(void);								 //��ȡĿǰ��D
	void set_p(double p_t);							 //�趨P
	void set_i(double i_t);							 //�趨I
	void set_d(double d_t);							 //�趨D
	void set_realval(double new_real_val);			 //ʵ��ֵ���£��������л�ȡ��
	void set_goalval(double new_goal_val);			 //�趨Ŀ��ֵ
	virtual double get_PID_control_output(void);			 //��ȡPID������
	void SET_Intergral_Limter(bool FLAG);			 //�����޷������ƽӿ�
	void SET_Intergral_Separater(bool FLAG);		 //���ַ��������ƽӿ�
	void SET_Limit_val(double limit_val_t);			 //�趨�����޷���ֵ
	void SET_ERR_Limit_val(double err_limit_val_t);  //�趨���ַ�����ֵ
protected:
	double p;										 //P
	double i;										 //I
	double d;										 //D
	double goal_val;								 //Ŀ��ֵ
	double real_val;								 //ʵ��ֵ
	double err;										 //���ֵ
	double err_last;								 //�ϴε����ֵ
	double integral;								 //������
	bool Intergral_Limit_Flag;						 //�����޷���������־λ
	double limit_val;								 //�����޷�ֵ
	bool Intergral_separate_Flag;					 //���ַ�����������־λ
	double err_limit_val;							 //�������ֵ
	double alpha;									 //����ȫ΢��ϵ��
	double Intergral_separate_Actuator;				 //��Ҫ��������
};
class PID_D_Incomplete :public PID
{
public:
	PID_D_Incomplete(double p_t, double i_t, double d_t);
	PID_D_Incomplete(PID_D_Incomplete& pid);
	~PID_D_Incomplete(void);
};