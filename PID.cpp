#include "PID.hpp"
PID::PID(double p_t, double i_t, double d_t) :p(p_t), i(i_t), d(d_t)
{
	err = 0;
	err_last = 0;
	real_val = 0;
	goal_val = 0;
	integral = 0;
	Intergral_Limit_Flag = false;
	Intergral_separate_Flag = false;
	Intergral_separate_Actuator = 1;  //默认不启动积分分离器
	limit_val = 0;
	err_limit_val = 0;
	alpha = 0;
	lastdev = 0;
	D_Incomplete_Flag = false;
}
PID::PID(PID& pid)  //拷贝构造函数
{
	p = pid.p;
	i = pid.i;
	d = pid.d;
	err = pid.err;
	err_last = pid.err_last;
	real_val = pid.real_val;
	goal_val = pid.goal_val;
	integral = pid.integral;
	Intergral_Limit_Flag = pid.Intergral_Limit_Flag;
	Intergral_separate_Flag = pid.Intergral_separate_Flag;
	Intergral_separate_Actuator = pid.Intergral_separate_Actuator;
	limit_val = pid.limit_val;
	err_limit_val = pid.err_limit_val;
	alpha = pid.alpha;
	lastdev = pid.lastdev;
	D_Incomplete_Flag = pid.D_Incomplete_Flag;
}
double PID::get_p(void)
{
	return p;
}
double PID::get_i(void)
{
	return i;
}
double PID::get_d(void)
{
	return d;
}
double PID::get_goal_val(void)
{
	return goal_val;
}
double PID::get_real_val(void)
{
	return real_val;
}
void PID::set_p(double p_t)
{
	p = p_t;
}
void PID::set_i(double i_t)
{
	i = i_t;
}
void PID::set_d(double d_t)
{
	d = d_t;
}
void PID::set_goalval(double new_goal_val)
{
	goal_val = new_goal_val;
}
void PID::set_realval(double new_real_val)
{
	real_val = new_real_val;
}
void PID::SET_Intergral_Limter(bool FLAG)
{
	Intergral_Limit_Flag = FLAG;
}
void PID::SET_Intergral_Separater(bool FLAG)
{
	Intergral_separate_Flag = FLAG;
}
void PID::SET_Limit_val(double limit_val_t)
{
	limit_val = limit_val_t;
}
void PID::SET_ERR_Limit_val(double err_limit_val_t)
{
	err_limit_val = err_limit_val_t;
}
PID::~PID(void)
{

}

double PID::get_PID_control_output(void)  //PID控制器主计算函数
{
	double dev = 0;
	double output = 0;
	err = goal_val - real_val;
	if (D_Incomplete_Flag)
		dev = (1 - alpha) * (err - err_last) + alpha * lastdev;
	else
		dev = err - err_last;
	if (Intergral_separate_Flag)
	{
		if (err > err_limit_val)
			Intergral_separate_Actuator = 0;
		else
			Intergral_separate_Actuator = 1;
	}
	else
		Intergral_separate_Actuator = 1;
	output = p * err + d * dev + i * integral * Intergral_separate_Actuator;  //主计算装置
	if (Intergral_Limit_Flag)  //如果启动积分限幅器
	{
		if (fabs(integral) >= limit_val)  //当绝对值大于限制时
			integral = limit_val;  //只是用限制值
		else
			integral += err;  //否则按照正常积分方法积分
	}
	else
		integral += err;  //不启用积分限幅器的话按照正常积分方法积分
	err_last = err;
	lastdev = dev;
	return output;
}
void PID::SET_Alpha(double alpha_t)
{
	alpha = alpha_t;
}
void PID::SET_D_Incompleter(bool FLAG)
{
	D_Incomplete_Flag = FLAG;
}