#include "VAGUE_PID.hpp"
#include <iostream>
#include "mymath.hpp"
using namespace std;
int deltaKpMatrix[7][7] = { {PB,PB,PM,PM,PS,ZO,ZO},
						   {PB,PB,PM,PS,PS,ZO,NS},
						   {PM,PM,PM,PS,ZO,NS,NS},
						   {PM,PM,PS,ZO,NS,NM,NM},
						   {PS,PS,ZO,NS,NS,NM,NM},
						   {PS,ZO,NS,NM,NM,NM,NB},
						   {ZO,ZO,NM,NM,NM,NB,NB} };
int deltaKiMatrix[7][7] = { {NB,NB,NM,NM,NS,ZO,ZO},
						 {NB,NB,NM,NS,NS,ZO,ZO},
						 {NB,NM,NS,NS,ZO,PS,PS},
						 {NM,NM,NS,ZO,PS,PM,PM},
						 {NM,NS,ZO,PS,PS,PM,PB},
						 {ZO,ZO,PS,PS,PM,PB,PB},
						 {ZO,ZO,PS,PM,PM,PB,PB} };
int deltaKdMatrix[7][7] = { {PS,NS,NB,NB,NB,NM,PS},
						 {PS,NS,NB,NM,NM,NS,ZO},
						 {ZO,NS,NM,NM,NS,NS,ZO},
						 {ZO,NS,NS,NS,NS,NS,ZO},
						 {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
						 {PB,NS,PS,PS,PS,PS,PB},
						 {PB,PM,PM,PM,PS,PS,PB} };
VAGUE_PID::VAGUE_PID(PID& pid_source,double emax_t,double de_max_t,double delta_Kp_max_t,double delta_Ki_max_t,double delta_Kd_max_t, double KP_MAX_T, double KI_MAX_T, double KD_MAX_T)  //调试好的PID参数
{
	int i = 0, j = 0;
	KpMAX = KP_MAX_T;
	KdMAX = KD_MAX_T;
	KiMAX = KI_MAX_T;
	Kp = pid_source.get_p();
	Kd = pid_source.get_d();
	Ki = pid_source.get_i();
	A = Kp+Ki+Kd;
	B = -2 * Kd - Kp;
	C = Kd;
	emax = emax_t;
	demax = de_max_t;
	Ke = (N / 2) / emax;
	Kde = (N / 2) / demax;
	delta_Kp_MAX = delta_Kp_max_t;
	delta_Ki_MAX = delta_Ki_max_t;
	delta_Kd_MAX = delta_Kd_max_t;
	mf_type_e = TRIMF;
	mf_type_de = TRIMF;
	mf_type_Kd = TRIMF;
	mf_type_Ki = TRIMF;
	mf_type_Kp = TRIMF;
	Ku_p = delta_Kp_MAX / (N / 2);
	Ku_i = delta_Ki_MAX / (N / 2);
	Ku_d = delta_Kd_MAX / (N / 2);
	derr = 0;
	err = 0;
	lasterr = 0;
	goalval = 0;
	realval = 0;
	preerr = 0;
	output = 0;
	result = 0;
	e_mf_paras = NULL;
	de_mf_paras = NULL;
	Kp_mf_paras = NULL;
	Kd_mf_paras = NULL;
	Ki_mf_paras = NULL;
	for (i = 0; i < N; i++)
	{
		for (j = 0; j < N; j++)
		{
			Kp_rule_matrix[i][j] = 0;
			Ki_rule_matrix[i][j] = 0;
			Kd_rule_matrix[i][j] = 0;
		}
	}
	for (i = 0; i < 2; i++)
	{
		qValue[i] = 0;
	}
}
VAGUE_PID::~VAGUE_PID(void)
{
	delete e_mf_paras;
	delete de_mf_paras;
	delete Kp_mf_paras;
	delete Ki_mf_paras;
	delete Kd_mf_paras;
}
void VAGUE_PID::SETRuleMatrix(int kp_m[N][N], int ki_m[N][N], int kd_m[N][N])
{
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
		{
			Kp_rule_matrix[i][j] = kp_m[i][j];
			Ki_rule_matrix[i][j] = ki_m[i][j];
			Kd_rule_matrix[i][j] = kd_m[i][j];
		}
}
//设置模糊隶属度函数的子函数
void VAGUE_PID::setMf_sub(TYPE type, double* paras, TYPE_MODE n)
{
	int N_mf_e, N_mf_de, N_mf_Kp, N_mf_Ki, N_mf_Kd;
	switch (n)
	{
	case TYPE_ERR:
		if (type == TRIMF || type == GAUSSMF || type == TRAPMF)
			mf_type_e = type;
		else
			cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
		if (mf_type_e == TRIMF)
			N_mf_e = 3;
		else if (mf_type_e == GAUSSMF)
			N_mf_e = 2;
		else if (mf_type_e == TRAPMF)
			N_mf_e = 4;
		if (e_mf_paras != NULL)
		{
			delete e_mf_paras;
			e_mf_paras = NULL;
		}
		e_mf_paras = new double[N * N_mf_e];
		for (int i = 0; i < N * N_mf_e; i++)
			e_mf_paras[i] = paras[i];
		break;

	case TYPE_DE:
		if (type == TRIMF || type == GAUSSMF || type == TRAPMF)
			mf_type_de = type;
		else
			cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
		if (mf_type_de == TRIMF)
			N_mf_de = 3;
		else if (mf_type_de == GAUSSMF)
			N_mf_de = 2;
		else if (mf_type_de == TRAPMF)
			N_mf_de = 4;
		if (de_mf_paras != NULL)
		{
			delete de_mf_paras;
			de_mf_paras = NULL;
		}
		de_mf_paras = new double[N * N_mf_de];
		for (int i = 0; i < N * N_mf_de; i++)
			de_mf_paras[i] = paras[i];
		break;

	case TYPE_KP:
		if (type == TRIMF || type == GAUSSMF || type == TRAPMF)
			mf_type_Kp = type;
		else
			cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
		if (mf_type_Kp == TRIMF)
			N_mf_Kp = 3;
		else if (mf_type_Kp == GAUSSMF)
			N_mf_Kp = 2;
		else if (mf_type_Kp == TRAPMF)
			N_mf_Kp = 4;
		if (Kp_mf_paras != NULL)
		{
			delete Kp_mf_paras;
			Kp_mf_paras = NULL;
		}
		Kp_mf_paras = new double[N * N_mf_Kp];
		for (int i = 0; i < N * N_mf_Kp; i++)
			Kp_mf_paras[i] = paras[i];
		break;

	case TYPE_KI:
		if (type == TRIMF || type == GAUSSMF || type == TRAPMF)
			mf_type_Ki = type;
		else
			cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
		if (mf_type_Ki == TRIMF)
			N_mf_Ki = 3;
		else if (mf_type_Ki == GAUSSMF)
			N_mf_Ki = 2;
		else if (mf_type_Ki == TRAPMF)
			N_mf_Ki = 4;
		if (Ki_mf_paras != NULL)
			delete Ki_mf_paras;
		Ki_mf_paras = new double[N * N_mf_Ki];
		for (int i = 0; i < N * N_mf_Ki; i++)
			Ki_mf_paras[i] = paras[i];
		break;

	case TYPE_KD:
		if (type == TRIMF || type == GAUSSMF || type == TRAPMF)
			mf_type_Kd = type;
		else
			cout << "Type of membership function must be \"trimf\" or \"gaussmf\" or \"trapmf\"" << endl;
		if (mf_type_Kd == TRIMF)
			N_mf_Kd = 3;
		else if (mf_type_Kd == GAUSSMF)
			N_mf_Kd = 2;
		else if (mf_type_Kd == TRAPMF)
			N_mf_Kd = 4;
		if (Kd_mf_paras != NULL)
			delete Kd_mf_paras;
		Kd_mf_paras = new double[N * N_mf_Kd];
		for (int i = 0; i < N * N_mf_Kd; i++)
			Kd_mf_paras[i] = paras[i];
		break;

	default: break;
	}
}
double VAGUE_PID::Get_VAGUE_PID_OUTPUT(void)
{
	double u_e[N], u_de[N],u_u[N];
	int u_e_index[3], u_de_index[3];//假设一个输入最多激活3个模糊子集
	double delta_Kp, delta_Ki, delta_Kd;
	err = goalval - realval;
	derr = err - lasterr;
	err = Ke * err;
	derr = Kde * derr;
	int j = 0;
	for (int i = 0; i < N; i++)
	{
		if (mf_type_e == TRIMF)
			u_e[i] = trimf(err, e_mf_paras[i * 3], e_mf_paras[i * 3 + 1], e_mf_paras[i * 3 + 2]);//e模糊化，计算它的隶属度
		else if (mf_type_e == GAUSSMF)
			u_e[i] = gaussmf(err, e_mf_paras[i * 2], e_mf_paras[i * 2 + 1]);//e模糊化，计算它的隶属度
		else if (mf_type_e == TRAPMF)
			u_e[i] = trapmf(err, e_mf_paras[i * 4], e_mf_paras[i * 4 + 1], e_mf_paras[i * 4 + 2], e_mf_paras[i * 4 + 3]);//e模糊化，计算它的隶属度
		if (u_e[i] != 0)
			u_e_index[j++] = i;                //存储被激活的模糊子集的下标，可以减小计算量
	}
	for (; j < 3; j++)u_e_index[j] = 0;             //富余的空间填0

	/*将误差变化率de模糊化*/
	j = 0;
	for (int i = 0; i < N; i++)
	{
		if (mf_type_de == TRIMF)
			u_de[i] = trimf(derr, de_mf_paras[i * 3], de_mf_paras[i * 3 + 1], de_mf_paras[i * 3 + 2]);//de模糊化，计算它的隶属度
		else if (mf_type_de == GAUSSMF)
			u_de[i] = gaussmf(derr, de_mf_paras[i * 2], de_mf_paras[i * 2 + 1]);//de模糊化，计算它的隶属度
		else if (mf_type_de == TRAPMF)
			u_de[i] = trapmf(derr, de_mf_paras[i * 4], de_mf_paras[i * 4 + 1], de_mf_paras[i * 4 + 2], de_mf_paras[i * 4 + 3]);//de模糊化，计算它的隶属度

		if (u_de[i] != 0)
			u_de_index[j++] = i;            //存储被激活的模糊子集的下标，可以减小计算量
	}
	for (; j < 3; j++)u_de_index[j] = 0;          //富余的空间填0
	double den = 0, num = 0;
	 /*计算delta_Kp和Kp*/
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * Kp_rule_matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}
	delta_Kp = num / den;
	delta_Kp = Ku_p * delta_Kp;
	if (delta_Kp >= delta_Kp_MAX)   delta_Kp = delta_Kp_MAX;
	else if (delta_Kp <= -delta_Kp_MAX) delta_Kp = -delta_Kp_MAX;
	Kp += delta_Kp;
	if (Kp < 0)Kp = 0;
	else if (Kp > KpMAX)
		Kp = KpMAX;
	/*计算delta_Ki和Ki*/
	den = 0; num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * Ki_rule_matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}

	delta_Ki = num / den;
	delta_Ki = Ku_i * delta_Ki;
	if (delta_Ki >= delta_Ki_MAX)   delta_Ki = delta_Ki_MAX;
	else if (delta_Ki <= -delta_Ki_MAX)  delta_Ki = -delta_Ki_MAX;
	Ki += delta_Ki;
	if (Ki < 0)Ki = 0;
	else if (Ki > KiMAX)
		Ki = KiMAX;
	/*计算delta_Kd和Kd*/
	den = 0; num = 0;
	for (int m = 0; m < 3; m++)
		for (int n = 0; n < 3; n++)
		{
			num += u_e[u_e_index[m]] * u_de[u_de_index[n]] * Kd_rule_matrix[u_e_index[m]][u_de_index[n]];
			den += u_e[u_e_index[m]] * u_de[u_de_index[n]];
		}
	delta_Kd = num / den;
	delta_Kd = Ku_d * delta_Kd;
	if (delta_Kd >= delta_Kd_MAX)   delta_Kd = delta_Kd_MAX;
	else if (delta_Kd <= -delta_Kd_MAX) delta_Kd = -delta_Kd_MAX;
	Kd += delta_Kd;
	if (Kd < 0)Kd = 0;
	else if (Kd > KdMAX)
		Kd = KdMAX;
	A = Kp + Ki + Kd;
	B = -2 * Kd - Kp;
	C = Kd;
	output = A * err + B * lasterr + C * preerr;
	result = output / Ke;
	//if (result >= 0.95 * goalval)
	//	result = 0.95 * goalval;
	//else if (result <= -0.95 * (goalval))
	//	result = -0.95 * goalval;
	preerr = lasterr;
	lasterr = err;

	return result;
}
void VAGUE_PID::showMf(TYPE type_t, double* mf_paras)
{
	int tab;
	string type;
	if (type_t == TRIMF)
	{
		type = "trimf";
		tab = 2;
	}
	else if (type_t == GAUSSMF)
	{
		type = "gaussmf";
		tab = 1;
	}
	else if (type_t == TRAPMF)
	{
		type = "trapmf";
		tab = 3;
	}
	cout << "函数类型：" << type << endl;
	cout << "函数参数列表：" << endl;
	double* p = mf_paras;
	for (int i = 0; i < N * (tab + 1); i++)
	{
		cout.width(3);
		cout << p[i] << "  ";
		if (i % (tab + 1) == tab)
			cout << endl;
	}
}
void VAGUE_PID::showInfo(void)
{
	cout << "Info of this fuzzy controller is as following:" << endl;
	cout << "基本论域e：[" << -emax << "," << emax << "]" << endl;
	cout << "基本论域de：[" << -demax << "," << demax << "]" << endl;
	cout << "基本论域delta_Kp：[" << -delta_Kp_MAX << "," << delta_Kp_MAX << "]" << endl;
	cout << "基本论域delta_Ki：[" << -delta_Ki_MAX << "," << delta_Ki_MAX << "]" << endl;
	cout << "基本论域delta_Kd：[" << -delta_Kd_MAX << "," << delta_Kd_MAX << "]" << endl;
	cout << "误差e的模糊隶属度函数参数：" << endl;
	showMf(mf_type_e, e_mf_paras);
	cout << "误差变化率de的模糊隶属度函数参数：" << endl;
	showMf(mf_type_de, de_mf_paras);
	cout << "delta_Kp的模糊隶属度函数参数：" << endl;
	showMf(mf_type_Kp, Kp_mf_paras);
	cout << "delta_Ki的模糊隶属度函数参数：" << endl;
	showMf(mf_type_Ki, Ki_mf_paras);
	cout << "delta_Kd的模糊隶属度函数参数：" << endl;
	showMf(mf_type_Kd, Kd_mf_paras);
	cout << "模糊规则表：" << endl;
	cout << "delta_Kp的模糊规则矩阵" << endl;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cout.width(3);
			cout << Kp_rule_matrix[i][j] << "  ";
		}
		cout << endl;
	}
	cout << "delta_Ki的模糊规则矩阵" << endl;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cout.width(3);
			cout << Ki_rule_matrix[i][j] << "  ";
		}
		cout << endl;
	}
	cout << "delta_Kd的模糊规则矩阵" << endl;
	for (int i = 0; i < N; i++)
	{
		for (int j = 0; j < N; j++)
		{
			cout.width(3);
			cout << Kd_rule_matrix[i][j] << "  ";
		}
		cout << endl;
	}
	cout << endl;
	cout << "误差的量化比例因子Ke=" << Ke << endl;
	cout << "误差变化率的量化比例因子Kde=" << Kde << endl;
	cout << "输出的量化比例因子Ku_p=" << Ku_p << endl;
	cout << "输出的量化比例因子Ku_i=" << Ku_i << endl;
	cout << "输出的量化比例因子Ku_d=" << Ku_d << endl;
	cout << "设定目标target=" << goalval << endl;
	cout << "误差e=" << err << endl;
	cout << "Kp=" << Kp << endl;
	cout << "Ki=" << Ki << endl;
	cout << "Kd=" << Kd << endl;
	cout << endl;
}
