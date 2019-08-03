#include <iostream>
#include "mymath.hpp"
using namespace std;
//三角隶属度函数
double trimf(double x, double a, double b, double c)
{
	double u;
	if (x >= a && x <= b)
	{
		u = (x - a) / (b - a);
	}
	else if (x > b && x <= c)
	{
		u = (c - x) / (c - b);
	}
	else
		u = 0.0;
	return u;
}
//正态隶属度函数
double gaussmf(double x, double ave, double sigma)
{
	double u;
	if (sigma < 0)
	{
		cout << "In gaussmf, sigma must larger than 0" << endl;
	}
	u = exp(-pow(((x - ave) / sigma), 2));
	return u;
}
double trapmf(double x, double a, double b, double c, double d)
{
	double u;
	if (x >= a && x < b)
		u = (x - a) / (b - a);
	else if (x >= b && x < c)
		u = 1;
	else if (x >= c && x <= d)
		u = (d - x) / (d - c);
	else
		u = 0;
	return u;
}