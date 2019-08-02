#include <iostream>
#include <iomanip>
#include "PID.hpp"
using namespace std;
int main()
{
	double output = 0;
	int count = 0;
	PID pid_ted(0.2,0.015,0.2);
	pid_ted.SET_Alpha(0.9);
	pid_ted.SET_D_Incompleter(DISABLE);
	for (count = 0; count < 1000; count++)
	{
		pid_ted.set_goalval(200.0);
		output = pid_ted.get_PID_control_output();
		pid_ted.set_realval(output + pid_ted.get_real_val());
		cout << "output: " <<  setiosflags(ios::fixed) << setprecision(3) << output << endl;
	}
}
