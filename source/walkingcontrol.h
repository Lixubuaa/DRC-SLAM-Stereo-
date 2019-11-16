#ifndef WK_CONTROL_H_
#define WK_CONTROL_H_

#include<iostream>
#include<windows.h>                //需包含的头文件  
//#include <MMSystem.h>  
//#include <mmsystem.h> 
//#include <dsound.h>
#pragma comment(lib,"winmm.lib")   //PlaySound()函数和其他WIndows wave I/O API 函数的使用  

using namespace std;

class wk_control
{
private:
	static DWORD WINAPI write(LPVOID lpParamter);

public:
	double voltage;//加上低通环节的输出（最终结果）
	double scope;//voltage最大范围估计值；
	void adaptivePID(double err);
	void createthread();
};

#endif