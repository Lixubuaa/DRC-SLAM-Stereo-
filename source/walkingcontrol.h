#ifndef WK_CONTROL_H_
#define WK_CONTROL_H_

#include<iostream>
#include<windows.h>                //�������ͷ�ļ�  
//#include <MMSystem.h>  
//#include <mmsystem.h> 
//#include <dsound.h>
#pragma comment(lib,"winmm.lib")   //PlaySound()����������WIndows wave I/O API ������ʹ��  

using namespace std;

class wk_control
{
private:
	static DWORD WINAPI write(LPVOID lpParamter);

public:
	double voltage;//���ϵ�ͨ���ڵ���������ս����
	double scope;//voltage���Χ����ֵ��
	void adaptivePID(double err);
	void createthread();
};

#endif