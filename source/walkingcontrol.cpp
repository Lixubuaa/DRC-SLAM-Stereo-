#include "walkingcontrol.h"
#include "path_planning.h"

#define PI 3.14159265
#define T 0.4//采样周期,暂定为0.4s
//#define f_f 20
//#define T_f (1/(2*PI*f_f))
//#define T_lpf T//低通滤波器的参数
#define Wi (0.1*2*PI)//0.1HZ*2*PI
#define Wd (20*2*PI)
#define W_lpf (20*2*PI)

#define Kp 300
#define Ki 0
#define Kd 0//(Kp/(2*PI*2))  //(Kp/(2*PI*2Hz))此处赋值  
//double Kp = 80;
//double Ki = 800;
//double Kd = 80;   //0,0.01  师姐参数  

double Up = 0;
double Ui = 0;
double Ud = 0;
double Upid = 0;//pid环节的输出
double Ui_last = 0;
double Ud_last = 0;
double voltage_last = 0;
double err_last = 0;       //定义上一个偏差值
 
double tec;  //变化率ec
double tmid; //中间变量       //定义调整因子
int k;
unsigned char signal;
double rate;//=1000                    //刺激率  
int sampleRate = 44100;                   //采样率  
const int NUMPTS = (int)(44100 * 0.6);    // 0.8s：音频一声的延时   
short int waveOut[NUMPTS];                //音频数据 

ofstream control;

HWAVEOUT     hWaveOut;
WAVEHDR      WaveOutHdr;
MMRESULT     result;

//DWORD WINAPI wk_control::write(LPVOID lpParamter)
//{	/*向给定音频媒体中输出波形数据块*/
//	while (1)
//	{
//		waveOutWrite(hWaveOut, &WaveOutHdr, sizeof(WAVEHDR));
//	}
//}

void wk_control::adaptivePID(double err)
{
	//voltage = Kp*err + Ki*err + Kd*err;//0317改前
	Up = Kp*err;
	Ui = Ki*T*err / (Wi*T + 1) + Ui_last / (Wi*T + 1);
	Ud = Kd*Wd*(err - err_last) / (Wd*T + 1) + Ud_last / (Wd*T + 1);
	Upid = Up + Ui + Ud;
	//voltage = W_lpf*T*Upid / (W_lpf*T + 1) + voltage_last / (W_lpf*T + 1);//控制量
	voltage = Up + Ud;
		
	if (abs(voltage)<1e-1){
		voltage = 0;
	}
	cout << "voltage=" << voltage << endl;    //向左角度为+，向右角度为-
	control.open(data + "/control_err_voltage.txt", ios::app);
	control << err << "," << voltage << endl;
	control.close();

	//voltage——只有P控制：最大范围大约560；
	scope = 560;

	err_last = err;//存储上帧值
	Ui_last = Ui;
	Ud_last = Ud;

	//double voltagetemp = 1;
	//if (abs(voltage - voltage_last) < 10){//有圆时，避免过度转弯
	//	voltagetemp = 0;
	//}
	//voltage_last = voltage;
	//if (voltagetemp == 0){
	//	voltage = 0;
	//}
	
	rate = abs(voltage);
	cout << "rate=" << rate << endl;

	/*确定音频内容*/
	if (voltage>(1e-15))  //左>0
	{
		for (int i = 0; i<NUMPTS; i = i + 2)
		{
			waveOut[i] = (short int)ceil(sin(2 * 3.1415926*500*i / sampleRate) * (100+rate*5));//大0左，小0右
			waveOut[i + 1] = (short int)0;
		}
	}
	else if (abs(voltage) < 1e-15)
	{
		for (int i = 0; i<NUMPTS; i = i + 2)
		{
			waveOut[i] = (short int)0;//大0左，小0右
			waveOut[i + 1] = (short int)0;
		}
	}
	else  //右
	{
		for (int i = 0; i<NUMPTS; i = i + 2)
		{
			waveOut[i] = (short int)0;//大0左，小0右
			waveOut[i + 1] = (short int)ceil(sin(2 * 3.1415926*500*i / sampleRate) * (100+rate*5));
		}
	}

	/*确定音频格式*/
	WAVEFORMATEX pFormat;
	pFormat.wFormatTag = WAVE_FORMAT_PCM;
	//simple,uncompressed format  
	pFormat.nChannels = 2;//1=mono, 2=stereo  
	pFormat.wBitsPerSample = 16;
	pFormat.nSamplesPerSec = sampleRate; // 44100  
	// = nSamplesPerSec * n.Channels * wBitsPerSample/8  
	pFormat.nBlockAlign = (pFormat.nChannels)*(pFormat.wBitsPerSample) / 8;
	// = n.Channels * wBitsPerSample/8
	pFormat.nAvgBytesPerSec = sampleRate*(pFormat.nBlockAlign);
	//16 for high quality, 8 for telephone-grade  
	pFormat.cbSize = 0;
	//pFormat.SetPan=-100;

	/*打开一个给定的波形音频输出装置*/
	result = waveOutOpen(&hWaveOut, WAVE_MAPPER, &pFormat, 0L, 0L, WAVE_FORMAT_DIRECT);
	if (result)
	{
		// MessageBox(_T("Failed to open waveform output device."));  
		cout << "Failed to open waveform output device.";
		//return;  
	}

	/*准备波形数据块*/
	WaveOutHdr.lpData = (LPSTR)waveOut;
	WaveOutHdr.dwBufferLength = NUMPTS * 2; //NUMPTS * 2;
	WaveOutHdr.dwBytesRecorded = 0;
	WaveOutHdr.dwUser = 0L;
	WaveOutHdr.dwFlags = 0L;
	WaveOutHdr.dwLoops = 0L;
	waveOutPrepareHeader(hWaveOut, &WaveOutHdr, sizeof(WAVEHDR));//为播放准备一个波形缓冲区

	//createthread();
	waveOutWrite(hWaveOut, &WaveOutHdr, sizeof(WAVEHDR));
	waveOutUnprepareHeader(hWaveOut, &WaveOutHdr, sizeof(WAVEHDR));
	waveOutClose(hWaveOut);
}

//void wk_control::createthread()
//{
//	HANDLE hThread = CreateThread(NULL, 0, write, NULL, 0, NULL);
//	CloseHandle(hThread);
//	//清除数据    清除由waveOutPrepareHeader函数实现的准备
//}


