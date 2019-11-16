#include "walkingcontrol.h"
#include "path_planning.h"

#define PI 3.14159265
#define T 0.4//��������,�ݶ�Ϊ0.4s
//#define f_f 20
//#define T_f (1/(2*PI*f_f))
//#define T_lpf T//��ͨ�˲����Ĳ���
#define Wi (0.1*2*PI)//0.1HZ*2*PI
#define Wd (20*2*PI)
#define W_lpf (20*2*PI)

#define Kp 300
#define Ki 0
#define Kd 0//(Kp/(2*PI*2))  //(Kp/(2*PI*2Hz))�˴���ֵ  
//double Kp = 80;
//double Ki = 800;
//double Kd = 80;   //0,0.01  ʦ�����  

double Up = 0;
double Ui = 0;
double Ud = 0;
double Upid = 0;//pid���ڵ����
double Ui_last = 0;
double Ud_last = 0;
double voltage_last = 0;
double err_last = 0;       //������һ��ƫ��ֵ
 
double tec;  //�仯��ec
double tmid; //�м����       //�����������
int k;
unsigned char signal;
double rate;//=1000                    //�̼���  
int sampleRate = 44100;                   //������  
const int NUMPTS = (int)(44100 * 0.6);    // 0.8s����Ƶһ������ʱ   
short int waveOut[NUMPTS];                //��Ƶ���� 

ofstream control;

HWAVEOUT     hWaveOut;
WAVEHDR      WaveOutHdr;
MMRESULT     result;

//DWORD WINAPI wk_control::write(LPVOID lpParamter)
//{	/*�������Ƶý��������������ݿ�*/
//	while (1)
//	{
//		waveOutWrite(hWaveOut, &WaveOutHdr, sizeof(WAVEHDR));
//	}
//}

void wk_control::adaptivePID(double err)
{
	//voltage = Kp*err + Ki*err + Kd*err;//0317��ǰ
	Up = Kp*err;
	Ui = Ki*T*err / (Wi*T + 1) + Ui_last / (Wi*T + 1);
	Ud = Kd*Wd*(err - err_last) / (Wd*T + 1) + Ud_last / (Wd*T + 1);
	Upid = Up + Ui + Ud;
	//voltage = W_lpf*T*Upid / (W_lpf*T + 1) + voltage_last / (W_lpf*T + 1);//������
	voltage = Up + Ud;
		
	if (abs(voltage)<1e-1){
		voltage = 0;
	}
	cout << "voltage=" << voltage << endl;    //����Ƕ�Ϊ+�����ҽǶ�Ϊ-
	control.open(data + "/control_err_voltage.txt", ios::app);
	control << err << "," << voltage << endl;
	control.close();

	//voltage����ֻ��P���ƣ����Χ��Լ560��
	scope = 560;

	err_last = err;//�洢��ֵ֡
	Ui_last = Ui;
	Ud_last = Ud;

	//double voltagetemp = 1;
	//if (abs(voltage - voltage_last) < 10){//��Բʱ���������ת��
	//	voltagetemp = 0;
	//}
	//voltage_last = voltage;
	//if (voltagetemp == 0){
	//	voltage = 0;
	//}
	
	rate = abs(voltage);
	cout << "rate=" << rate << endl;

	/*ȷ����Ƶ����*/
	if (voltage>(1e-15))  //��>0
	{
		for (int i = 0; i<NUMPTS; i = i + 2)
		{
			waveOut[i] = (short int)ceil(sin(2 * 3.1415926*500*i / sampleRate) * (100+rate*5));//��0��С0��
			waveOut[i + 1] = (short int)0;
		}
	}
	else if (abs(voltage) < 1e-15)
	{
		for (int i = 0; i<NUMPTS; i = i + 2)
		{
			waveOut[i] = (short int)0;//��0��С0��
			waveOut[i + 1] = (short int)0;
		}
	}
	else  //��
	{
		for (int i = 0; i<NUMPTS; i = i + 2)
		{
			waveOut[i] = (short int)0;//��0��С0��
			waveOut[i + 1] = (short int)ceil(sin(2 * 3.1415926*500*i / sampleRate) * (100+rate*5));
		}
	}

	/*ȷ����Ƶ��ʽ*/
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

	/*��һ�������Ĳ�����Ƶ���װ��*/
	result = waveOutOpen(&hWaveOut, WAVE_MAPPER, &pFormat, 0L, 0L, WAVE_FORMAT_DIRECT);
	if (result)
	{
		// MessageBox(_T("Failed to open waveform output device."));  
		cout << "Failed to open waveform output device.";
		//return;  
	}

	/*׼���������ݿ�*/
	WaveOutHdr.lpData = (LPSTR)waveOut;
	WaveOutHdr.dwBufferLength = NUMPTS * 2; //NUMPTS * 2;
	WaveOutHdr.dwBytesRecorded = 0;
	WaveOutHdr.dwUser = 0L;
	WaveOutHdr.dwFlags = 0L;
	WaveOutHdr.dwLoops = 0L;
	waveOutPrepareHeader(hWaveOut, &WaveOutHdr, sizeof(WAVEHDR));//Ϊ����׼��һ�����λ�����

	//createthread();
	waveOutWrite(hWaveOut, &WaveOutHdr, sizeof(WAVEHDR));
	waveOutUnprepareHeader(hWaveOut, &WaveOutHdr, sizeof(WAVEHDR));
	waveOutClose(hWaveOut);
}

//void wk_control::createthread()
//{
//	HANDLE hThread = CreateThread(NULL, 0, write, NULL, 0, NULL);
//	CloseHandle(hThread);
//	//�������    �����waveOutPrepareHeader����ʵ�ֵ�׼��
//}


