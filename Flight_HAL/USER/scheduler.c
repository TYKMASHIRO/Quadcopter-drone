//========================================================================
//	�����ߵ��ӹ�����-�Ա� https://devotee.taobao.com/
//	STM32���ᰮ����QQȺ: 799870988
//	���ߣ�С��
//	�绰:13728698082
//	����:1042763631@qq.com
//	���ڣ�2020.05.17
//	�汾��V1.0
//========================================================================
//�׼������ַ��https://devotee.taobao.com/
//�׼������ַ��https://devotee.taobao.com/
//                 �����ߵ��ӹ�����
//�ش�������
//
//         �˳���ֻ������ѧϰ��������ҵ��;����׷�����Σ�
//          
//
//
#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "ANO_Data_Transfer.h"
#include "WIFI_UFO.h"
#include "ADC.h"

loop_t loop; 
u32 time[10],time_sum;
u8 Flow_SSI_CNT,Locat_SSI_CNT,Locat_SSI,Flow_SSI,Locat_Mode;
void Loop_check()
{
	loop.cnt_2ms++;
	loop.cnt_4ms++;
	loop.cnt_6ms++;
	loop.cnt_10ms++;
	loop.cnt_20ms++;
	loop.cnt_50ms++;
	loop.cnt_1000ms++;

	if( loop.check_flag >= 1)
	{
		loop.err_flag ++;// 2ms 
	}
	else
	{
		loop.check_flag += 1;   //�ñ�־λ��ѭ��������0
	}
}
void main_loop()
{
	if( loop.check_flag >= 1 )
	{
		
		if( loop.cnt_2ms >= 1 )
		{
			loop.cnt_2ms = 0;
			
			Duty_2ms();	 					//����2ms������
		}
		if( loop.cnt_4ms >= 2 )
		{
			loop.cnt_4ms = 0;
			Duty_4ms();						//����4ms������
		}
		if( loop.cnt_6ms >= 3 )
		{
			loop.cnt_6ms = 0;
			Duty_6ms();						//����6ms������
		}
		if( loop.cnt_10ms >= 5 )
		{
			loop.cnt_10ms = 0;
			Duty_10ms();					//����10ms������
		} 
		if( loop.cnt_20ms >= 10 )
		{
			loop.cnt_20ms = 0;
			Duty_20ms();					//����20ms������
		}
		if( loop.cnt_50ms >= 25 )
		{
			loop.cnt_50ms = 0;
			Duty_50ms();					//����50ms������
		}
		if( loop.cnt_1000ms >= 500)
		{
			loop.cnt_1000ms = 0;
			Duty_1000ms();				//����1s������
		}
		loop.check_flag = 0;		//ѭ��������ϱ�־
	}
}
/////////////////////////////////////////////////////////
void Duty_2ms()
{
	time[0] = GetSysTime_us();
	
	MpuGetData();				          //��ȡ����������
	FlightPidControl(0.002f);     /// ��̬����
	MotorControl();               //�������
	
	time[0] = GetSysTime_us() - time[0];
}
//////////////////////////////////////////////////////////
void Duty_4ms()
{
	time[1] = GetSysTime_us();
	
	ANO_NRF_Check_Event();    //ɨ�����2.4G�ź�
	ANO_DT_Data_Exchange();		//���طɻ����ݵ�ң����
	Rc_Connect();  						//����ң��������
	time[1] = GetSysTime_us() - time[1];
}
//////////////////////////////////////////////////////////
void Duty_6ms()
{
	time[2] = GetSysTime_us();
	
	GetAngle(&MPU6050,&Angle,0.006f);   //������̬����
	
	time[2] = GetSysTime_us() - time[2];
}
/////////////////////////////////////////////////////////
void Duty_10ms()
{
	time[3] = GetSysTime_us();
	
	RC_Analy();	     					//ң��������ָ���
	
	time[3] = GetSysTime_us() - time[3];
}
/////////////////////////////////////////////////////////
void Duty_20ms()
{
	time[4] = GetSysTime_us();

	ANTO_polling(); 	//����3�ڷɻ� ������ݵ� ������λ��
	
	time[4] = GetSysTime_us() - time[4];
}
//////////////////////////////////////////////////////////
void Duty_50ms()
{
	time[5] = GetSysTime_us();
	
	PilotLED(); 						//LEDˢ��
	
	Flag_Check();   			 //������״̬��־
	
	time[5] = GetSysTime_us() - time[5];
}
/////////////////////////////////////////////////////////////
void Duty_1000ms()
{
	u8 i;
  NRF_SSI = NRF_SSI_CNT;  //NRF�ź�ǿ��
	NRF_SSI_CNT = 0;
	
	WIFI_SSI = WIFI_SSI_CNT;//WiFi�ź�ǿ��    Ԥ��
	WIFI_SSI_CNT = 0;
	
	Locat_SSI = Locat_SSI_CNT;//�Ӿ�λ������Ƶ��  Ԥ��
	Locat_SSI_CNT = 0;
	
	Flow_SSI = Flow_SSI_CNT;  //��������Ƶ��  Ԥ��
	Flow_SSI_CNT = 0;
	
		//����Ӿ���λģ���Ƿ����    Ԥ��
	if(Locat_SSI>10)  Locat_Err = 0;
	else Locat_Mode=0,Locat_Err = 1;
	
	  //������ģ���Ƿ����				Ԥ��
	if(Flow_SSI>10)  Flow_Err = 0;
	else 						 Flow_Err = 1;
	
	time_sum = 0;
	for(i=0;i<6;i++)	time_sum += time[i];
}



//////////////////////////end///////////////////////////////////////////