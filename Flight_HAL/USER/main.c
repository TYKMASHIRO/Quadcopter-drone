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
//�ر��������������˿տ��ĵش��������ڽ��з��С������ǳ�������������ɽ����ر�ң�ء�


int main(void)
{	
	cycleCounterInit();  //�õ�ϵͳÿ��us��ϵͳCLK������Ϊ�Ժ���ʱ�������͵õ���׼�ĵ�ǰִ��ʱ��ʹ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //4��bit����ռ���ȼ���4��bit�������ȼ�
	SysTick_Config(SystemCoreClock / 1000);	//ϵͳ�δ�ʱ��

	ALL_Init();//ϵͳ��ʼ�� 

	while(1)
	{
		  main_loop();  //��������
	}
}









