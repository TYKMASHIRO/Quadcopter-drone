#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "main.h"
#include "spi.h"

#define NRF24L01_CSN_HIGH HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET)
#define NRF24L01_CSN_LOW HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET)
#define NRF24L01_CE_HIGH HAL_GPIO_WritePin(SI_EN_GPIO_Port, SI_EN_Pin, GPIO_PIN_SET)
#define NRF24L01_CE_LOW HAL_GPIO_WritePin(SI_EN_GPIO_Port, SI_EN_Pin, GPIO_PIN_RESET)
#define READ_NRF24L01_IRQ HAL_GPIO_ReadPin(SI_IRQ_GPIO_Port, SI_IRQ_Pin) // IRQ������������

/*****************************************���ͽ������ݿ�ȶ���***********************************************/
#define TX_ADR_WIDTH    5                               //5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5                               //5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  28                              //32�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  28                              //32�ֽڵ��û����ݿ��

/*******************************************�Ĵ�������ָ��**************************************************/
#define SPI_READ_REG    0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define SPI_WRITE_REG   0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define R_RX_PL_WID     0x60  //��ȡ�յ��������ֽ���
#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define W_ACK_PAYLOAD   0xA8  //���շ�,������ͨ��ACK����ʽ����ȥ�����������֡���ݴ���FIFO�С�
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���   

/*******************************************�Ĵ�����ַ****************************************************/
#define CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��                          
#define EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);
#define MAX_TX          0x10  //״̬�Ĵ���;bit4:�ﵽ����ʹ����ж�
#define TX_OK           0x20  //״̬�Ĵ���;bit5:TX��������ж�
#define RX_OK           0x40  //״̬�Ĵ���;bit6:���յ������ж�

#define OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD              0x09  //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define FIFO_STATUS     0x17  //FIFO״̬�Ĵ���;bit0:RX FIFO�Ĵ����ձ�־;bit1:RX FIFO����־;bit2,3:����
                              //FIFO״̬�Ĵ���;bit4:TX FIFO�ձ�־;bit5:TX FIFO����־;
                              //FIFO״̬�Ĵ���;bit6:1,ѭ��������һ���ݰ�/0,��ѭ��;

#define DYNPD           0x1C  //��̬���س��ȼĴ���;bit0����bit5��Ӧͨ��0-5�Ķ�̬���س���ʹ��
#define FEATURE         0x1D  //�����Ĵ���;bit1:ʹ��ACK����(���������ݵ�ACK��);bit2:ʹ�ܶ�̬���س���;

/**********************************************************************************************************/
#define MODEL_RX                1           //��ͨ����
#define MODEL_TX                2           //��ͨ����
#define MODEL_RX2               3           //����ģʽ2,����˫����
#define MODEL_TX2               4           //����ģʽ2,����˫����


extern uint16_t connect_flag;
extern uint8_t TX_BUFF[TX_PLOAD_WIDTH];
extern uint8_t RX_BUFF[RX_PLOAD_WIDTH];

uint8_t NRF24L01_TX_Mode(void);

uint8_t NRF24L01_RX_Mode(void);

uint8_t NRF24L01_TxPacket(uint8_t *txBuf);

uint8_t NRF24L01_RxPacket(uint8_t *txBuf);

uint8_t NRF24L01_Check();

#endif
