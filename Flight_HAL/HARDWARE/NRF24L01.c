/*
 * @Author: mashiro
 * @FilePath: NRF24L01.c
 * Copyright  All Rights Reserved.
 */
#include "NRF24L01.h"

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01}; // 定义一个静态发送地址
uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x0A, 0x01, 0x07, 0x0E, 0x01}; // 定义一个静态发送地址

uint8_t TX_BUFF[TX_PLOAD_WIDTH];
uint8_t RX_BUFF[RX_PLOAD_WIDTH];

/* 连接状态标志位，用于后续判断是否失联 */
uint16_t connect_flag = 1;

/**
 * @description: 写寄存器
 * @param {uint8_t} reg 寄存器地址
 * @param {uint8_t} data 要写入的字节
 * @return {*}
 */
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t data)
{
    uint8_t status = 0;
    /* 1、片选选中 */
    NRF24L01_CSN_LOW;
    /* 2、写寄存器地址 */
    status = Driver_SPI_SwapByte(reg);
    /* 3、写入数据 */
    Driver_SPI_SwapByte(data);
    /* 4、片选取消 */
    NRF24L01_CSN_HIGH;

    return status;
}

/**
 * @description: 读寄存器
 * @param {uint8_t} reg 要读取的寄存器
 * @return {*} 读取到的值
 */
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
    uint8_t res = 0;
    /* 1、片选选中 */
    NRF24L01_CSN_LOW;
    /* 2、写寄存器地址(读指令) */
    Driver_SPI_SwapByte(reg);
    /* 3、读取数据（写什么无所谓） */
    res = Driver_SPI_SwapByte(0);
    /* 4、片选取消 */
    NRF24L01_CSN_HIGH;

    return res;
}

/**
 * @description: 向指定寄存器写多个字节
 * @param {uint8_t} reg 写入的寄存器地址
 * @param {uint8_t*} pBuf   写入的多个字节数据指针
 * @param {uint8_t} len 数据字节数
 * @return {*}
 */
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    uint8_t status = 0;
    /* 1、片选选中 */
    NRF24L01_CSN_LOW;
    /* 2、写寄存器地址 */
    status = Driver_SPI_SwapByte(reg);
    /* 3、循环写入多个字节数据 */
    for (uint8_t i = 0; i < len; i++)
    {
        Driver_SPI_SwapByte(*pBuf++);
    }
    /* 4、片选取消 */
    NRF24L01_CSN_HIGH;

    return status;
}

/**
 * @description: 读多个字节
 * @param {uint8_t} reg 要读取的寄存器地址
 * @param {uint8_t *} pBuf  用来接收的数据缓冲的指针
 * @param {uint8_t} len 接收的字节数
 * @return {*}
 */
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
    /* 1、片选选中 */
    NRF24L01_CSN_LOW;
    /* 2、写寄存器地址(读指令) */
    Driver_SPI_SwapByte(reg);
    /* 3、循环读取多个字节数据（写什么无所谓） */
    for (uint8_t i = 0; i < len; i++)
    {
        *pBuf++ = Driver_SPI_SwapByte(0);
        printf("pBuf[%d] = %2x\r\n", i, pBuf[i]);
    }

    /* 4、片选取消 */
    NRF24L01_CSN_HIGH;

    return 0;
}

/**
 * @description: 发送模式初始化（参考例程、寄存器映射表）
 * @return {*}
 */
uint8_t NRF24L01_TX_Mode(void)
{
    /* 1、进入待机模式，CE=0 */
    NRF24L01_CE_LOW;
    /* 2、相关的配置：发送地址、接收管道0地址（一样）、ACK使能、使能管道0、功率、Config为发送模式 */
    NRF24L01_Write_Buf(SPI_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // 写入发送地址
    NRF24L01_Write_Buf(SPI_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 为了应答接收设备，接收通道0地址和发送地址相同

    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_AA, 0x01);      // 使能接收通道0自动应答
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_RXADDR, 0x01);  // 使能接收通道0
    NRF24L01_Write_Reg(SPI_WRITE_REG + SETUP_RETR, 0x0a); // 自动重发延时等待250us+86us，自动重发10次
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_CH, 40);        // 选择射频通道0x40
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x0f);   // 数据传输率2Mbps，发射功率7dBm
    NRF24L01_Write_Reg(SPI_WRITE_REG + CONFIG, 0x0e);     // CRC使能，16位CRC校验，上电
    /* 3、使能CE */
    NRF24L01_CE_HIGH;
    return 0;
}

/**
 * @description: 接收模式
 * @return {*}
 */
uint8_t NRF24L01_RX_Mode(void)
{
    NRF24L01_CE_LOW;
    /*
        与发送的区别：
            1、不需要设置发送地址
            2、需要设置接收通道0的负载长度
            3、config配置的，bit0=1 为接收模式
     */
    NRF24L01_Write_Buf(SPI_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 接收设备接收通道0使用和发送设备相同的发送地址
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_AA, 0x01);                          // 使能接收通道0自动应答
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_RXADDR, 0x01);                      // 使能接收通道0
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_CH, 40);                            // 选择射频通道0x40
    NRF24L01_Write_Reg(SPI_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);             // 接收通道0选择和发送通道相同有效数据宽度
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x0f);                       // 数据传输率2Mbps，发射功率7dBm
    NRF24L01_Write_Reg(SPI_WRITE_REG + CONFIG, 0x0f);                         // CRC使能，16位CRC校验，上电，接收模式
    NRF24L01_CE_HIGH;                                                         // 拉高CE启动接收设备
    return 0;
}

/**
 * @description: 发送一个数据包
 * @param {uint8_t} *txBuf
 * @return {*}
 */
uint8_t NRF24L01_TxPacket(uint8_t *txBuf)
{
    uint8_t state = 0;
    /* 1、使用写Tx FIFO指令，将数据发送 */
    NRF24L01_CE_LOW; // 拉低是为了确保不进入空闲模式 ps:具体见数据手册
    NRF24L01_Write_Buf(WR_TX_PLOAD, txBuf, TX_PLOAD_WIDTH);
    NRF24L01_CE_HIGH; // 确保进入发射模式

    /* 2、判断发送完成（或达到最大重发次数）：循环读取状态寄存器，并判断bit4、bit5 */
    while (!(state & (TX_OK | MAX_TX)))
    {
        state = NRF24L01_Read_Reg(SPI_READ_REG + STATUS); // 读取通道状态
    }

    /* 3、 清空 接收或最大重发次数 中断标志位 */
    NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, state);

    /* 4、达到最大重发次数，就要主动清除Tx FIFO，否则无法继续发送 */
    if (state & MAX_TX)
    {
        if (state & 0x01) // bit0: 如果TX FIFO满了，=1，没满=0
        {
            NRF24L01_Write_Reg(FLUSH_TX, 0xff);
        }
    }

    /* 5、如果是发送成功 */
    if (state & TX_OK)
    {
        return 0;
    }

    return 1; // 其他原因未成功
}

/**
 * @description: 接收一个数据包
 * @param {uint8_t} *txBuf  存读取的数据
 * @return {*}  0：成功读到数据；1：未读到数据
 */
uint8_t NRF24L01_RxPacket(uint8_t *txBuf)
{
    /* 1、读状态寄存器，判断是否收到数据了 */
    uint8_t state = 0;
    state = NRF24L01_Read_Reg(SPI_READ_REG + STATUS);

    /* 2、 清除 接收中断 标志 */
    NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, state);

    /* 3、如果收到数据了，就开始读 RX FIFO */
    if (state & RX_OK)
    {
        /* 2.1 从 RX FIFO读取数据到 buff 里 */
        NRF24L01_Read_Buf(RD_RX_PLOAD, txBuf, RX_PLOAD_WIDTH);
        /* 2.2 清空 RX FIFO */
        NRF24L01_Write_Reg(FLUSH_RX, 0xff);
        /* ！失联判断逻辑：加一个标志位 */
        connect_flag = 0;

        /* =============测试：打印接收的数据================= */
        for (uint8_t i = 0; i < RX_PLOAD_WIDTH; i++)
        {
            printf("receive[%d]=%02x\r\n", i, txBuf[i]);
        }
        /* ================================================ */

        return 0; // 成功接收数据
    }

    return 1; // 未接收到数据
}

/**
 * @description: 自检
 * @return {*}
 */
uint8_t NRF24L01_Check()
{
    uint8_t buff_w[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t buff_r[5] = {0};
    uint8_t count = 0;

    /* 1、往寄存器写入几个字节 */
    NRF24L01_Write_Buf(SPI_WRITE_REG + TX_ADDR, buff_w, 5);

    /* 2、从寄存器读出几个字节 */
    NRF24L01_Read_Buf(SPI_READ_REG + TX_ADDR, buff_r, 5);

    /* 3、判断是否相同 */
    for (uint8_t i = 0; i < 5; i++)
    {
        if (buff_r[i] == buff_w[i])
        {
            count++;
        }
    }

    if (count == 5)
    {
        return 0; // 校验成功
    }
    else
    {
        return 1; // 校验失败
    }
}