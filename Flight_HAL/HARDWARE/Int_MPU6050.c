/*
 * @Author: mashiro
 * @FilePath: Int_MPU6050.c
 * Copyright, All Rights Reserved.
 */
#include "Int_MPU6050.h"

/**
 * @description: 向MPU6050写一个字节
 * @param {uint8_t} reg_addr  内部寄存器的地址
 * @param {uint8_t} byte      要发送的1字节数据
 * @return {*}
 */
uint8_t Int_MPU6050_WriteByte(uint8_t reg_addr, uint8_t byte)
{

    HAL_I2C_Mem_Write(&hi2c1, MPU_IIC_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, &byte, 1, HAL_MAX_DELAY);
    return 0;
}

/**
 * @description: 向MPU6050读取一个字节
 * @param {uint8_t} reg_addr  内部寄存器的地址
 * @param {uint8_t} byte      保存读取的1字节数据
 * @return {*}
 */
uint8_t Int_MPU6050_ReadByte(uint8_t reg_addr, uint8_t *byte)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU_IIC_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, byte, 1, HAL_MAX_DELAY);
    return 0;
}

/**
 * @description: 向MPU6050写多个字节
 * @param {uint8_t} dev_addr  设备地址
 * @param {uint8_t} reg_addr  内部寄存器的地址
 * @param {uint8_t} data      要发送的多字节数据
 * @param {uint8_t} size      要发送的字节数
 * @return {*}
 */
uint8_t Int_MPU6050_Write_Len(uint8_t reg_addr, uint8_t *data, uint8_t size)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU_IIC_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
    return 0;
}

/**
 * @description: 向MPU6050读多个字节
 * @param {uint8_t} reg_addr  内部寄存器的地址
 * @param {uint8_t} data      接收读取的多字节数据
 * @param {uint8_t} size      要接收的字节数
 * @return {*}
 */
uint8_t Int_MPU6050_Read_Len(uint8_t reg_addr, uint8_t *data, uint8_t size)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU_IIC_ADDR << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
    return 0;
}

/**
 * @description: 设置陀螺仪的满量程
 * @param {uint8_t} fsr  满量程配置值：  3 ----- +-2000 °/s
 * @return {*}
 */
uint8_t Int_MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    /* bit4、bit3 */
    return Int_MPU6050_WriteByte(MPU_GYRO_CFG_REG, fsr << 3);
}

/**
 * @description: 设置加速度计的满量程
 * @param {uint8_t} fsr 满量程配置值：  0  --------- +- 2g
 * @return {*}
 */
uint8_t Int_MPU_Set_Accel_Fsr(uint8_t fsr)
{
    /* bit4、bit3 */
    return Int_MPU6050_WriteByte(MPU_ACCEL_CFG_REG, fsr << 3);
}

/**
 * @description: 设置低通滤波器的带宽（截至频率，低于他的才能过）
 * @param {uint16_t} band_width 设置的带宽
 * @return {*}
 */
uint8_t Int_MPU_Set_LPF(uint16_t band_width)
{
    uint8_t cfg = 0;
    /* 从高的阈值往低的阈值判断（从最宽容到最严格要求） */
    if (band_width >= 188)
    {
        cfg = 1;
    }
    else if (band_width >= 98)
    {
        cfg = 2;
    }
    else if (band_width >= 42)
    {
        cfg = 3;
    }
    else if (band_width >= 20)
    {
        cfg = 4;
    }
    else if (band_width >= 10)
    {
        cfg = 5;
    }
    else
    {
        cfg = 6;
    }
    /* bit2 --- bit0 */
    return Int_MPU6050_WriteByte(MPU_CFG_REG, cfg);
}

/**
 * @description: 设置陀螺仪的采样率 = 输出频率/(1+分频)；设置低通滤波带宽=采样率/2;
 * @param {uint16_t} rate 期望的采样值：4-1000（手册min=4,1k/8k考虑加速度1k）
 * @return {*}
 */
uint8_t Int_MPU_Set_Rate(uint16_t rate)
{
    uint8_t div = 0;
    /* 采样频率=输出频率（1k）/(1+分频) ==》算出分频 ===》 设置到寄存器中*/
    /* 为了以后的通用性，可以做一个阈值限制 */
    if (rate > 1000)
    {
        rate = 1000;
    }
    else if (rate < 4)
    {
        rate = 4;
    }

    /* 计算分频 = 输出频率（1k）/ 采样频率rate -1 */
    div = 1000 / rate - 1;
    /* 将计算得到的分频，写入 采样率分频计数器 */
    Int_MPU6050_WriteByte(MPU_SAMPLE_RATE_REG, div);
    /* 顺便设置以下低通滤波的带宽 = 采样率/2 */
    return Int_MPU_Set_LPF(rate / 2);
}

void Int_MPU6050_Init()
{
    uint8_t res = 0;
    /* 1、 初始化I2C2 */

    /* 2、 复位 */
    Int_MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0X80);
    /* 3、延迟一下 */
    HAL_Delay(100);
    /* 4、 唤醒 */
    Int_MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0X00);
    /* 5、设置满量程：陀螺仪（角速度），+- 2000 °/s ,设为11*/
    Int_MPU_Set_Gyro_Fsr(3);
    /* 6、设置满量程: 加速度， +- 2g，设为0 */
    Int_MPU_Set_Accel_Fsr(0);
    /* 7、禁用中断、FIFO、I2C AUX模式 */
    // Int_MPU6050_WriteByte(MPU_INT_EN_REG, 0X01);    // 开启数据准备中断
    // Int_MPU6050_WriteByte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
    Int_MPU6050_WriteByte(MPU_INT_EN_REG, 0X00); // 关闭中断
    Int_MPU6050_WriteByte(MPU_FIFO_EN_REG, 0X00);
    Int_MPU6050_WriteByte(MPU_USER_CTRL_REG, 0X00);
    /* 判断是否可用，如果可以读到器件ID，说明前面的初始化没有错误 */
    Int_MPU6050_ReadByte(MPU_DEVICE_ID_REG, &res);
    if (res == MPU_IIC_ADDR)
    {
        /* 8、时钟：默认8M晶振===》 X轴上的时钟 */
        Int_MPU6050_WriteByte(MPU_PWR_MGMT1_REG, 0X01);
        /* 8、陀螺仪的采样率\设置低通滤波带宽(采样率一半) ，如果频率500，2ms采样一次*/
        Int_MPU_Set_Rate(500);
        /* 9、加速度与陀螺仪都工作 */
        Int_MPU6050_WriteByte(MPU_PWR_MGMT2_REG, 0X00);
    }
}

/**
 * @description: 获取加速度的原始数据
 * @param {short} *ax
 * @param {short} *ay
 * @param {short} *az
 * @return {*}
 */
uint16_t Int_MPU6050_GetAccl(short *ax, short *ay, short *az)
{
    uint8_t acc_buff[6];
    /* 每个轴对应2个寄存器：高8位、低8位， 第一个是0x3B（X轴的高位） */
    Int_MPU6050_Read_Len(MPU_ACCEL_XOUTH_REG, acc_buff, 6);
    /*
        acc_buff[0]:X轴加速度的高8位，
        acc_buff[1]:X轴加速度的低8位，
        acc_buff[2]:Y轴加速度的高8位，
        acc_buff[3]:Y轴加速度的低8位，
        acc_buff[4]:Z轴加速度的高8位，
        acc_buff[5]:Z轴加速度的低8位，
     */
    *ax = ((short)acc_buff[0] << 8) | acc_buff[1];
    *ay = ((short)acc_buff[2] << 8) | acc_buff[3];
    *az = ((short)acc_buff[4] << 8) | acc_buff[5];
}

/**
 * @description: 获取陀螺仪的角速度原始数据
 * @param {short} *gx
 * @param {short} *gy
 * @param {short} *gz
 * @return {*}
 */
uint16_t Int_MPU6050_GetGyro(short *gx, short *gy, short *gz)
{
    uint8_t gyro_buff[6];
    /* 每个轴对应2个寄存器：高8位、低8位， 第一个是0x43（X轴的高位） */
    Int_MPU6050_Read_Len(MPU_GYRO_XOUTH_REG, gyro_buff, 6);
    /*
        gyro_buff[0]:X轴角速度的高8位，
        gyro_buff[1]:X轴角速度的低8位，
        gyro_buff[2]:Y轴角速度的高8位，
        gyro_buff[3]:Y轴角速度的低8位，
        gyro_buff[4]:Z轴角速度的高8位，
        gyro_buff[5]:Z轴角速度的低8位，
     */
    *gx = ((short)gyro_buff[0] << 8) | gyro_buff[1];
    *gy = ((short)gyro_buff[2] << 8) | gyro_buff[3];
    *gz = ((short)gyro_buff[4] << 8) | gyro_buff[5];
}
