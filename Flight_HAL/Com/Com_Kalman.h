#ifndef _COM_KALMAN_H
#define _COM_KALMAN_H




struct _1_ekf_filter
{
    float LastP;    //上一时刻的状态方差（或协方差）
    float Now_P;    //当前时刻的状态方差（或协方差）
    float out;      //滤波器的输出值，即估计的状态
    float Kg;       //卡尔曼增益，用于调节预测值和测量值之间的权重
    float Q;        //过程噪声的方差，反映系统模型的不确定性
    float R;        //测量噪声的方差，反映测量过程的不确定性
};

extern struct _1_ekf_filter ekf[3];

extern void Com_Kalman_1(struct _1_ekf_filter *ekf,float input);  //一维卡尔曼

#endif


