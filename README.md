# Quadcopter-drone

### 1.1 项目概述

本项目是基于STM32的微型四轴无人机，相较于中大型四轴无人机，微型四轴无人机具有成本低、事故代价低、结构简单和量产率高等优势。控制核心采用STM32F103C8T6，姿态运动传感器选择MPU6050。无人机通过Si24R1（NRF24L01）与控制器进行2.4G无线通信，实现了即时有效地接收控制器指令，通过串级PID进行姿态控制，从而在空间中实现自由移动。

### 1.2 功能描述

### 1.2.1 姿态控制

通过MPU6050获取三轴加速度和三轴角速度，经过四元数姿态解算得到三种倾角。利用串级PID算法控制四轴无人机平衡状态。

### 1.2.2 实时操作系统

##### 移植FreeRTOS实现多任务调度和管理。

### 1.2.3 数据显示

把测得电池电压、摇杆大小、2.4G信道数据实时显示在液晶屏幕上。

### 1.2.4 遥控控制

遥控板与飞控板通过2.4G信号交互，发送指令，控制四轴无人机上升下降、前进后退、左右移动。

## 1.3 系统总体设计

1）飞控板主要硬件

MCU：STM32F103C8T6

传感器：MPU6050六轴传感器

2.4G通讯：Si24R1国产

螺旋桨：带有刷电机，X型布局

电池：3.7V，2000mAh

其他：开关、LED等

2）遥控板主要硬件

MCU：STM32F103C8T6

2.4G通讯：Si24R1国产

显示屏：12864OLED

摇杆：左油门，右方向

按键：微调、对频

电池：3.7V，2000mAh

其他：开关、LED等

## 1.4 飞控相关理论

### 1.1.1 常用姿态解算

| **方法**             | **描述**                                                     | **优点**                                                     | **缺点**                                           |
| -------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ | -------------------------------------------------- |
| 角速度积分（欧拉角） | 类似二维平面上∠θ =ω *t。 利用三维旋转矩阵映射到三维时空。    | 原理和运算量都简单，稳定性好                                 | 长时间积分会出现积分偏差。欧拉角存在万向锁的问题。 |
| 加速度解算           | 类似二维平面反三角函数∠θ  =arctan(a 对边/b 临边)。三维旋转矩阵映射到三维时空 | 原理简单好理解                                               | 容易受振动影响                                     |
| 互补解算             | 由于单一传感器获得角度都有缺点，那就综合上述二者             | 综合了上述两者的优点                                         | 会受振动影响，并且动态性能差，大角度变化反应慢     |
| 四元数姿态解算       | 通过某个固定的算法得到角度                                   | 算法固定，数学理论 成熟，解算准确，计 算量小，效果直逼卡 尔曼 | 会受振动影响，角度回复有延迟。算法不容易理解       |
| 卡尔曼姿态评估       | 所谓评估就是综合某些因素，去估计姿态，就如同一部车驶来，你会根据它的速度评估下一秒的位置。 | 动态性能好                                                   | 模型复杂，计算量大， 理论复杂。参数 难调           |
