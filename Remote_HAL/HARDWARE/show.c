#include "show.h"
#include "oledfont.h" 
#include "oled.h"

#define Line1_Begin 29
#define Line2_Begin 5
#define Line3_Begin 5
#define Line4_Begin 30
#define Line5_Begin 2

#define X_Begin 0
#define Y_Begin 51
#define Z_Begin 103

#define Line1_Begin1 0
#define Line2_Begin1 0
#define Line3_Begin1 40
#define Line4_Begin1 0
#define Line5_Begin1 0

#define Y0 0
#define Y1 14
#define Y2 Y1+12
#define Y3 Y2+12
#define Y4 Y3+12
#define Y5 Y4+12

struct _Show Show;

// unsigned char i;          //计数变量

/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
	uint8_t temp;
	int temp1;
	
	static uint8_t page,page_temp,flash_cnt,show_mode=1;
	
	if(page != page_temp)//切换页面先清屏
	{
		page_temp = page;
		OLED_Clear();
	}
	///////////////////////////////第一行///////////////////////////////////
    OLED_Show_CH(Line1_Begin+00,Y0,0,12,1);
    OLED_Show_CH(Line1_Begin+12,Y0,1,12,1);
    OLED_Show_CH(Line1_Begin+24,Y0,2,12,1);
    OLED_Show_CH(Line1_Begin+36,Y0,3,12,1);
    OLED_Show_CH(Line1_Begin+48,Y0,4,12,1);
    OLED_Show_CH(Line1_Begin+60,Y0,5,12,1);

    OLED_ShowNumber(2,Y0,40,3,12);//显示无线信道
	///////////////////////////////第二行///////////////////////////////////

	///////////////////////////////第三、四行/////////////////////////////////
	//显示遥控数据
	OLED_ShowString(Line3_Begin+00,Y2,"THR:",12,1);
	temp = (rc.THR-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+6 ,Y2,12,1);
	temp = (rc.THR-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+18,Y2,12,1);
	
	OLED_ShowString(Line3_Begin+64,Y2,"ROL:",12,1);
	temp = (rc.ROL-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+70,Y2,12,1);
	temp = (rc.ROL-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+82,Y2,12,1);
	
	OLED_ShowString(Line3_Begin+00,Y3,"YAW:",12,1);
	temp = (rc.YAW-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+6 ,Y3,12,1);
	temp = (rc.YAW-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+18,Y3,12,1);
	
	OLED_ShowString(Line3_Begin+64,Y3,"PIT:",12,1);
	temp = (rc.PIT-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+70,Y3,12,1);
	temp = (rc.PIT-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+82,Y3,12,1);
	///////////////////////////////第五行///////////////////////////////////

	//显示微调旋钮数据
	OLED_ShowString(Line5_Begin+00,Y4,"R:",12,1);
	temp = (offset.ROL-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line5_Begin+15 ,Y4,12,1);
	temp = (offset.ROL-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line5_Begin+27,Y4,12,1);
		
	OLED_ShowString(Line5_Begin+44,Y4,"Y:",12,1);
	temp = (offset.YAW-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line5_Begin+59 ,Y4,12,1);
	temp = (offset.YAW-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line5_Begin+71,Y4,12,1);
		
	OLED_ShowString(Line5_Begin+88,Y4,"P:",12,1);
	temp = (offset.PIT-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line5_Begin+103 ,Y4,12,1);
	temp = (offset.PIT-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line5_Begin+115,Y4,12,1);

	
	OLED_Refresh_Gram();//开始显示
}

//进度条显示函数
void OLED_Show_progress_bar(uint8_t temp,uint8_t chr_star,uint8_t chr_default,uint8_t x,uint8_t y,uint8_t size,uint8_t mode)
{
	switch(temp)
	{
		case  0:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  1:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  2:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  3:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  4:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  5:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  6:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  7:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  8:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  9:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case 10:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case 11:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case 12:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		
		default:OLED_Show_CH(x,y,chr_default,size,size);break;
	}
}
