#include "delay.h"
#include "sys.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
//////////////////////////////////////////////////////////////////////////////////  
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32F429������
//ʹ��SysTick����ͨ����ģʽ���ӳٽ��й���(֧��ucosii/ucosiii)
//����delay_us,delay_ms
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/6/10
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
//All rights reserved
//********************************************************************************
//�޸�˵��
////////////////////////////////////////////////////////////////////////////////// 

static u32 fac_us=180;							//us��ʱ������						    

//��ʱnus
//nusΪҪ��ʱ��us��.	
//nus:0~190887435(���ֵ��2^32/fac_us@fac_us=22.5)	 
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD��ֵ	    	 
	ticks=nus*fac_us; 						//��Ҫ�Ľ����� 
	told=SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	};
}

//��ʱnms
//nms:Ҫ��ʱ��ms��
void delay_ms(u16 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}
			 


































