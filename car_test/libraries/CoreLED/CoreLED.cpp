// 
// 
// 

#include "CoreLED.h"

_CoreLED CoreLED;

_CoreLED::_CoreLED()
{
}

_CoreLED::~_CoreLED()
{
}

/************************************************************************************************************
【函 数 名】：	Initialization		核心板LED接口初始化函数
【参数说明】：	无
【返 回 值】：	无
【简    例】：	Initialization();	初始化核心板LED接口
************************************************************************************************************/
void _CoreLED::Initialization(void)
{
	/*核心板LED端口 A0/1/2/3*/
	PORTF |= 0x0f;
	DDRF  |= 0x0f;
	PORTF &= 0xf0;
	TurnOnOff(0x00);
}

/************************************************************************************************************
【函 数 名】：	TurnOn		打开核心板对应编号的LED灯函数
【参数说明】：	s：			核心板LED灯编号：1、2、3、4
【返 回 值】：	无
【简    例】：	TurnOn(1);	点亮核心板LED1
************************************************************************************************************/
void _CoreLED::TurnOn(uint8_t s)
{
	switch (s)
	{
	case LED1:
		PORTF |= _BV(PORTF0);
		break;
	case LED2:
		PORTF |= _BV(PORTF1);
		break;
	case LED3:
		PORTF |= _BV(PORTF2);
		break;
	case LED4:
		PORTF |= _BV(PORTF3);
		break;
	default:
		break;
	}
}

/************************************************************************************************************
【函 数 名】：	TurnOff			关闭核心板对应编号的LED灯函数
【参数说明】：	s：				核心板LED灯编号：1、2、3、4
【返 回 值】：	无
【简    例】：	TurnOff(1);		关闭核心板LED1
************************************************************************************************************/
void _CoreLED::TurnOff(uint8_t s)
{
	switch (s)
	{
	case LED1:
		PORTF &= ~_BV(PORTF0);
		break;
	case LED2:
		PORTF &= ~_BV(PORTF1);
		break;
	case LED3:
		PORTF &= ~_BV(PORTF2);
		break;
	case LED4:
		PORTF &= ~_BV(PORTF3);
		break;
	default:
		break;
	}
}

/************************************************************************************************************
【函 数 名】：	TurnOnOff			同时操作核心板4个LED灯函数
【参数说明】：	s：					s的低四位表示LED灯的状态，1表示亮，·0表示灭
【返 回 值】：	无
【简    例】：	TurnOnOff(0x01);	led1亮，led2、led3和led4灭
************************************************************************************************************/
void _CoreLED::TurnOnOff(uint8_t s)
{
	PORTF &= 0xf0;
	PORTF |= (s & 0x0f);
}
