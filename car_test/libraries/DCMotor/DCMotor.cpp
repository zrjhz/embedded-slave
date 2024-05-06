#include "DCMotor.h"
#include <Command.h>
#include "wiring_private.h"
#include <ExtSRAMInterface.h>
#include <Metro.h>
#include <BEEP.h>
#include "Pid.h"

_DCMotor DCMotor;
Metro DCMotorMetro(20);

_DCMotor::_DCMotor()
{
	ExtSRAMInterface.Initialization();
}

_DCMotor::~_DCMotor()
{
}

/************************************************************************************************************
【函 数 名】：	Initialization	直流电机初始化
【参数说明】：	fHz	：		初始化PWM输出频率，单位：Hz
【返 回 值】：	无
【简    例】：	Initialization(30000);
************************************************************************************************************/
void _DCMotor::Initialization(uint32_t fHz)
{
	/*ExtSRAMInterface.Initialization();*/
	pinMode(L_CONTROL_PIN, OUTPUT);
	pinMode(R_CONTROL_PIN, OUTPUT);

	pinMode(R_F_M_PIN, OUTPUT);
	pinMode(R_B_M_PIN, OUTPUT);
	pinMode(L_F_M_PIN, OUTPUT);
	pinMode(L_B_M_PIN, OUTPUT);
	digitalWrite(L_CONTROL_PIN, HIGH);
	digitalWrite(R_CONTROL_PIN, HIGH);

	// 选择工作模式,模式15--fast PWM 该模式下即重装载值为OCRnA（输出比较寄存器）
	TCCR4A |= _BV(WGM41) | _BV(WGM40);
	TCCR4B |= _BV(WGM42) | _BV(WGM43);

	TCCR3A |= _BV(WGM31) | _BV(WGM30);
	TCCR3B |= _BV(WGM32) | _BV(WGM33);

	// 设置输出通道
	//  B C 通道计数器值小于比较寄存器时引脚输出1，大于OCR比较寄存器值时输出0
	TCCR4A |= _BV(COM4C1) | _BV(COM4B1);
	TCCR3A |= _BV(COM3C1) | _BV(COM3B1);

	// 设置PWM波的频率
	MotorFrequency(fHz);
	ParameterInit();
}

/************************************************************************************************************
【函 数 名】：  MotorFrequency	设置PWM波的频率，占空比保持最后一次设置值
【参数说明】：	_fHz	：
【返 回 值】：	无
【简    例】：	MotorFrequency(30000);
************************************************************************************************************/
void _DCMotor::MotorFrequency(uint32_t _fHz)
{
	// 配置预分频系数 0 0 1 不分频
	TCCR4B &= ~(_BV(CS40) | _BV(CS41) | _BV(CS42));
	TCCR4B |= _BV(CS40);
	TCCR3B &= ~(_BV(CS30) | _BV(CS31) | _BV(CS32));
	TCCR3B |= _BV(CS30);

	// f=16Mhz(mega2560时钟源)/(预分频系数)/(重装载值）
	fHz = 16000000 / _fHz;
	// 模式15--fast PWM该模式下即重装载值为OCRnA（输出比较寄存器）
	OCR3A = fHz;
	OCR4A = fHz;

	// 将BC通道值都置为0，通讯显示板正反转的显示：是通过判断对应引脚是否有脉冲，优先判断反转的引脚脉冲
	TCCR3A |= _BV(COM3B0);
	OCR3B = fHz;
	TCCR3A |= _BV(COM3C0);
	OCR3C = fHz;
	TCCR4A |= _BV(COM4B0);
	OCR4B = fHz;
	TCCR4A |= _BV(COM4C0);
	OCR4C = fHz;
}

/************************************************************************************************************
【函 数 名】：  ParameterInit	分配速度对应的比较数值
【参数说明】：	_fHz：
【返 回 值】：	无
【简    例】：	ParameterInit();
************************************************************************************************************/
void _DCMotor::ParameterInit(void)
{
	for (uint8_t i = 1; i < 101; i++) // 占空比初始化
	{
		speed[i] = (fHz * 0.6) + (fHz * 0.4) * i / 100; // *0.6 因为电机在60%以上时扭矩才足以驱动小车
	}
	speed[0] = 0;
}

void _DCMotor::SpeedSetOne(int16_t s, uint16_t *c1, uint16_t *c2)
{
	uint8_t t;
	t = (s >= 0) ? s : s * (-1);
	if (t > 100)
		t = 100;
	if (t < 5)
		t = 5;
	if (s == 0)
	{
		*c1 = speed[100]; // 100;
		*c2 = speed[100]; // 100;
	}
	else if (s > 0)
	{
		*c1 = speed[t];
		*c2 = speed[0];
	}
	else
	{
		*c1 = speed[0];
		*c2 = speed[t];
	}
}

void _DCMotor::SpeedCtr(int16_t L_speed, int16_t R_speed)
{
	uint16_t ocr3b, ocr3c, ocr4b, ocr4c;

	SpeedSetOne(L_speed, &ocr4c, &ocr4b);
	SpeedSetOne(R_speed, &ocr3b, &ocr3c);

	(ocr3b == 0) ? (TCCR3A |= _BV(COM3B0), ocr3b = fHz) : (TCCR3A &= ~_BV(COM3B0));
	(ocr3c == 0) ? (TCCR3A |= _BV(COM3C0), ocr3c = fHz) : (TCCR3A &= ~_BV(COM3C0));
	(ocr4b == 0) ? (TCCR4A |= _BV(COM4B0), ocr4b = fHz) : (TCCR4A &= ~_BV(COM4B0));
	(ocr4c == 0) ? (TCCR4A |= _BV(COM4C0), ocr4c = fHz) : (TCCR4A &= ~_BV(COM4C0));

	OCR4C = ocr4c;
	OCR4B = ocr4b;
	OCR3C = ocr3c;
	OCR3B = ocr3b;
}

bool _DCMotor::ClearCodeDisc(void)
{
	uint16_t distance;
	unsigned long t;
	Command.Judgment(Command.command01);
	for (size_t i = 0; i < 8; i++)
	{
		ExtSRAMInterface.ExMem_JudgeWrite(WRITEADDRESS + i, Command.command01[i]);
	}
	DCMotorMetro.interval(20);
	for (size_t i = 0; i < 5; i++)
	{
		if (DCMotorMetro.check() == 1)
		{
			distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
			if (distance == 0x0000)
			{
				return false;
			}
		}
	}
	return true;
}

/************************************************************************************************************
小车动作的相关函数
************************************************************************************************************/
/************************************************************************************************************
【函 数 名】：  Go		小车前进函数
【参数说明】：	speed	：设置速度
distance: 设置前进的距离
【返 回 值】：	无
【简    例】：	Go(70);	小车动作：前进，前进速度：70
************************************************************************************************************/
void _DCMotor::Go(uint8_t speed)
{
	SpeedCtr(speed, speed);
}
uint16_t _DCMotor::Go(uint8_t speed, uint16_t _distance)
{
	unsigned long t;
	uint16_t distance;
	while (ClearCodeDisc())
	{
	}
	SpeedCtr(speed, speed);
	t = millis();
	do
	{
		distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
		if ((65516 > distance) && (distance > 20))
		{
			if ((distance >= _distance) || ((millis() - t) >= 30000))
			// if ((distance >= _distance) || ((millis() - t) >= 30000) || (ExtSRAMInterface.ExMem_Read(0x6100) == 0x55))
			{
				Stop();
				delay(50);
				distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
				break;
			}
		}
		delay(1);
	} while (true);
	return distance;
}

uint16_t _DCMotor::GoSelfDefine(uint8_t left_speed, uint8_t right_speed, uint16_t _distance)
{
	unsigned long t;
	uint16_t distance;
	while (ClearCodeDisc())
	{
	}
	SpeedCtr(left_speed, right_speed);
	t = millis();
	do
	{
		distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
		if ((65516 > distance) && (distance > 20))
		{
			if ((distance >= _distance) || ((millis() - t) >= 30000))
			// if ((distance >= _distance) || ((millis() - t) >= 30000) || (ExtSRAMInterface.ExMem_Read(0x6100) == 0x55))
			{
				Stop();
				delay(50);
				distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
				break;
			}
		}
		delay(1);
	} while (true);
	return distance;
}

/************************************************************************************************************
【函 数 名】：  Back		 小车后退函数
【参数说明】：	speed	:	 设置速度
			   distance:	设置后退的距离
【返 回 值】：	无
【简    例】：	Back(70);	小车动作：后退，后退速度：70
************************************************************************************************************/
void _DCMotor::Back(uint8_t speed)
{
	SpeedCtr(speed * (-1), speed * (-1));
}
uint16_t _DCMotor::Back(uint8_t speed, uint16_t _distance)
{
	unsigned long t;
	uint16_t distance;

	while (ClearCodeDisc())
	{
	}
	SpeedCtr(speed * (-1), speed * (-1));
	t = millis();
	do
	{
		distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
		if ((65516 > distance) && (distance > 20))
		{
			if (((65536 - distance) >= _distance) || ((millis() - t) > 30000))
			{
				Stop();
				delay(50);
				distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
				break;
			}
		}
		delay(10);
	} while (true);
	return (65536 - distance);
}

uint16_t _DCMotor::BackSelfDefine(uint8_t left_speed, uint8_t right_speed, uint16_t _distance)
{
	unsigned long t;
	uint16_t distance;

	while (ClearCodeDisc())
	{
	}
	SpeedCtr(-left_speed, -right_speed);
	t = millis();
	do
	{
		distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
		if ((65516 > distance) && (distance > 20))
		{
			if (((65536 - distance) >= _distance) || ((millis() - t) > 30000))
			{
				Stop();
				delay(50);
				distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
				break;
			}
		}
		delay(10);
	} while (true);
	return (65536 - distance);
}

/************************************************************************************************************
【函 数 名】：  TurnLeft	小车左转函数,Lspeed <= Rspeed
【参数说明】：	Lspeed	：	设置左轮速度
Rspeed	：	设置右轮速度
【返 回 值】：	无
【简    例】：	TurnLeft(70);	小车动作：左转，左转速度：70
************************************************************************************************************/
void _DCMotor::TurnLeft(int8_t Lspeed, int8_t Rspeed)
{
	SpeedCtr(Lspeed * (-1), Rspeed);
}
void _DCMotor::TurnLeft(int8_t speed)
{
	uint8_t tgd, tp;
	unsigned long t;
	uint8_t trackval;
	while (ClearCodeDisc())
	{
	}
	SpeedCtr(speed * (-1), speed);
	do
	{
		tgd = ExtSRAMInterface.ExMem_Read(BASEADDRESS + TRACKOFFSET);
		tp = SearchBit(1, tgd);
		if (tp <= 0x04)
			break;
	} while (true);
	t = millis();
	do
	{
		trackval = ExtSRAMInterface.ExMem_Read(BASEADDRESS + TRACKOFFSET);
		if ((!(trackval & 0x10)) || ((millis() - t) > 10000))
		// if (((!(trackval & 0x10)) || ((millis() - t) > 10000)) || (ExtSRAMInterface.ExMem_Read(0x6100) != 0x00))
		{
			Stop();
			break;
		}
	} while (true);
}

/************************************************************************************************************
【函 数 名】：  TurnLeft_Code	 小车码盘左转函数
【参数说明】：	speed：		 	  设置转向速度
			   _distance	：	 设置码盘距离
【返 回 值】：	无
【简    例】：	TurnLeft_Code(50, 500);
************************************************************************************************************/
uint16_t _DCMotor::TurnLeft_Code(uint8_t speed, uint16_t _distance)
{
	unsigned long t;
	uint16_t distance;
	while (ClearCodeDisc())
	{
	}
	SpeedCtr(-speed, speed);

	t = millis();
	do
	{
		distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
		if ((65516 > distance) && (distance > 20))
		{
			// if ((65536 - distance >= _distance) || ((millis() - t) >= 30000) || (ExtSRAMInterface.ExMem_Read(0x6100) == 0x55))
			if ((65536 - distance >= _distance) || ((millis() - t) >= 30000))
			{
				Stop();
				delay(50);
				distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
				break;
			}
		}
		delay(1);
	} while (true);
	return distance;
}

/************************************************************************************************************
【函 数 名】：  TurnRight_Code	 小车码盘右转函数
【参数说明】：	speed	：		 设置转向速度
			   _distance	：	设置码盘距离
【返 回 值】：	无
【简    例】：	TurnRight_Code(50, 500);
************************************************************************************************************/
uint16_t _DCMotor::TurnRight_Code(uint8_t speed, uint16_t _distance)
{
	unsigned long t;
	uint16_t distance;
	while (ClearCodeDisc())
	{
	}
	SpeedCtr(speed, -speed);

	t = millis();
	do
	{
		distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
		if ((65516 > distance) && (distance > 20))
		{
			if ((distance >= _distance) || ((millis() - t) >= 30000))
			// if ((distance >= _distance) || ((millis() - t) >= 30000) || (ExtSRAMInterface.ExMem_Read(0x6100) == 0x55))
			{
				Stop();
				delay(50);
				distance = uint16_t(ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET) + (ExtSRAMInterface.ExMem_Read(BASEADDRESS + CODEOFFSET + 1) << 8));
				break;
			}
		}
		delay(1);
	} while (true);
	return distance;
}

/************************************************************************************************************
【函 数 名】：  TurnRight	小车右转函数,Rspeed <= Lspeed
【参数说明】：	Lspeed	：	设置左轮速度
Rspeed	：	设置右轮速度
【返 回 值】：	无
【简    例】：	TurnRight(70);	小车动作：右转，右转速度：70
************************************************************************************************************/
void _DCMotor::TurnRight(int8_t Lspeed, int8_t Rspeed)
{
	SpeedCtr(Lspeed, Rspeed * (-1));
}
void _DCMotor::TurnRight(int8_t speed)
{
	uint8_t tgd, tp;
	unsigned long t;
	uint8_t trackval;
	while (ClearCodeDisc())
	{
	}

	SpeedCtr(speed, speed * (-1));

	do
	{
		tgd = ExtSRAMInterface.ExMem_Read(BASEADDRESS + TRACKOFFSET);
		tp = SearchBit(0, tgd);
		if (tp >= 0x20)
			break;
	} while (true);

	t = millis();
	do
	{
		trackval = ExtSRAMInterface.ExMem_Read(BASEADDRESS + TRACKOFFSET);
		if (((!(trackval & 0x08)) || ((millis() - t) > 10000)))
		// if (((!(trackval & 0x08)) || ((millis() - t) > 10000)) || (ExtSRAMInterface.ExMem_Read(0x6100) != 0x00))
		{
			Stop();
			break;
		}
	} while (true);
}

/************************************************************************************************************
【函 数 名】：  Stop	小车停止函数
【参数说明】：	无
【返 回 值】：	无
【简    例】：	Stop();	小车动作：停止
************************************************************************************************************/
void _DCMotor::Stop(void)
{
	// 电机锁死
	SpeedCtr(0, 0);
	OCR4C = fHz;
	OCR4B = fHz;
	OCR3C = fHz;
	OCR3B = fHz;
	// PORTE |= _BV(PE3);
	// PORTH |= _BV(PH3);
	/**********END************/
}

uint8_t _DCMotor::SearchBit(uint8_t mode, uint8_t s)
{
	if (mode == 1)
	{
		for (size_t i = 0x80; i > 0x00; i >>= 1)
		{
			if ((s & i) == 0)
				return i;
		}
		return 0;
	}
	else
	{
		for (size_t i = 0x01; i < 0x100; i <<= 1)
		{
			if ((s & i) == 0)
				return i;
		}
		return 0xff;
	}
}

uint8_t _DCMotor::Count_Number(uint8_t count, uint8_t limit_number)
{
	uint8_t number = 0;
	for (uint8_t j = 0; j < limit_number; j++)
	{
		if ((count % 2) == 0)
		{
			number = number + 1;
		}
		count = count / 2;
	}
	return number;
}

uint8_t _DCMotor::Return_Cross(uint8_t trackeight, uint8_t tracksiven)
{
	uint8_t res = 0, array_s[7], array_e[8];
	for (uint8_t j = 0; j < 8; j++)
	{
		if (trackeight % 2 == 1)
		{
			array_e[j] = 1;
		}
		else
		{
			array_e[j] = 0;
		}
		if (j < 7)
		{
			if (tracksiven % 2 == 1)
			{
				array_s[j] = 1;
			}
			else
			{
				array_s[j] = 0;
			}
		}
		trackeight = trackeight / 2;
		tracksiven = tracksiven / 2;
	}
	if (array_e[0] == 0 || array_e[7] == 0 || array_s[0] == 0 || array_e[6] == 0)
	{
		res = 1;
	}
	else
	{
		res = 0;
	}
	return res;
}

uint8_t _DCMotor::Cumulate_WORB_FirstNumber(uint8_t track_eight, uint8_t track_seven)
{
	unsigned eight_array[8], seven_array[7], flag_A = 0, flag_B = 0, flag_C = 0, flag_D = 0, white_mark[4] = {0, 0, 0, 0}, black_mark[4] = {0, 0, 0, 0}, res = 0, state[2] = {0, 0};
	for (int i = 0; i < 8; i++) // 将读取的循迹值存放于数组eight_array和seven_array中
	{
		if (track_eight % 2 == 1)
		{
			eight_array[i] = 1;
		}
		else
		{
			eight_array[i] = 0;
		}
		if (i < 7)
		{
			if (track_seven % 2 == 1)
			{
				seven_array[i] = 1;
			}
			else
			{
				seven_array[i] = 0;
			}
		}
		track_eight = track_eight / 2;
		track_seven = track_seven / 2;
	}

	for (int i = 0; i < 8; i++)
	{
		if (i != 0)
		{
			if (eight_array[i] != eight_array[i - 1])
			{
				state[0] += 1;
			}
		}
		if (flag_A == 0 && eight_array[i] == 1) // 记录八路循迹白色第一次出现的位置
		{
			white_mark[0] = i + 1;
			flag_A = 1;
		}
		if (flag_C == 0 && eight_array[i] == 0) // 记录八路循迹黑色第一次出现的位置
		{
			black_mark[0] = i + 1;
			flag_C = 1;
		}
		if (flag_B == 0 && eight_array[i] == 0 && eight_array[i - 1] == 1 && i != 0) // 记录八路循迹白色第一次结束的位置
		{
			white_mark[1] = i;
			flag_B = 1;
		}
		if (flag_D == 0 && eight_array[i] == 1 && eight_array[i - 1] == 0 && i != 0) // 记录八路循迹黑色第一次结束的位置
		{
			black_mark[1] = i;
			flag_D = 1;
		}
	}

	if (white_mark[1] == 0 && white_mark[0] != 0)
		white_mark[1] = 8; // 全白
	if (black_mark[1] == 0 && black_mark[0] != 0)
		black_mark[1] = 8; // 全黑

	flag_A = 0;
	flag_B = 0;
	flag_C = 0;
	flag_D = 0;

	for (int i = 0; i < 7; i++)
	{
		if (i != 0)
		{
			if (seven_array[i] != seven_array[i - 1])
			{
				state[1] = state[1] + 1;
			}
		}
		if (flag_A == 0 && seven_array[i] == 1) // 记录七路循迹白色第一次出现的位置
		{
			white_mark[2] = i + 1;
			flag_A = 1;
		}
		if (flag_C == 0 && seven_array[i] == 0) // 记录七路循迹白色第一次出现的位置
		{
			black_mark[2] = i + 1;
			flag_C = 1;
		}
		if (flag_B == 0 && seven_array[i] == 0 && seven_array[i - 1] == 1 && i != 0) // 记录七路循迹白色第一次结束的位置
		{
			white_mark[3] = i;
			flag_B = 1;
		}
		if (flag_D == 0 && seven_array[i] == 1 && seven_array[i - 1] == 0 && i != 0) // 记录七路循迹黑色第一次结束的位置
		{
			black_mark[3] = i;
			flag_D = 1;
		}
	}

	if (white_mark[2] == 0 && white_mark[3] != 0)
		white_mark[1] = 7; // 全白
	if (black_mark[2] == 0 && black_mark[3] != 0)
		black_mark[1] = 7; // 全黑

	switch (state[1])
	{
	case 0:
		if (seven_array[0] == 0)
			res = 1; // 全黑
		else
			res = 2; // 全白
		break;
	case 1:
		if (seven_array[0] == 0)
			res = 3; // 白黑
		else
			res = 4; // 黑白
		break;
	case 2:
		if (seven_array[0] == 0)
			res = 5; // 黑白黑
		else
			res = 6; // 白黑白
		break;
	default:
		res = 7;
		break;
	}

	return res;
}
