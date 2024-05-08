#include <BEEP.h>
#include <BH1750.h>
#include <BKRC_Voice.h>
#include <Command.h>
#include <CoreBeep.h>
#include <CoreKEY.h>
#include <CoreLED.h>
#include <DCMotor.h>
#include <ExtSRAMInterface.h>
#include <Infrare.h>
#include <LED.h>
#include <Ultrasonic.h>
#include <math.h>
#include <string.h>
#include <Pid.h>

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 宏定义 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
#define MAX 70            // 路径个数
#define TurnAngle 6.48    // 延时转弯(转换为角度得比例)
#define TurnCarSpeed 70   // 转弯速度
#define CarTrackSpeed 50  // 循迹速度
#define Turn_Code 700     // 转弯码盘值
#define Code_ToLength 25  // 码盘转厘米
#define Speed50_Delay 29  // 29ms = 1CM(延时前进，速度为50)
#define Ultrasonic_Flag 1 // 超声波测量  1：返回厘米  0：返回英寸

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 枚举定义 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
typedef enum
{
  AGV_UP = 1,         // 上
  AGV_LEFT = 2,       // 左
  AGV_DOWN = 3,       // 下
  AGV_RIGHT = 4,      // 右
  AGV_LEFT_UP = 5,    // 左上
  AGV_LEFT_DOWN = 6,  // 左下
  AGV_RIGHT_UP = 7,   // 右上
  AGV_RIGHT_DOWN = 8, // 右下
} Direction_t;        // 方向
typedef enum
{
  ToMainCar_Ultrasonic = 0X0B,    // 超声波距离
  ToMainCar_InitLedLevel = 0X0A,  // 初始路灯档位
  ToMainCar_BeaconAlarm = 0X10,   // 烽火台开启码
  ToMainCar_Movement = 0XB0,      // 发送启动命令给主车
  ToMainCar_QRSent = 0x92,        // 二维码发送
  ToMainCar_MisonComplete = 0xFF, // 任务完成
  ToMainCar_CarPlate = 0x15,      // 车牌
  ToMainCar_BGarageFloor = 0x2B,  // B车库初始层数
  ToMainCar_AGarageFloor = 0x2A,  // A车库初始层数key
} Tomain_Maincmd;                 // 发送数据
typedef enum
{
  ReceiverMainCar_AGV_Movement = 0xD0,     // 从车启动
  ReceiverMainCar_AGV_Quitwait = 0xB0,     // 退出等待
  ReceiverMainCar_AGV_Forward = 0x71,      // 从车方向定义
  ReceiverMainCar_AGV_Route = 0xE1,        // 从车路径
  ReceiverMainCar_AGV_NextRoute = 0xE3,    // 从车路径
  ReceiverMainCar_AlarmFrontCode = 0x10,   // 烽火台报警码前三位
  ReceiverMainCar_AlarmBackCode = 0x11,    // 烽火台报警码前三位
  ReceiverMainCar_RoadFrontCode = 0xD6,    // 道闸前三位数据
  ReceiverMainCar_RoadBackCode = 0xD7,     // 道闸后三位数据
  ReceiverMainCar_StreetLight = 0xEF,      // 交通信号
  ReceiverMainCar_AGV_TrafficSign = 0xE4,  // 接收交通标志
  ReceiverMainCar_YearMoneyDay = 0x37,     // 语音播报年月日
  ReceiverMainCar_HourMinuteSecond = 0x38, // 语音播报时分秒
  ReceiverMainCar_WetherTemp = 0x39,       // 语音天气温度
  ReceiverMainCar_StereoDistance = 0x70,   // 立体显示距离(旋转LED)
  ReceiverMainCar_AGV_DataComplete = 0xE2, // 接收主车
  AGV_CMD_Data1 = 0xEA,                    // 预留数据
  AGV_CMD_Data2 = 0xEB,                    // 预留数据
  AGV_CMD_Data3 = 0xEC,                    // 预留数据
  AGV_CMD_Data4 = 0xED,                    // 预留数据
  AGV_CMD_Data5 = 0xEE,                    // 预留数据
} Recieve_ToSalvecar;                      // 接收数据
typedef struct
{
  uint16_t car_step; // 任务
  uint16_t car_x;    // 路径坐标x-'A'
  uint16_t car_y;    // 路径坐标'7'-Y
} car_state;         // 路径结构体
typedef enum
{
  TrackMode_NORMAL = 0,          // 红外+矫正循迹
  TrackMode_ENCODER,             // 码盘+矫正循迹
  TrackMode_BACK_TO_CROSS,       // 退回黑线
  TrackMode_ENCODER_SpecialRoad, // 过特殊地形
} TrackMode_t;                   // 结构体

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 数据定义 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
// 外部数据
uint8_t ZigBee_command[8];      // 读取Zigbee传输数据
uint8_t ZigBee_judge;           // 判断校验位
uint8_t Define_Zero48[48];      // 发送给主车数据
uint8_t Light_Level[2];         // 路灯档位(初始 设置)
uint8_t Ultrasonic_Distance[3]; // 超声波距离(百 十 个(cm))
uint8_t Beacon_Alarm[6];        // 烽火台开启
uint8_t Init_AGarage_Floor;     // A车库当前层数
uint8_t Init_BGarage_Floor;     // B车库当前层数
uint8_t QR_Length6[6];          // 二维码数据
uint8_t color_reult[2];         // 二维码颜色
uint8_t WIFI_start[3];          // 无线充电桩开启码
uint8_t recieve_alarm[6];       // 蜂鸣器报警接收
uint8_t Weather_Temp[2];        // 语音播报 天气和气温
uint8_t Rtc_Data[6];            // RTC时间
uint8_t Distance_Display = 0;   // 显示距离信息
uint8_t FirstPoint[2] = {0};    // 显示初始点坐标信息
uint8_t quit_wait = 0;          // 退出等待标志位
uint8_t Road_Code[6];           // 车牌
uint8_t data1 = 0;              // 预留数据位1
uint8_t data2 = 0;              // 预留数据位2
uint8_t data3 = 0;              // 预留数据位3
uint8_t data4 = 0;              // 预留数据位4
uint8_t data5 = 0;              // 预留数据位5

// 功能标识位
uint8_t send_move_flag = 0;      // 发送主车命令标志位
uint8_t move_flag = 0;           // 发送主车命令标志位
uint8_t traffic_sign = 0;        // 发送主车命令标志位
uint8_t data_complete = 0;       // 主车数据接受完成
uint8_t Find_SpeicalRoad = true; // 寻找特殊地形
uint8_t find_space = 0;          // 寻找特殊地形标志位
uint8_t track_mode = 0;          // 视觉循迹与红外循迹切换，一般写在按键里，0为红外，1为视觉
uint8_t cross_spacefloor = 0;    // 寻找完特殊形式,继续循迹

// 小车路径方向
Direction_t car_dir;              // 小车方向
car_state Car_FristRoute[MAX];    // 存储小车第一段路径
uint8_t Recode_FristRoute = 0;    // 第一段路径
uint8_t CarFristRouteNumber = 0;  // 第一段路径数目
car_state Car_SecondRoute[MAX];   // 存储小车第二段路径
uint8_t Recode_SecondRoute = 0;   // 第二段路径数目
uint8_t CarSecondRouteNumber = 0; // 第二段路径数目
uint8_t AGV_Forward;              // 小车初始化方向

// 摄像头储存数据
uint8_t QRcode_Array[10][100];  // 存储二维码数据
uint8_t QRcode_Length[10];      // 存储二维码长度
uint8_t QR_Number = 0;          // 记录二维码识别下标
uint8_t QRcode_OTABufData[100]; // 接收每次二维码信息

// 内部数据
uint8_t SameNum_Flag = 0;       // 判断需要识别的个数和真正识别到个数是否相同
uint8_t SameColor_Flag = 0;     // 判断需要颜色与返回的是否一样
uint8_t road_open_flag = 0;     // 道闸(开启标志位)
uint8_t garageA_quit_move = 0;  // 驶出车库A
uint8_t garageA_move_front = 0; // 驶入车库A
uint8_t garageB_quit_move = 0;  // 驶出车库B
uint8_t garageB_move_front = 0; // 驶入车库B
uint8_t traffic_status = 0;     // K210交通灯颜色返回
char flag_AB;                   // 交通灯AB标志位
uint8_t Data_OpenMVBuf[8];

// 内部指令k210
uint8_t qr_disc_buf[8] = {0x55, 0x02, 0x92, 0x01, 0x00, 0x00, 0x00, 0xBB};       // 给K210发送识别二维码
uint8_t traffical_StartUp[8] = {0x55, 0x02, 0x91, 0x04, 0x00, 0x00, 0x00, 0xBB}; // 给K210发送识别二维码/交通灯

// 内部指令主车和标志物
uint8_t Send_AGV[8] = {0X55, 0X01, 0X00, 0X00, 0X00, 0X00, 0X00, 0XBB};
uint8_t road_buf[] = {0x55, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t led_data_buf[8] = {0x55, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t led_time_buf[8] = {0x55, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB}; // LED显示
uint8_t led_distance_buf[8] = {0x55, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t stereogarage_buf[8] = {0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB};       // 立体车库（A/B）
uint8_t stereosarage_quire_buf[8] = {0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB}; // 立体车库（A/B）// 语音播报使用，请勿删除，此处现在
uint8_t Number_Voice[] = {0xC1, 0xE3, 0xD2, 0xBB, 0xB6, 0xFE,
                          0xC8, 0xFD, 0xCB, 0xC4, // 零一二三四五六七八九十
                          0xCE, 0xE5, 0xC1, 0xF9, 0xC6, 0xDF,
                          0xB0, 0xCB, 0xBE, 0xC5, 0xCA, 0xAE};
uint8_t Weather_Code[][4] = {{0xB4, 0xF3, 0xB7, 0xE7}, {0xB6, 0xE0, 0xD4, 0xC6}, {0xC7, 0xE7, 0X01, 0X01}, // 大风 多云 晴
                             {0xD0, 0xA1, 0xD1, 0xA9},
                             {0xD0, 0xA1, 0xD3, 0xEA},
                             {0xD2, 0xF5, 0xCC, 0xEC}}; // 小雪 小雨 阴天
uint8_t voice_buf[8] = {0x55, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t voice_weather_buf[8] = {0x55, 0x06, 0x42, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t voice_check_buf[8] = {0x55, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t test_buf[] = {
    0xFD, 0x00, 0x0C, 0x01, 0x01, 0xC0, 0xEE, 0xB3,
    0xBF, 0xD1, 0xEE, 0xCE, 0xDE, 0xB5, 0xD0}; // 需要合成的文本更改

uint8_t year_voice[] = {0xFD, 0x00, 0x14, 0x01, 0x01, 0xB5, 0xB1,
                        0xC7, 0xB0, 0xCA, 0xB1, 0xBC, 0xE4, // 当前时间
                        0xB6, 0xFE, 0xC1, 0xE3, 0xB6, 0xFE, 0xC8,
                        0xFD, 0xC4, 0xEA}; // 二零二三年  17 18 19 20
uint8_t date_voice[] = {
    0xFD, 0x00, 0x10, 0x01, 0x01, 0xCA, 0xAE, 0xB6,
    0xFE, 0xD4, 0xC2,                                // 十二月 5 6 7 8
    0xC8, 0xFD, 0xCA, 0xAE, 0xD2, 0xBB, 0xC8, 0xD5}; // 三十一日11 12 13 14 15 16
uint8_t time_voice[] = {
    0xFD, 0x00, 0x1A, 0x01, 0x01, 0xB6, 0xFE, 0xCA, 0xAE,
    0xCB, 0xC4, 0xCA, 0xB1,                          // 二十三时  5  6  7  8  9 10
    0xC8, 0xFD, 0xCA, 0xAE, 0xC8, 0xFD, 0xB7, 0xD6,  // 三十三分  13 14 15 16 17 18
    0xC8, 0xFD, 0xCA, 0xAE, 0xC1, 0xF9, 0xC3, 0xEB}; // 三十三秒  21 22 23 24 25 26
uint8_t beacon_sendquire[8] = {0x55, 0x07, 0x09, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t beacon_bendcode_buf[8] = {0x55, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t tft_picture_buf[] = {0x55, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t tft_licenseplate_buf[] = {0x55, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t tft_time_buf[] = {0x55, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t tft_date_buf[] = {0x55, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t tft_distance_buf[] = {0x55, 0x00, 0x50, 0x00, 0x00, 0x00, 0x00, 0xbb};
uint8_t tft_traffic_buf[8] = {0x55, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0xbb};
uint8_t wireless_buf[8] = {0x55, 0x0a, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t wireless_buf_5_2[8] = {0x55, 0x0A, 0X00, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t etc_buf[8] = {0X55, 0x0C, 0x08, 0x00, 0x00, 0x00, 0X00, 0XBB};
uint8_t traffic_buf[] = {0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t traffic_result_buf[] = {0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t special_terrain[] = {0x55, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t infare_rotationled_buf[6] = {0xFF, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t rotationled_zigbee_buf[8] = {0x55, 0x11, 0x31, 0x00, 0x00, 0x00, 0x00, 0xBB};
uint8_t trm_buf[] = {0xAF, 0x06, 0x00, 0x02, 0x00, 0x00, 0x01, 0xBB};
uint8_t whether_voice[] = {
    0xFD, 0x00, 0x1A, 0x01, 0x01, 0xB5, 0xB1,
    0xC7, 0xB0, 0xCC, 0xEC, 0xC6, 0xF8,  // 当前天气
    0xB4, 0xF3, 0xB7, 0xE7,              // 大风  13 14 15 16
    0xB6, 0xFE, 0xCA, 0xAE, 0xBE, 0xC5,  // 二十五  17 18 19 20 21 22
    0xC9, 0xE3, 0xCA, 0xCF, 0xB6, 0xC8}; // 摄氏度

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ arduino函数 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:setup
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:启动函数,初始化固件
*************************************************************************************************************/
void setup()
{
  CoreLED.Initialization();          // 初始化核心板LED
  CoreKEY.Initialization();          // 初始化核心板按键
  CoreBeep.Initialization();         // 初始化核心板蜂鸣器
  ExtSRAMInterface.Initialization(); // 外部总线初始化
  LED.Initialization();              // 初始化任务板子LED
  BH1750.Initialization();           // 初始化关照传感器
  BEEP.Initialization();             // 初始化任务板蜂鸣器
  Infrare.Initialization();          // 红外传感器初始化
  Ultrasonic.Initialization();       // 超声波初始化
  DCMotor.Initialization();          // 直流电机初始化
  BKRC_Voice.Initialization();       // 小创语音识别初始化
  Serial.begin(115200);              // 初始化串口
  while (!Serial)
    ; // 等待串口初始化完成
  car_dir = AGV_LEFT;
}
/************************************************************************************************************
 【函 数】:loop
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:循环函数
*************************************************************************************************************/
void loop()
{
  // 按键检测
  CoreKEY.Kwhile(KEY_Handler);
  // Zigbee接收
  Zigbee_Recive();
  // 小车行径
  if (move_flag != 0)
  {
    // 路径非空跑主车发送路径
    if (CarFristRouteNumber != 0)
    {
      // 1是这第一段路径非空是指，从车第一段路径为主车获取到
      Car_Move_FirstRoute(Car_FristRoute);
    }
    // 空则跑自己定义的路径(在路径已知的情况,填写自己路径即可)
    else
    {
      // 2非空，则是按自己的路线走，然后car_dir得定义一下，按照你小车放的方向
      Car_FirstRoute_Config(car_dir, "G4F4F6D6B6B7");
    }
    move_flag = 0;
  }
}
/************************************************************************************************************
 【函 数】:KEY_Handler
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:按键定义
*************************************************************************************************************/
void KEY_Handler(uint8_t k_value)
{
  switch (k_value)
  {
  case 1:
    move_flag = 1;
    Serial.println("move");
    break;
  case 2:
    DCMotor.Go(30, 980); // 前进980码，直接冲过特殊地形
    break;
  case 3:
    Normal_Track(30, TrackMode_NORMAL, 0);
    break;
  case 4:
    adjust_car2(3, 20, 0);
    break;
  default:
    break;
  }
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 通讯函数 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:Zigbee_Recive
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:接收zigbee函数
*************************************************************************************************************/
void Zigbee_Recive(void)
{
  // 从车接收ZigBee数据
  if (ExtSRAMInterface.ExMem_Read(0x6100) != 0x00)
  {
    // 读取Zigbee当前数据
    ExtSRAMInterface.ExMem_Read_Bytes(ZigBee_command, 8);
    // 获取校验和
    ZigBee_judge = ZigBee_command[6];
    // 计算校验和
    Command.Judgment(ZigBee_command);
    // 打印zigbee数据
    Content_Print_HEX(ZigBee_command, 8);
    // Zigbee信息处理
    if (ZigBee_command[0] == 0x55 && ZigBee_command[7] == 0xBB && (ZigBee_command[6] == ZigBee_judge))
    {
      if (ZigBee_command[1] == 0x02)
      {
        // 主车Zigbee信息处理
        MainCar_Information();
      }
      else
      {
        // 标志物返回数据处理
        ZigBeeRx_Handler(ZigBee_command);
      }
    }
  }
}
/************************************************************************************************************
 【函 数】:MainCar_Information
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:处理主车Zigbee指令 (与主车protocol.h中的AGV_CMD_t对应,按题意添加)
*************************************************************************************************************/
void MainCar_Information(void)
{
  switch (ZigBee_command[2])
  {
  case ReceiverMainCar_AlarmFrontCode: // 烽火台开启码前三位
    Beacon_Alarm[0] = ZigBee_command[3];
    Beacon_Alarm[1] = ZigBee_command[4];
    Beacon_Alarm[2] = ZigBee_command[5];
    break;
  case ReceiverMainCar_AlarmBackCode: // 烽火台开启码后三位
    Beacon_Alarm[3] = ZigBee_command[3];
    Beacon_Alarm[4] = ZigBee_command[4];
    Beacon_Alarm[5] = ZigBee_command[5];
    break;
  case ReceiverMainCar_YearMoneyDay: // RTC 年月日
    Rtc_Data[0] = ZigBee_command[3];
    Rtc_Data[1] = ZigBee_command[4];
    Rtc_Data[2] = ZigBee_command[5];
    break;
  case ReceiverMainCar_HourMinuteSecond: // RTC 时分秒
    Rtc_Data[3] = ZigBee_command[3];
    Rtc_Data[4] = ZigBee_command[4];
    Rtc_Data[5] = ZigBee_command[5];
    break;
  case ReceiverMainCar_WetherTemp: // 天气  温度
    Weather_Temp[0] = ZigBee_command[3];
    Weather_Temp[1] = ZigBee_command[4];
    break;
  case ReceiverMainCar_AGV_Forward: // 小车初始化朝向
    AGV_Forward = ZigBee_command[3];
    break;
  case ReceiverMainCar_AGV_Quitwait: // 从车等待在起信号
    quit_wait = 1;
    break;
  case ReceiverMainCar_AGV_Movement: // 主车启动跑路径
    move_flag = 1;
    break;
  case ReceiverMainCar_RoadFrontCode: // 道闸车牌前三位
    Road_Code[0] = ZigBee_command[3];
    Road_Code[1] = ZigBee_command[4];
    Road_Code[2] = ZigBee_command[5];
    break;
  case ReceiverMainCar_RoadBackCode: // 道闸车牌后三位
    Road_Code[3] = ZigBee_command[3];
    Road_Code[4] = ZigBee_command[4];
    Road_Code[5] = ZigBee_command[5];
    break;
  case ReceiverMainCar_AGV_Route: // 第一段路径
    if ((CarFristRouteNumber == 0 || Car_FristRoute[CarFristRouteNumber - 1].car_step != ZigBee_command[3]) && move_flag == 0)
    {
      Car_FristRoute[CarFristRouteNumber].car_step = ZigBee_command[3];
      Car_FristRoute[CarFristRouteNumber].car_x = ZigBee_command[4];
      Car_FristRoute[CarFristRouteNumber].car_y = ZigBee_command[5];
      CarFristRouteNumber++;
    }
    break;
  case ReceiverMainCar_AGV_NextRoute: // 第二段路径
    if ((CarSecondRouteNumber == 0 || Car_SecondRoute[CarSecondRouteNumber - 1].car_step != ZigBee_command[3]))
    {
      Car_SecondRoute[CarSecondRouteNumber].car_step = ZigBee_command[3]; //
      Car_SecondRoute[CarSecondRouteNumber].car_x = ZigBee_command[4];
      Car_SecondRoute[CarSecondRouteNumber].car_y = ZigBee_command[5];
      CarSecondRouteNumber++;
    }
    break;
  case ReceiverMainCar_AGV_TrafficSign: // 交通标志
    traffic_sign = ZigBee_command[3];
    break;
  case ReceiverMainCar_AGV_DataComplete: // 数据传输完成
    data_complete = 1;
    break;
  case AGV_CMD_Data1: // 预留数据
    break;
  }
}
/************************************************************************************************************
 【函 数】:ZigBeeRx_Handler
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:处理标志物Zigbee指令 (没写枚举 因为懒得改了)
*************************************************************************************************************/
void ZigBeeRx_Handler(uint8_t *mar)
{
  switch (mar[1])
  {
  case 0x02:
    break;
  case 0x03:
    if (mar[2] == 0x01) // 道闸标志物
    {
      if (mar[5] == 0x05) // 道闸开启
        road_open_flag = 1;
    }
    break;
  case 0x04: // LED显示标志物（暂无返回）
    break;
  case 0x05:
    switch (mar[3]) // 立体车库标志物B
    {
    case 0x01:
      Init_BGarage_Floor = mar[4];
      Serial.print("Init_BGarage_Floor");
      Serial.println(Init_BGarage_Floor);
      break;
    case 0x02:
      if (mar[4] == 0x02)
      {
        garageB_move_front = 1; // 前侧被触发,向后退
      }
      else if (mar[5] == 0x02)
      {
        garageB_move_front = 2; // 后侧被触发,向前进
        garageB_quit_move = 1;
      }
      else
      {
        if (garageB_quit_move == 0)
        {
          garageB_move_front = 1; // 前后都未触发向后退
        }
        else
        {
          garageB_quit_move = 2; // 直到后侧被触发,并且回到都为触发状态
        }
      }
      Serial.print("garageB_quit_move:");
      Serial.print(garageB_quit_move);
      Serial.print("garageB_move_front:");
      Serial.println(garageB_move_front);
      break;
    default:
      break;
    }
    break;
  case 0x06:
    switch (mar[2]) // 语音播报标志物
    {
    case 1:
      break;
    case 2:
      Rtc_Data[0] = mar[3]; // 返回RTC日期
      Rtc_Data[1] = mar[4];
      Rtc_Data[2] = mar[5];
      break;
    case 3:
      Rtc_Data[3] = mar[3]; // 返回RTC时间
      Rtc_Data[4] = mar[4];
      Rtc_Data[5] = mar[5];
      break;
    case 4:
      Weather_Temp[0] = mar[3]; // 返回温度和天气
      Weather_Temp[1] = mar[4];
      break;
    default:
      break;
    }
    break;
  case 0x07: // 红外报警标志物
    break;
  case 0x08: // TFT显示标志物B
    break;
  case 0x09: // 调光标志物
    break;
  case 0x0A: // 磁悬浮无线充电标志物
    break;
  case 0x0B: // TFT显示标志物A
    break;
  case 0x0C: // ETC系统标志物
    break;
  case 0x0D:        // 立体车库标志物A
    switch (mar[3]) // 立体车库标志物B
    {
    case 0x01:
      Init_AGarage_Floor = mar[4];
      Serial.print("Init_AGarage_Floor");
      Serial.println(Init_AGarage_Floor);
      break;
    case 0x02: // 挡住是0x02(未被触发),没挡住是0x01(被触发) 第4位 前，第5位 后
      if (mar[4] == 0x02)
      {
        garageA_move_front = 1; // 前侧被触发,向后退
      }
      else if (mar[5] == 0x02)
      {
        garageA_move_front = 2; // 后侧被触发,向前进
        garageA_quit_move = 1;
      }
      else
      {
        if (garageA_quit_move == 0)
        {
          garageA_move_front = 1; // 前后都未触发向后退
        }
        else
        {
          garageA_quit_move = 2; // 直到后侧被触发,并且回到都为触发状态
        }
      }
      break;
    default:
      break;
    }
    break;
  case 0x0E: // 交通灯标注物A
    break;
  case 0x0F: // 交通灯标志物B
    break;
  case 0x10: // 特殊地形
    break;
  case 0x11: // 智能立体显示标志物
    break;
  case 0x12: // TFT显示C
    break;
  case 0x13: // 交通灯标志物C
    break;
  case 0x14: // 交通灯标志物D
    break;
  }
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 从车任务 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
void Task_Detal(int x, int y)
{
  switch (x)
  {
  case 1:
    switch (y)
    {
    case 0:
      Task_B7();
      break; //"B7"
    case 1:
      Task_B6();
      break; //"B6"
    case 2:
      Task_B5();
      break; //"B5"
    case 3:
      Task_B4();
      break; //"B4"
    case 4:
      Task_B3();
      break; //"B3"
    case 5:
      Task_B2();
      break; //"B2"
    case 6:
      Task_B1();
      break; //"B1"
    default:
      break;
    }
    break;
  case 2:
    switch (y)
    {
    case 1:
      Task_C6();
      break; //"C6"
    case 3:
      Task_C4();
      break; //"C4"
    case 5:
      Task_C2();
      break; //"C2"
    default:
      break;
    }
    break;
  case 3:
    switch (y)
    {
    case 0:
      Task_D7();
      break; //"D7"
    case 1:
      Task_D6();
      break; //"D6"
    case 2:
      Task_D5();
      break; //"D5"
    case 3:
      Task_D4();
      break; //"D4"
    case 4:
      Task_D3();
      break; //"D3"
    case 5:
      Task_D2();
      break; //"D2"
    case 6:
      Task_D1();
      break; //"D1"
    default:
      break;
    }
    break;
  case 4:
    switch (y)
    {
    case 1:
      Task_E6();
      break; //"E6"
    case 3:
      Task_E4();
      break; //"E4"
    case 5:
      Task_E2();
      break; //"E2"
    default:
      break;
    }
    break;
  case 5:
    switch (y)
    {
    case 0:
      Task_F7();
      break; //"F7"
    case 1:
      Task_F6();
      break; //"F6"
    case 2:
      Task_F5();
      break; //"F5"
    case 3:
      Task_F4();
      break; //"F4"
    case 4:
      Task_F3();
      break; //"F3"
    case 5:
      Task_F2();
      break; //"F2"
    case 6:
      Task_F1();
      break; //"F1"
    default:
      break;
    }
    break;
  case 6:
    switch (y)
    {
    case 1:
      Task_G6();
      break; //"G6"
    case 3:
      Task_G4();
      break; //"G4"
    case 5:
      Task_G2();
      break; //"G2"
    }
  default:
    break;
  }
}
void Task_B7(void) {}
void Task_B6(void)
{
  // Turn_Conner_TypeDelay(90, -90, 45);
  // delay(1000);
  // Beacon_Infrare_Alarm(Beacon_Alarm);
  // delay(1000);
  // Turn_Conner_TypeDelay(-90, 90, 45);
  // delay(1000);
}
void Task_B5(void) {}
void Task_B4(void) {}
void Task_B3(void) {}
void Task_B2(void) {}
void Task_B1(void) {}
void Task_C6(void) {}
void Task_C4(void) {}
void Task_C2(void) {}
void Task_D7(void) {}
void Task_D6(void)
{
  // Turn_Conner_TypeDelay(90, -90, 45);
  // delay(300);
  // RotationLED_Traffic(traffic_sign);
  // delay(300);
  // Turn_Conner_TypeDelay(-90, 90, 45);
  // delay(300);

  // RoadGate_Plate(Road_Code);
}
void Task_D5(void) {}
void Task_D4(void) {}
void Task_D3(void) {}
void Task_D2(void) {}
void Task_D1(void) {}
void Task_E6(void) {}
void Task_E4(void) {}
void Task_E2(void) {}
void Task_F7(void) {}
void Task_F6(void) {}
void Task_F5(void) {}
void Task_F4(void)
{
  K210_Scan_ModeTwo(2);

  // delay(300); // TODO
  // Turn_Conner_TypeDelay(-90, 90, 90);
  // delay(300);
  // car_dir = AGV_DOWN;

  // adjust_car(2, 20, 2);
  // delay(300);

  // DCMotor.Back(30, 20);
  // delay(300);
  // K210_Traffic_Disc('B');
  // delay(300);
  // DCMotor.Go(30, 20);
}
void Task_F3(void) {}
void Task_F2(void) {}
void Task_F1(void) {}
void Task_G2(void) {}
void Task_G4(void) {}
void Task_G6(void) {}
// 第一条路径起始点任务
void RouteOne_FirstTask(void)
{
  // 摄像头低头
  K210_Servo_Control(-45);
  delay(500);
}
// 第一条路径终点任务
void RouteOne_LastTask(void)
{
  Car_Reverse_Paring(2, 1, 0);
  DCMotor.Back(20);
}
// 第二条路径起始点任务
void RouteTwo_FirstTask(void) {}
// 第二条路径终点任务
void RouteTwo_LastTask(void) {}
// 等待主车发送启动命令，结束当期避让
void Wait_MainCar(void)
{
  while (!quit_wait)
  {
    // zigbee数据读取
    Zigbee_Recive();
    // 接收到B0时候,退出循环
    if (quit_wait == 1)
      break;
  }
  quit_wait = 0;
}
// 第二段路径
void Car_Move_SecondRoute(car_state *car)
{
  int x, y;

  RouteTwo_FirstTask();
  for (Recode_FristRoute; Recode_FristRoute < (CarFristRouteNumber - 2);
       Recode_FristRoute++) // 自动行径至虚压轴点(包含)
  {
    // 确定目标方向
    x = car[Recode_FristRoute + 1].car_x - car[Recode_FristRoute].car_x;
    y = car[Recode_FristRoute + 1].car_y - car[Recode_FristRoute].car_y;
    // 打印坐标点
    Serial.print("x:");
    Serial.print(car[Recode_FristRoute].car_x);
    Serial.print("y:");
    Serial.println(car[Recode_FristRoute].car_y);
    if (!(y == 0 && x == 0))
    {
      // 任务(多次经过同一点,可以尝试Task_B7写法,所有任务都得看方向调整,且调整后根据路径自行调整回来或者改变方向)
      Task_Detal(car[Recode_FristRoute].car_x, car[Recode_FristRoute].car_y);
      // 方向
      Auto_Forward(x, y);
      // 循迹
      CarTrack(50, track_mode);
    }
  }
  // 最后一个点任务即倒车入库前一个点
  RouteTwo_LastTask();
}
// 第二段路径初始化
void Car_Second_Config(uint8_t foword, char *cmd)
{
  uint8_t length = strlen(cmd) / 2; // 计算路径个数
  uint8_t i = 0;
  CarFristRouteNumber = 0;
  for (i = 0; i < length; i++) // 将行驶坐标存入数组 Car_FristRoute 中
  {
    CarFristRouteNumber++;
    if (cmd[2 * i] == 0 && cmd[2 * i + 1] == 0)
      break;
    Car_FristRoute[i].car_step = i;
    Car_FristRoute[i].car_x = cmd[2 * i] - 'A';     // 坐标x
    Car_FristRoute[i].car_y = '7' - cmd[2 * i + 1]; // 坐标y
  }
  // 补充最后点坐标
  CarFristRouteNumber++;
  Car_FristRoute[i].car_step = i;
  Car_FristRoute[i].car_x = 'D' - 'A'; // 坐标x
  Car_FristRoute[i].car_y = '7' - '7'; // 坐标y 此处更改得7为后面得7

  AGV_Forward = foword; // 相对于地图方向的朝向
  Car_Move_SecondRoute(Car_FristRoute);
}
// 第一段路径
void Car_Move_FirstRoute(car_state *car)
{
  int x, y;
  /*------------------------------------车头朝向初始化--------------------------*/
  switch (AGV_Forward)
  {
  case 0x01:
    car_dir = AGV_UP;
    break; // 小车朝上
  case 0x02:
    car_dir = AGV_LEFT;
    break; // 小车朝左
  case 0x03:
    car_dir = AGV_DOWN;
    break; // 小车朝下
  case 0x04:
    car_dir = AGV_RIGHT;
    break; // 小车朝右
  default:
    break;
  }
  // 起始点任务 (用视觉循迹低头45度)
  RouteOne_FirstTask();
  /*-------------------------路径与任务(已屏蔽白卡与特殊地形)--------------------------*/
  for (Recode_FristRoute; Recode_FristRoute < (CarFristRouteNumber - 1); Recode_FristRoute++) // 自动行径至虚压轴点
  {
    // 确定目标方向
    x = car[Recode_FristRoute + 1].car_x - car[Recode_FristRoute].car_x;
    y = car[Recode_FristRoute + 1].car_y - car[Recode_FristRoute].car_y;
    // 打印坐标
    Serial.print("x:");
    Serial.print(car[Recode_FristRoute].car_x);
    Serial.print("y:");
    Serial.println(car[Recode_FristRoute].car_y);
    // 小车自动行径
    if (!(y == 0 && x == 0)) // 如果相邻两个点不是同一个点则执行任务
    {
      // 任务
      // Task_Detal(car[Recode_FristRoute].car_x, car[Recode_FristRoute].car_y);
      // 方向更改
      Auto_Forward(x, y);
      // 循迹
      CarTrack(30, track_mode);
    }
  }
  // 最后点任务(倒车入库)
  RouteOne_LastTask();

  Clear_Route(Car_FristRoute, &Recode_FristRoute,
              &CarFristRouteNumber); // 清除路径
  Clear_Route(Car_SecondRoute, &Recode_SecondRoute,
              &CarSecondRouteNumber); // 清除路径
}
/************************************************************************************************************
【函 数 名】：	Car_FirstRoute_Config 一键路径启动 【参数说明】：
cmd：路径字符串 注/意：起始点坐标需要，终点坐标不需要 foword:小车相对地图方向
               0x01：上 0x02：左 0x03：下 0x04：右 0x05：左上 0x06：左下
               0x07：右上 0x08：右下 【返 回 值】：	无返回 【简    例】
Car_FirstRoute_Config(0x03,"F1F2D2D4D6D7") ;
例如：此路径小车只循迹到D6,要入库至至D7得在 RouteOne_LastTask();函数中添加任务
************************************************************************************************************/
void Car_FirstRoute_Config(uint8_t foword, char *cmd)
{
  uint8_t length = strlen(cmd) / 2;
  // 清空路径和任务点个数
  CarFristRouteNumber = 0;
  Recode_FristRoute = 0;
  for (uint8_t i = 0; i < length; i++) // 将行驶坐标存入数组 Car_FristRoute 中
  {
    CarFristRouteNumber++;
    Car_FristRoute[i].car_step = i;
    Car_FristRoute[i].car_x = cmd[2 * i] - 'A';     // 差值
    Car_FristRoute[i].car_y = '7' - cmd[2 * i + 1]; // 差值
  }

  AGV_Forward = foword; // 相对于地图方向的朝向
  Car_Move_FirstRoute(Car_FristRoute);
}
/************************************************************************************************************
【函 数 名】：	Clear_Route 清空路径，将内容清空 【参数说明】：	car： route:
route_number: 第一条路径：Car_FristRoute                  Recode_FristRoute
CarFristRouteNumber 第二条路径：Car_SecondRoute Recode_SecondRoute
CarSecondRouteNumber 【返 回 值】：	无返回 【简    例】：
Clear_Route(Car_FristRoute, &Recode_FristRoute, &CarFristRouteNumber);
//清除第一条路径 Clear_Route(Car_SecondRoute, &Recode_SecondRoute,
&CarSecondRouteNumber);  //清除第二条路径
************************************************************************************************************/
void Clear_Route(car_state *car, uint8_t *route, uint8_t *route_number)
{
  for (int i = 0; i < (*route_number); i++) // 清空路径
  {
    car[i].car_x = 0;
    car[i].car_y = 0;
    car[i].car_step = 0;
  }
  *route = 0;
  *route_number = 0;
}
/************************************************************************************************************
【函 数 名】：	Copy_Route 复制路径 【参数说明】：	car： second_route:
route_number: Car_FristRoute                  Car_SecondRoute
CarSecondRouteNumber 【返 回 值】：	无返回 【简    例】：
Copy_Route(Car_FristRoute,  Car_SecondRoute, &CarFristRouteNumber);
将第二题路径赋值给第一条路径 此条函数用于两条路径情况，且第二条路径不知；
               例如：已知第一条路径为F1F2D2B2B4;
B4之后得路径由主车白卡获得，车库为D7，假设第二条路径为B4D4D6,
               Car_FirstRoute_Config(AGV_DWON,"F1F2D2B2B4B6"),最后一个B6点为补点，只是为了让车循迹至B4,
               然后执行第二条路径时候得，先用 Clear_Route(Car_FristRoute,
&Recode_FristRoute, &CarFristRouteNumber);清除第一条路径 紧接着用
Copy_Route(Car_FristRoute,  Car_SecondRoute, &CarFristRouteNumber);
并且同理在需要补上D7这个点， 如何补请看 Car_Move_FirstRoute函数中的示例
************************************************************************************************************/
void Copy_Route(car_state *car, car_state *second_route, uint8_t *route_number)
{
  Clear_Route(Car_FristRoute, &Recode_FristRoute,
              &CarFristRouteNumber); // 清除路径
  for (int i = 0; i < (*route_number); i++)
  {
    car[i].car_x = second_route[i].car_x;
    car[i].car_y = second_route[i].car_y;
    car[i].car_step = second_route[i].car_step;
  }
  CarFristRouteNumber = *route_number;
  Recode_FristRoute = 0;
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 转弯函数 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:Turn_Conner_TypeDelay
 【参 数】:leftspeed 左轮速度  rightspeed 右轮速度  angle 旋转角度
 【返 回】:
 【简 例】:Turn_Conner_TypeDelay(90,-90,180); 小车右转180°
 【说 明】:通过延迟来转弯(需要更改TurnAngle的值,TurnAngle转1°需要的时间)
*************************************************************************************************************/
void Turn_Conner_TypeDelay(int16_t leftspeed, int16_t rightspeed, uint16_t angle)
{
  delay(200);
  DCMotor.SpeedCtr(leftspeed, rightspeed);
  delay((int)(TurnAngle * angle));
  DCMotor.Stop();
}
/************************************************************************************************************
 【函 数】:TurnRight
 【参 数】:speed 车轮速度
 【返 回】:
 【简 例】:TurnRight(90,0);
 【说 明】:通过码盘来右转90°
*************************************************************************************************************/
void TurnRight(uint8_t speed)
{
  Clear_Code();
  DCMotor.TurnRight_Code(speed, Turn_Code);
  delay(500);
}
/************************************************************************************************************
 【函 数】:TurnLeft
 【参 数】:speed 车轮速度
 【返 回】:
 【简 例】:TurnRight(90,0);
 【说 明】:通过码盘来左转90°
*************************************************************************************************************/
void TurnLeft(uint8_t speed)
{
  Clear_Code();
  DCMotor.TurnLeft_Code(speed, Turn_Code);
  delay(500);
}
/************************************************************************************************************
 【函 数】:TurnRight
 【参 数】:speed 车轮速度
 【返 回】:
 【简 例】:TurnBack(90);
 【说 明】:通过码盘来转180°
*************************************************************************************************************/
void TurnBack(uint8_t speed)
{
  Clear_Code();
  TurnLeft(speed);
  TurnLeft(speed);
}
/************************************************************************************************************
 【函 数】:Auto_Forward
 【参 数】:x轴 y轴
 【返 回】:
 【简 例】:x=0,y=1  转到上方
 【说 明】:通过xy轴转向
*************************************************************************************************************/
void Auto_Forward(int x, int y)
{
  uint8_t car_direction_front;

  if (x == 0)
  {
    if (y > 0)
      car_direction_front = 1; // 朝上
    else
      car_direction_front = 3; // 朝下
  }
  else
  {
    if (x > 0)
      car_direction_front = 4; // 朝右
    else
      car_direction_front = 2; // 朝左
  }
  switch (car_dir) // 上左下右
  {
  case 1:
    switch (car_direction_front) // 上
    {
    case 1:
      car_dir = AGV_UP;
      break; // 上
    case 2:
      TurnLeft(TurnCarSpeed);
      car_dir = AGV_LEFT;
      break; // 左
    case 3:
      TurnBack(TurnCarSpeed);
      car_dir = AGV_DOWN;
      break; // 下
    case 4:
      TurnRight(TurnCarSpeed);
      car_dir = AGV_RIGHT;
      break; // 右
    default:
      break;
    }
    break;
  case 2:
    switch (car_direction_front) // 左
    {
    case 1:
      TurnRight(TurnCarSpeed);
      car_dir = AGV_UP;
      break; // 上
    case 2:
      car_dir = AGV_LEFT;
      break; // 左
    case 3:
      TurnLeft(TurnCarSpeed);
      car_dir = AGV_DOWN;
      break; // 下
    case 4:
      TurnBack(TurnCarSpeed);
      car_dir = AGV_RIGHT;
      break; // 右
    default:
      break;
    }
    break;
  case 3:
    switch (car_direction_front) // 下
    {
    case 1:
      TurnBack(TurnCarSpeed);
      car_dir = AGV_UP;
      break; // 上
    case 2:
      TurnRight(TurnCarSpeed);
      car_dir = AGV_LEFT;
      break; // 左
    case 3:
      car_dir = AGV_DOWN;
      break; // 下
    case 4:
      TurnLeft(TurnCarSpeed);
      car_dir = AGV_RIGHT;
      break; // 右
    default:
      break;
    }
    break;
  case 4:
    switch (car_direction_front) // 右
    {
    case 1:
      TurnLeft(TurnCarSpeed);
      car_dir = AGV_UP;
      break; // 上
    case 2:
      TurnBack(TurnCarSpeed);
      car_dir = AGV_LEFT;
      break; // 左
    case 3:
      TurnRight(TurnCarSpeed);
      car_dir = AGV_DOWN;
      break; // 下
    case 4:
      car_dir = AGV_RIGHT;
      break; // 右
    default:
      break;
    }
    break;
  default:
    break;
  }
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 循迹函数 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:CarTrack
 【参 数】:speed 车轮速度   mode 循迹模式(0红外/1视觉)
 【返 回】:
 【简 例】:CarTrack(30,0);
 【说 明】:循迹主函数
*************************************************************************************************************/
void CarTrack(uint8_t speed, uint8_t mode)
{
  Clear_Code();
  if (mode)
  {
    OpenMV_Track(speed, 0, 0); // 视觉循迹
    delay(500);
    DCMotor.GoSelfDefine(47, 54, 420);
  }
  else
  {
    Normal_Track(speed, 0); // 红外循迹
    delay(500);
    DCMotor.Go(speed, 380);
  }
  delay(500);
}
/************************************************************************************************************
 【函 数】:Normal_Track
 【参 数】:speed 车轮速度   mode 循迹状态   distance 行驶距离(只有mode为TrackMode_ENCODER时 才使用)
 【返 回】:
 【简 例】:CarTrack(30);
 【说 明】:红外循迹主函数
*************************************************************************************************************/
void Normal_Track(uint8_t speed, TrackMode_t mode, uint8_t distance)
{
  Clear_Code();
  bool enter_white = false;
  uint8_t H8_N, Q7_N;
  uint8_t Q7[7], H8[8], ALL_TRACK[15];
  int16_t encode_value = Code_ToLength * distance;
  while (true)
  {
    Q7_N = ExtSRAMInterface.ExMem_Read(TRACK_ADDR + 1);
    H8_N = ExtSRAMInterface.ExMem_Read(TRACK_ADDR);

    for (int8_t i = 6; i >= 0; --i)
    {
      Q7[i] = (Q7_N >> i) & 0x01;
      H8[i] = (H8_N >> i) & 0x01;
      ALL_TRACK[i * 2] = H8[i];
      ALL_TRACK[i * 2 + 1] = Q7[i];
    }
    H8[7] = (H8_N >> 7) & 1;
    ALL_TRACK[14] = H8[7];

    switch (mode)
    {
    case TrackMode_NORMAL:
      if ((H8[0] + Q7[0] + H8[7] + Q7[6]) < 2)
      { // 到十字路口
        Pid.PidData_Clear();
        DCMotor.Stop();
        return;
      }
      else
      { // 正常循迹
        Pid.Calculate_pid(getOffset(ALL_TRACK));
        DCMotor.SpeedCtr(speed + Pid.PID_value, speed - Pid.PID_value);
      }
      break;
    case TrackMode_ENCODER:
      if (Read_Code() <= encode_value)
      {
        Pid.Calculate_pid(getOffset(ALL_TRACK));
        DCMotor.SpeedCtr(speed + Pid.PID_value, speed - Pid.PID_value);
      }
      else
      {
        Pid.PidData_Clear();
        DCMotor.Stop();
        return;
      }
      break;
    case TrackMode_BACK_TO_CROSS:

      break;
    case TrackMode_ENCODER_SpecialRoad:
      if ((H8[0] + Q7[0] + H8[7] + Q7[6]) < 2)
      { // 到十字路口
        Pid.PidData_Clear();
        DCMotor.Stop();
        return;
      }
      else if (Q7[2] && Q7[3] && Q7[4])
      {
        if (!enter_white)
        {
          enter_white = true;
          Pid.PidData_Clear();
          DCMotor.Stop();
        }
        if (Special_Road_Identify())
        { // 特殊地形
          adjust_car2(3, 20, 0);
          DCMotor.Go(speed, 980); // 前进980码，直接冲过特殊地形
        }
        else
        { // 白卡
          Pid.Calculate_pid(getOffset(ALL_TRACK));
          DCMotor.SpeedCtr(speed + Pid.PID_value - 10, speed - Pid.PID_value - 10);
        }
      }
      else if (enter_white && !Q7[3] && Q7[2] && Q7[4])
      {
        Find_SpeicalRoad = true;
        enter_white = false;
      }
      else
      { // 正常循迹
        Pid.Calculate_pid(getOffset(ALL_TRACK));
        DCMotor.SpeedCtr(speed + Pid.PID_value, speed - Pid.PID_value);
      }
      break;
    }
  }
}
/************************************************************************************************************
 【函 数】:Clear_Code
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:清除码盘值
*************************************************************************************************************/
void Clear_Code(void)
{
  uint8_t code_zigbee[8] = {0x55, 0x02, 0x07, 0x00, 0x00, 0x00, 0x00, 0xbb};
  Command.Judgment(code_zigbee); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, code_zigbee, 8);
}
/************************************************************************************************************
 【函 数】:Read_Code
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:读取十六位码盘值
*************************************************************************************************************/
int16_t Read_Code(void)
{
  int16_t Code_Wheel = 0;

  Code_Wheel = ExtSRAMInterface.ExMem_Read(0x6002);                     // 读取码盘低八位
  Code_Wheel = Code_Wheel + (ExtSRAMInterface.ExMem_Read(0x6003) << 8); // 读取码盘高八位

  return Code_Wheel;
}
/************************************************************************************************************
 【函 数】:getOffset
 【参 数】:arr 循迹灯数组
 【返 回】:有循迹灯得到的误差值 用于pid计算
 【简 例】:getOffset([1,1,1,1,1,1,1,0,1,1,1,1,1,1,1]);  返回 0
 【说 明】:循迹灯算法
  1.循迹灯状态             2.将两个数组按以下顺序打为一个数组   3.得到数组并通过算法得值
  Q7 [1,1,1,0,1,1,1]       [2,4,6,8,10,12,14]            [1,1,1,1,1,1,1,0,1,1,1,1,1,1,1] -> 0
  H8[1,1,1,1,1,1,1,1]     [1,3,5,7,9,11,13,15]           [1,1,1,1,1,1,1,0,0,1,1,1,1,1,1] -> 0.5
  简单的说就是通过循迹灯的15位值得到一个值 比如往左偏就是-10 右偏就是+10 正中间就是0  用这个值带入pid去算
*************************************************************************************************************/
float getOffset(uint8_t arr[])
{
  int8_t all_weights[15] = {0};
  for (uint8_t i = 1; i < 14; i++)
  {
    for (uint8_t j = i - 1; j <= i + 1; j++)
    {
      all_weights[i] += arr[j];
    }
  }

  int8_t minimum = 3;
  for (uint8_t i = 1; i < 14; i++)
  {
    if (minimum > all_weights[i])
      minimum = all_weights[i];
  }

  int8_t errorValue1 = 0, errorValue2 = 14;
  for (;;)
  {
    if (all_weights[++errorValue1] == minimum)
      break;
  }
  for (;;)
  {
    if (all_weights[--errorValue2] == minimum)
      break;
  }

  // 平均之后计算误差值
  float errorValue = 7.0 - (errorValue1 + errorValue2) / 2.0;

  return errorValue;
}
/************************************************************************************************************
 【函 数】:Count_BlackNumber
 【参 数】:track_value 循迹值   length  灯数量
 【返 回】:灯灭的数量
 【简 例】:Count_BlackNumber(119, 7); 转二进制得1110111 返回1
 【说 明】:记录循迹灯灭的个数
*************************************************************************************************************/
uint8_t Count_BlackNumber(uint8_t track_value, uint8_t length)
{
  uint8_t res = 0; // 记录循迹灯灭的个数

  for (uint8_t i = 0; i < length; i++)
  {
    if (track_value % 2 == 0)
      res += 1;
    track_value >>= 1;
  }

  return res;
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 倒车函数 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:adjust_car
 【参 数】:count 调整次数   distance 距离   mode 循迹模式
 【返 回】:
 【简 例】:adjust_car(2, 30, 2);
 【说 明】:调整倒车
*************************************************************************************************************/
void adjust_car(uint8_t count, uint8_t distance, uint8_t mode)
{
  for (int i = 0; i < count; i++)
  {
    if (!mode)
    {
      Normal_Track(50, TrackMode_NORMAL, distance);
      delay(1000);
      DCMotor.BackSelfDefine(47, 54, distance * 31);
      delay(1000);
    }
    else
    {
      OpenMV_Track_Distance(50, distance);
      delay(1000);
      DCMotor.BackSelfDefine(47, 54, distance * 23);
      delay(1000);
    }
  }
}
/************************************************************************************************************
 【函 数】:adjust_car2
 【参 数】:count 调整次数   distance 距离   mode 循迹模式
 【返 回】:
 【简 例】:adjust_car2(2, 30, 2);
 【说 明】:倒车调整
*************************************************************************************************************/
void adjust_car2(uint8_t count, uint8_t distance, uint8_t mode)
{
  for (int i = 0; i < count; i++)
  {
    if (mode)
    {
      DCMotor.BackSelfDefine(47, 54, distance * 23);
      delay(1000);
      OpenMV_Track_Distance(30, distance);
      delay(1000);
    }
    else
    {
      DCMotor.Back(30, distance * Code_ToLength);
      delay(1000);
      Normal_Track(30, TrackMode_NORMAL, distance);
      delay(1000);
    }
  }
}
/************************************************************************************************************
 【函 数】:AdjustCar_Back_Parking
 【参 数】:flag 车库标识  Layer 上升层数  count 调整次数   distance 距离   mode 循迹模式
 【返 回】:
 【简 例】:AdjustCar_Back_Parking('A', 2, 2, 30, 2);
 【说 明】:倒车入口
*************************************************************************************************************/
void AdjustCar_Back_Parking(char *flag, uint8_t Layer, uint8_t adjust_count, uint8_t distance, uint8_t mode)
{
  delay(500);
  StereoGarage_WaitingToFloor(flag);
  delay(500);
  Car_Reverse_Paring(adjust_count, distance, mode);
  StereoGarage_BackToHome(flag);
  delay(500);
  StereoGarage_ToLayer(flag, Layer);
  delay(500);
}
/************************************************************************************************************
 【函 数】:Car_Reverse_Paring
 【参 数】:count 调整次数   distance 距离   mode 循迹模式
 【返 回】:
 【简 例】:Car_Reverse_Paring(2, 30, 2);
 【说 明】:倒车入库
*************************************************************************************************************/
void Car_Reverse_Paring(uint8_t count, uint8_t distance, uint8_t mode)
{
  signed char x, y, backhome;

  x = Car_FristRoute[CarFristRouteNumber - 1].car_x - Car_FristRoute[CarFristRouteNumber - 2].car_x; // 小车相对车库的方向
  y = Car_FristRoute[CarFristRouteNumber - 1].car_y - Car_FristRoute[CarFristRouteNumber - 2].car_y;

  if (x == 0)
  {
    if (y > 0)
      backhome = 1; // 朝上
    else
      backhome = 3; // 朝下
  }
  else
  {
    if (x > 0)
      backhome = 4; // 朝右
    else
      backhome = 2; // 朝左
  }
  Serial.print("Backhome:");
  Serial.print(backhome);

  // 倒车入库
  switch (car_dir)
  {
  case 1:
    switch (backhome)
    {
    case 1:
      TurnBack(90);
      break; // 上  ->  上
    case 2:
      TurnRight(90);
      break; // 上  ->  左
    case 3:
      break; // 上  ->  下
    case 4:
      TurnLeft(90);
      break; // 上  ->  右
    default:
      break;
    }
    break;
  case 2:
    switch (backhome)
    {
    case 1:
      TurnLeft(90);
      break; // 左  ->  上
    case 2:
      TurnBack(90);
      break; // 左  ->  左
    case 3:
      TurnRight(90);
      break; // 左  ->  下
    case 4:
      break; // 左  ->  右
    default:
      break;
    }
    break;
  case 3:
    switch (backhome) // 下
    {
    case 1:
      break; // 下  ->  上
    case 2:
      TurnLeft(90);
      break; // 下  ->  左
    case 3:
      TurnBack(90);
      break; // 下  ->  下
    case 4:
      TurnRight(90);
      break; // 下  ->  右
    default:
      break;
    }
    break;
  case 4:
    switch (backhome) // 右
    {
    case 1:
      TurnRight(90);
      break; // 右  ->  上
    case 2:
      break; // 右  ->  左
    case 3:
      TurnLeft(90);
      break; // 右  ->  下
    case 4:
      TurnBack(90);
      break; // 右  ->  右
    default:
      break;
    }
    break;
  default:
    break;
  }
  delay(500);
  adjust_car(count, distance, mode);
  // DCMotor.Back(50, 800);
  DCMotor.BackSelfDefine(40, 40, 800);
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 任务板任务 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:Ultrasonic_Ranging
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:超声波测距 测量数据赋值数组Ultrasonic_Distance[3]中
*************************************************************************************************************/
void Ultrasonic_Ranging(void)
{
  double read_date = 0, sum_date = 0; // 超声波距离
  double max_date = 0, min_date = 0;

  delay(500);
  for (int i = 0; i < 20; i++)
  {
    read_date =
        Ultrasonic.Ranging(Ultrasonic_Flag) * 10; // true代表cm,false代表英寸
    sum_date = sum_date + read_date;
    if (i == 0)
    {
      max_date = read_date; // 取第一次测量作为最大值
      min_date = read_date; // 取第一次测量作为最小值
    }
    else
    {
      if (max_date < read_date)
        max_date = read_date; // 记录最大数据
      if (min_date > read_date)
        min_date = read_date; // 记录最小数据
    }
    delay(100);
  }
  read_date = (sum_date - max_date - min_date) / 18; // 求测量平均
  Serial.println(read_date);

  Ultrasonic_Distance[0] = ((int)read_date / 100) % 10; // 百
  Ultrasonic_Distance[1] = ((int)read_date % 100) / 10; // 十
  Ultrasonic_Distance[2] = (int)read_date % 10;         // 个
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ K210 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:K210_QR_ScanOne
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:发送扫描单个二维码指令
*************************************************************************************************************/
void K210_QR_ScanOne(void)
{
  qr_disc_buf[4] = 0x01; // 开始识别
  qr_disc_buf[5] = 0x00;
  Command.Judgment(qr_disc_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, qr_disc_buf, 8);
}
/************************************************************************************************************
 【函 数】:K210_QR_ScanTwo
 【参 数】:qr_numbber 二维码数量
 【返 回】:
 【简 例】:
 【说 明】:发送扫码规定个数的二维码指令(回传方式为从左到右,从上往下)
*************************************************************************************************************/
void K210_QR_ScanTwo(uint8_t qr_numbber)
{
  qr_disc_buf[4] = 0x02; // 开始识别
  qr_disc_buf[5] = qr_numbber;
  Command.Judgment(qr_disc_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, qr_disc_buf, 8);
}
/************************************************************************************************************
 【函 数】:K210_QR_ScanThree
 【参 数】:color 二维码颜色 0black 1blue 2green 3red 4yellow 5遮挡
 【返 回】:
 【简 例】:
 【说 明】:发送扫码规定颜色的二维码指令
*************************************************************************************************************/
void K210_QR_ScanThree(uint8_t color)
{
  qr_disc_buf[4] = 0x03; // 开始识别
  qr_disc_buf[5] = color;
  Command.Judgment(qr_disc_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, qr_disc_buf, 8);
}
/************************************************************************************************************
 【函 数】:K210_QR_Close
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:发送结束识别指令
*************************************************************************************************************/
void K210_QR_Close(void)
{
  uint8_t qr_close_buf[8] = {0x55, 0x02, 0x92, 0x02, 0x00, 0x00, 0x00, 0xBB}; // 给K210发送识别二维码
  Command.Judgment(qr_close_buf);                                             // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, qr_close_buf, 8);
}
/************************************************************************************************************
 【函 数】:K210_TrackControl
 【参 数】:cmd 循迹开关(1启动/2关闭)
 【返 回】:
 【简 例】:
 【说 明】:发送循迹指令
*************************************************************************************************************/
void K210_TrackControl(uint8_t cmd)
{
  uint8_t track_buf[8] = {0x55, 0x02, 0x91, 0x01, 0x00, 0x00, 0x00, 0xBB};
  if (cmd == 0x01)
    Serial.println("循迹开始");
  else
    Serial.println("循迹关闭");
  track_buf[3] = cmd;
  Command.Judgment(track_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, track_buf, 8);
}
/************************************************************************************************************
 【函 数】:K210_Check_Space
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:发送特殊地形识别指令
*************************************************************************************************************/
void K210_Check_Space()
{
  uint8_t sapcefind_buf[8] = {0x55, 0x02, 0x91, 0x05, 0x00, 0x00, 0x00, 0xBB};
  Command.Judgment(sapcefind_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, sapcefind_buf, 8);
}
/************************************************************************************************************
 【函 数】:K210_Scan_ModeOne
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:识别单个二维码并处理
*************************************************************************************************************/
void K210_Scan_ModeOne(void)
{
  uint8_t location = 0, count = 0; // location:二维码数组下标 count:左右摆动
  uint8_t Data_Length;             // 获取二维码的长度
  uint32_t timecontrol = 0;        // 获取当前Tim时间
  uint8_t Clear_Date[21] = {0};
  int8_t Move_Dsitrance = 0;
  int8_t Quit_Dsitrance = 0;

  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Clear_Date, 21); // 清空缓存区
  K210_Servo_Control(5);                                       // 调整舵机垂直
  delay(500);
  Ultrasonic_Ranging();
  Move_Dsitrance = Ultrasonic_Distance[0] * 10 + Ultrasonic_Distance[1] + Ultrasonic_Distance[2] / 10.0; // 测量距离
  if (Move_Dsitrance < 18)
  {
    DCMotor.Back(50, (int)((18 - Move_Dsitrance) * Code_ToLength));
  }
  else if (Move_Dsitrance > 19)
  {
    DCMotor.Go(50, (int)((Move_Dsitrance - 19) * Code_ToLength));
  }
  delay(200);
  K210_QR_ScanOne(); // 发送开启二维码识别
  Clear_Code();      // 清除码盘值
  timecontrol = millis();
  while (1)
  {
    // 读取Zigbee数据
    Zigbee_Recive();
    // 读取二维码
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00) // 确认是否识别到二维码
    {
      Data_Length = ExtSRAMInterface.ExMem_Read(0x603C); // 读取二维码数据长度
      if (ExtSRAMInterface.ExMem_Read(0x603B) == 0X01)
      {
        ExtSRAMInterface.ExMem_Read_Bytes(0x6038, QRcode_OTABufData, Data_Length + 6); // 读取二维码数据长度
        Content_Print_HEX(&QRcode_OTABufData[0], Data_Length + 6);
      }
      else
      {
        ExtSRAMInterface.ExMem_Read_Bytes(0x6038, QRcode_OTABufData, 8); // 读取二维码识别完成数据
        Content_Print_HEX(&QRcode_OTABufData[0], 8);
      }

      if ((QRcode_OTABufData[0] == 0x55) && (QRcode_OTABufData[1] == 0x02) && (QRcode_OTABufData[2] == 0x92) && (QRcode_OTABufData[3] == 0x01))
      {
        for (int i = 5; i < (Data_Length + 4); i++) // 前面五位判断位，数据区最后一位为二维码长度拼接判断位，即二维码长度超过14判断位
        {
          QRcode_Array[QR_Number][location++] = QRcode_OTABufData[i]; // 读取二维码数据,储存于
          Serial.print(char(QRcode_OTABufData[i]));
          QRcode_OTABufData[i] = 0;
        }
        //+4，是因为最后一位为标志判断位0x88，0x99
        if (QRcode_OTABufData[Data_Length + 4] == 0x88)
        {
          Serial.println();
          QRcode_Length[QR_Number++] = location;
          Serial.print("QRcode_Length:");
          Serial.println(QRcode_Length[QR_Number - 1]);
          location = 0; // 清除二维码下标
        }
      }
      else if ((QRcode_OTABufData[0] == 0x55) && (QRcode_OTABufData[1] == 0x02) && (QRcode_OTABufData[2] == 0x92) && (QRcode_OTABufData[3] == 0X02))
      {
        delay(100);
        K210_QR_Close(); // 关闭二维码识别
        break;
      }
    }

    // 控制小车移动(前后移动，视野变得，以防二维码不在视野)
    if ((millis() - timecontrol) > 800)
    {
      if (count % 2 == 0)
        DCMotor.SpeedCtr(-20, -20);
      else
        DCMotor.SpeedCtr(20, 20);
      count++;
      if (count >= 20)
      {
        //  Serial.print(millis());
        break;
      }
      timecontrol = millis();
    }
  }
  DCMotor.Stop(); // 小车停止
  delay(500);
  K210_Servo_Control(-45); // 视觉循迹低头
  Ultrasonic_Ranging();
  Quit_Dsitrance = Ultrasonic_Distance[0] * 10 + Ultrasonic_Distance[1] + Ultrasonic_Distance[2] / 10.0; // 测量距离

  if (Move_Dsitrance - Quit_Dsitrance >= 0) // 向前进
  {
    DCMotor.Back(50, (int)(Move_Dsitrance - Quit_Dsitrance) * Code_ToLength);
  }
  else if (Move_Dsitrance > 19) // 向后退
  {
    DCMotor.Go(50, (int)(Quit_Dsitrance - Move_Dsitrance) * Code_ToLength);
  }
  delay(200);
}
/************************************************************************************************************
 【函 数】:K210_Scan_ModeTwo
 【参 数】:need_scanf_number 需要扫码二维码的个数 2:先左后右/3:先上，然后左下，右下/4:左上，左下，右上，右下/其他:不按顺序来
 【返 回】:
 【简 例】:
 【说 明】:识别规定数量二维码并处理
*************************************************************************************************************/
void K210_Scan_ModeTwo(uint8_t need_scanf_number)
{
  uint8_t Clear_Date[21] = {0}; // 用于清除缓存区
  int8_t Move_Dsitrance = 0;    // 记录行进距离，用于后面回到原来位置
  uint8_t Code_Number = 0;      // 记录真正识别的二维码个数
  uint8_t location = 0;         // location:二维码数组下标
  uint8_t Data_Length;          // 获取二维码的长度
  uint32_t timecontrol = 0;     // 获取当前Tim时间
  SameNum_Flag = 0;
  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Clear_Date, 21); // 清空缓存区
  K210_Servo_Control(5);                                       // 调整舵机垂直
  delay(500);
  Ultrasonic_Ranging();                                                                                  // 超声波测距
  Move_Dsitrance = Ultrasonic_Distance[0] * 10 + Ultrasonic_Distance[1] + Ultrasonic_Distance[2] / 10.0; // 测量距离
  Serial.print(Move_Dsitrance);
  if (Move_Dsitrance < 21)
  {
    DCMotor.Back(50, (int)((21 - Move_Dsitrance) * Code_ToLength));
  }
  else if (Move_Dsitrance > 22)
  {
    DCMotor.Go(50, (int)((Move_Dsitrance - 22) * Code_ToLength));
  }
  delay(500);
  K210_QR_ScanTwo(need_scanf_number); // 发送二维码模式二启动
  timecontrol = millis();
  while (1)
  {
    // //读取Zigbee数据
    Zigbee_Recive();
    // 读取二维码
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00) // 确认是否识别到二维码
    {
      Data_Length = ExtSRAMInterface.ExMem_Read(0x603C); // 读取二维码数据长度
      if (ExtSRAMInterface.ExMem_Read(0x603B) == 0X01)
      {
        ExtSRAMInterface.ExMem_Read_Bytes(0x6038, QRcode_OTABufData, Data_Length + 6); // 读取二维码数据长度
        Content_Print_HEX(&QRcode_OTABufData[0], Data_Length + 6);
      }
      else
      {
        ExtSRAMInterface.ExMem_Read_Bytes(0x6038, QRcode_OTABufData, 8); // 读取二维码识别完成数据
        Content_Print_HEX(&QRcode_OTABufData[0], 8);
      }

      if ((QRcode_OTABufData[0] == 0x55) && (QRcode_OTABufData[1] == 0x02) && (QRcode_OTABufData[2] == 0x92) && (QRcode_OTABufData[3] == 0x01))
      {
        for (
            int i = 5; i < (Data_Length + 4);
            i++) // 前面五位判断位，数据区最后一位为二维码长度拼接判断位，即二维码长度超过14判断位
        {
          QRcode_Array[QR_Number][location++] = QRcode_OTABufData[i]; // 读取二维码数据,储存于
          Serial.print(char(QRcode_OTABufData[i]));
          QRcode_OTABufData[i] = 0;
        }
        //+4，是因为最后一位为标志判断位0x88，0x99
        if (QRcode_OTABufData[Data_Length + 4] == 0x88)
        {
          Code_Number += 1;
          Serial.println();
          QRcode_Length[QR_Number++] = location;
          Serial.print("QRcode_Length:");
          Serial.println(QRcode_Length[QR_Number - 1]);
          location = 0; // 清除二维码下标
        }
      }
      else
      {
        if (Code_Number == need_scanf_number)
        {
          SameNum_Flag = 1;
          Serial.println("OK");
        }
        else
        {
          Serial.println("Fail");
        }
        delay(100);
        K210_QR_Close(); // 关闭二维码识别
        break;
      }
    }

    if ((millis() - timecontrol) > 16000) // 超时退出识别二维码
    {
      Serial.println("3333");
      break;
    }
  }
  delay(100);
  K210_Servo_Control(-45);
  delay(500);
  if (Move_Dsitrance < 21)
  {
    DCMotor.Go(50, (int)((21 - Move_Dsitrance) * Code_ToLength));
  }
  else if (Move_Dsitrance > 22)
  {
    DCMotor.Back(50, (int)((Move_Dsitrance - 22) * Code_ToLength));
  }
}
/************************************************************************************************************
 【函 数】:K210_Scan_ModeThree
 【参 数】:color 规定颜色 0black 1blue 2green 3red 4yellow 5遮挡
 【返 回】:
 【简 例】:
 【说 明】:识别规定颜色二维码并处理
*************************************************************************************************************/
void K210_Scan_ModeThree(uint8_t color)
{
  uint8_t Clear_Date[21] = {0}; // 用于清除缓存区
  int8_t Move_Dsitrance = 0;    // 记录行进距离，用于后面回到原来位置
  uint8_t location = 0;         // location:二维码数组下标
  uint8_t Data_Length;          // 获取二维码的长度
  uint32_t timecontrol = 0;     // 获取当前Tim时间
  SameColor_Flag = 0;
  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Clear_Date, 21); // 清空缓存区
  K210_Servo_Control(5);                                       // 调整舵机垂直
  delay(500);
  Ultrasonic_Ranging();                                                                                  // 超声波测距
  Move_Dsitrance = Ultrasonic_Distance[0] * 10 + Ultrasonic_Distance[1] + Ultrasonic_Distance[2] / 10.0; // 测量距离
  if (Move_Dsitrance < 18)
  {
    DCMotor.Back(50, (int)((18 - Move_Dsitrance) * Code_ToLength));
  }
  else if (Move_Dsitrance > 19)
  {
    DCMotor.Go(50, (int)((Move_Dsitrance - 19) * Code_ToLength));
  }
  delay(500);
  K210_QR_ScanThree(color); // 发送二维码模式二启动
  timecontrol = millis();
  while (1)
  {
    // //读取Zigbee数据
    // Zigbee_Recive();
    // 读取二维码
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00) // 确认是否识别到二维码
    {
      Data_Length = ExtSRAMInterface.ExMem_Read(0x603C); // 读取二维码数据长度
      if (ExtSRAMInterface.ExMem_Read(0x603B) == 0X01)
      {
        ExtSRAMInterface.ExMem_Read_Bytes(0x6038, QRcode_OTABufData, Data_Length + 6); // 读取二维码数据长度
        Content_Print_HEX(&QRcode_OTABufData[0], Data_Length + 6);
      }
      else
      {
        ExtSRAMInterface.ExMem_Read_Bytes(0x6038, QRcode_OTABufData, 8); // 读取二维码识别完成数据e
        Content_Print_HEX(&QRcode_OTABufData[0], 8);
      }
      if ((QRcode_OTABufData[0] == 0x55) && (QRcode_OTABufData[1] == 0x02) && (QRcode_OTABufData[2] == 0x92) && (QRcode_OTABufData[3] == 0x01))
      {
        for (int i = 5; i < (Data_Length + 4); i++) // 前面五位判断位，数据区最后一位为二维码长度拼接判断位，即二维码长度超过14判断位
        {
          QRcode_Array[QR_Number][location++] = QRcode_OTABufData[i]; // 读取二维码数据,储存于
          Serial.print(char(QRcode_OTABufData[i]));
          QRcode_OTABufData[i] = 0;
        }
        //+4，是因为最后一位为标志判断位0x88，0x99
        if (QRcode_OTABufData[Data_Length + 4] == 0x88)
        {
          SameColor_Flag = 1;
          Serial.println();
          QRcode_Length[QR_Number++] = location;
          Serial.print("QRcode_Length:");
          Serial.println(QRcode_Length[QR_Number - 1]);
          location = 0; // 清除二维码下标
        }
      }
      else
      {
        delay(100);
        K210_QR_Close(); // 关闭二维码识别
        break;
      }
    }

    if ((millis() - timecontrol) > 16000) // 超时退出识别二维码
    {
      break;
    }
  }
  delay(100);
  K210_Servo_Control(-45);
  delay(500);
  if (Move_Dsitrance < 18)
  {
    DCMotor.Go(50, (int)((18 - Move_Dsitrance) * Code_ToLength));
  }
  else if (Move_Dsitrance > 19)
  {
    DCMotor.Back(50, (int)((Move_Dsitrance - 19) * Code_ToLength));
  }
}
/************************************************************************************************************
 【函 数】:K210_Servo_Control
 【参 数】:angle 舵机角度 范围-80至+20，0度垂直于车身 注意：官方说为-80~+40， 但是最高取20
 【返 回】:
 【简 例】:K210_Servo_Control(20);  将舵机调至20°
 【说 明】:控制舵机
*************************************************************************************************************/
void K210_Servo_Control(int8_t angle)
{
  uint8_t servo_buf[8] = {0x55, 0x02, 0x91, 0x03, 0x00, 0x00, 0x00, 0xBB}; // 给OpenMV发送舵机角度
  if (angle >= 0)
  {
    servo_buf[4] = 0x2B;
  }
  else
  {
    servo_buf[4] = 0x2D;
  }
  servo_buf[5] = abs(angle);   // 开始识别
  Command.Judgment(servo_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, servo_buf, 8);
  delay(1000);
}
/************************************************************************************************************
 【函 数】:K210_Traffical_StartUp
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:发送交通灯识别指令 此条协议自定,问题在于k210与arduino通讯只能用0x91和0x92,STM32与Arduino通讯官方写死(估计)
*************************************************************************************************************/
void K210_Traffical_StartUp(void)
{
  Command.Judgment(traffical_StartUp); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, traffical_StartUp, 8);
  delay(200);
}
/************************************************************************************************************
 【函 数】:K210_Traffic_Disc
 【参 数】:flag：'A'、'B'、'C'、'D'
 【返 回】:
 【简 例】:
 【说 明】:识别交通灯任务
*************************************************************************************************************/
void K210_Traffic_Disc(char flag)
{
  uint8_t traffic_color[8];
  uint8_t Back_Count = 0, Flag_BackGo = 0;
  uint32_t time_out = 0;
  uint32_t shift_BackGo = 0;
  int16_t distance_code = 0;

  delay(200);
  K210_Servo_Control(5);
  delay(1500);
  Traffic_Distinguish(flag); // 发送交通灯请求识别
  delay(500);
  K210_Traffical_StartUp(); // k210十次识别
  Clear_Code();
  time_out = millis();
  while (1)
  {
    // zigbee数据读取
    Zigbee_Recive();
    // K210交通灯数据读取
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00) // K210接收交通灯数据
    {
      ExtSRAMInterface.ExMem_Read_Bytes(0x6038, traffic_color, 8);
      Content_Print_HEX(traffic_color, 8);
      if (traffic_color[2] == 0x91)
      {
        if (traffic_color[4] >= 1 && traffic_color[4] < 4)
        {
          traffic_status = traffic_color[4]; // 颜色标识
          Traffic_ResponseResult(flag);      // 发送识别结果
          break;
        }
        else if (traffic_color[4] == 4)
        {
          Back_Count += 1;
          if (Back_Count == 1)
          {
            DCMotor.Back(50, 150);
            delay(200);
            shift_BackGo = millis();
            DCMotor.Go(25);
          }
          K210_Traffical_StartUp(); // k210十次识别
        }
      }
    }
    if (Back_Count >= 1)
    {
      if ((millis() - shift_BackGo) > 500)
      {
        Flag_BackGo += 1;
        if (Flag_BackGo % 2 == 1)
          DCMotor.SpeedCtr(-25, -25);
        else
          DCMotor.SpeedCtr(25, 25);
        shift_BackGo = millis();
      }
    }
    if (millis() - time_out >= 9000)
    {
      if (traffic_status == 0)
      {
        traffic_status = 1;
        Traffic_ResponseResult(flag); // 发送识别结果
      }
      break;
    }
  }
  DCMotor.Stop();
  distance_code = Read_Code();
  if (Back_Count >= 1)
    if (155 - distance_code >= 0)
    {
      DCMotor.Go(50, abs(155 - distance_code));
    }
    else
    {
      DCMotor.Back(50, abs(155 - distance_code));
    }
  delay(200);
  K210_Servo_Control(-45);
  delay(1500);
  if (Back_Count != 0)
    delay(500);
}
/************************************************************************************************************
 【函 数】:OpenMV_Track
 【参 数】:Car_Speed 车速   distance 距离   mode(已废弃)
 【返 回】:
 【简 例】:
 【说 明】:视觉循迹
*************************************************************************************************************/
void OpenMV_Track(uint8_t Car_Speed, uint8_t distance, uint8_t mode)
{
  uint16_t code_value = 0;
  OpenMV_Track_Route(Car_Speed, distance, mode);
  if (cross_spacefloor == 1) // 特殊地形退出然后循迹，避免在特殊地形出现意外
  {
    cross_spacefloor = 0;
    OpenMV_Track_Route(Car_Speed, 0, 0);
    find_space = 0;
    code_value = Read_Code(); // 读取码盘值
    if (code_value > 600)     // 路径大于600说明特殊路口在十字路口处,用于自适应特殊地形随机摆放
    {
      Recode_FristRoute += 1; // 路径点+1
    }
  }
}
/************************************************************************************************************
 【函 数】:OpenMV_Track_Route
 【参 数】:Car_Speed 车速   distance 距离   mode(已废弃)
 【返 回】:
 【简 例】:
 【说 明】:视觉循迹
*************************************************************************************************************/
void OpenMV_Track_Route(uint8_t Car_Speed, uint8_t distance, uint8_t mode)
{
  uint32_t timeout = 0, track_time = 0;
  int32_t code_value = 0, diffrence_code = 0;
  uint8_t firstbit[8] = {0}, tp = 0, track_value = 0, clear_buf[21] = {0};
  uint8_t last_state = 0, foword_state = 0;
  uint8_t cross_flag = 0;
  uint8_t whilt_flag = 0;
  uint8_t black_offset = 0;
  uint8_t black_angle = 0;
  uint8_t whilt_value = 0;
  int16_t long_C = 0;
  // 清空串口缓存
  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, clear_buf, 21);
  Clear_Array_Date(clear_buf, 21); // 清空数组 Data_OpenMVBuf
  Serial.println(find_space);

  if (find_space == 0)
    Space_Distinguish(); // 特殊地形识别

  Serial.print("cross_spacefloor");
  Serial.println(cross_spacefloor);
  delay(50);
  if (cross_spacefloor != 1)
  {
    // 发送循迹查询请求
    K210_TrackControl(0x01);
    // 判断当前是否能识别到黑色直线
    delay(50);
    while (1)
    {
      // 接收Zigbee数据
      Zigbee_Recive();
      Clear_Code(); // 清除码盘值
      // 检测循迹状态
      if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
      {
        ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 8);
        // 关闭循迹查询
        K210_TrackControl(0x02);
        // 如果没有识别到白黑白
        if (OpenMV_TrackValue_Type(Data_OpenMVBuf[6]) != 6 || OpenMV_TrackValue_Type(Data_OpenMVBuf[4]) != 6 || Count_BlackNumber(Data_OpenMVBuf[6], 8) > 2 || Count_BlackNumber(Data_OpenMVBuf[4], 8) > 2)
        {
          // DCMotor.Back(50,150);//倒退150
          DCMotor.BackSelfDefine(47, 54, 150);
          timeout += 250;
          delay(500);
        }
        break;
      }
    }
    // 清空缓存
    while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
      ExtSRAMInterface.ExMem_Read_Bytes(0x6038, clear_buf, 21);
    Clear_Array_Date(clear_buf, 21); // 清空数组 Data_OpenMVBuf
    // 模式选择
    switch (mode)
    {
    case 0:
      break;
    case 1:
      timeout = timeout + (int)(29 * distance);
      break;
    default:
      break;
    }

    delay(200);
    K210_TrackControl(0x01); // 发送循迹请求
    delay(50);
    DCMotor.SpeedCtr(Car_Speed - 3, Car_Speed + 4);
    track_time = millis();
    // 真正循迹部分
    while (1)
    {
      // 延时循迹
      if (mode == 1 && (millis() - track_time) >= timeout)
      {
        K210_TrackControl(0x02);
        DCMotor.Stop();
        break;
      }
      // zigbee数据读取
      Zigbee_Recive();
      // 读取循迹值
      if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00) // 检测循迹状态
      {
        // 读取K210 循迹状态
        ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 8);
        track_value = Data_OpenMVBuf[6];  // 循迹值
        black_offset = Data_OpenMVBuf[5]; // 循迹线偏移量
        black_angle = Data_OpenMVBuf[4];  // 循迹线角度
        whilt_flag = Data_OpenMVBuf[3];   // 白边判断
        foword_state = OpenMV_TrackValue_Type(track_value);
        whilt_value = (track_value & 0x3C);
        Serial.println(black_offset);
        if (OpenMV_TrackValue_Type(track_value) == 5)
        {
          DCMotor.Stop();
          delay(300);
          K210_TrackControl(0x02);
          break;
        }
        if (!(whilt_flag == 1 && OpenMV_TrackValue_Type(track_value) != 6)) // 是否有白色
        {
          tp = 0;
          firstbit[0] = 0;                          // 黑色状态位置(右边到左边)
          for (size_t i = 0x01; i < 0x100; i <<= 1) // 右到左
          {
            if ((track_value & i) == 0)
            {
              firstbit[tp++] = uint8_t(i);
            }
          }

          if (tp >= 0x05)
          {
            long_C = Read_Code();
            if (long_C >= 0)
            {
              DCMotor.Stop();
              delay(50);
              K210_TrackControl(0x02);
              break;
            }
            else
            {
              DCMotor.SpeedCtr(47, 54);
            }
          }
          else
          {
            switch (firstbit[0])
            {
            case 0x00:
              DCMotor.SpeedCtr(47, 54);
              break;
            case 0x01:
              DCMotor.SpeedCtr(50, -40);
              break;
            case 0x02:
              DCMotor.SpeedCtr(40, -30);
              break;
            case 0x04:
              // 可能在循迹在直线上,精细化然后使车身调整
              DCMotor.SpeedCtr(Car_Speed + 20, Car_Speed - 20);
              break;
            case 0x08:
              // 可能在循迹在直线上,精细化然后使车身调整
              if (black_offset > 157)
                DCMotor.SpeedCtr(Car_Speed + 30, Car_Speed - 30);
              else if (black_offset > 147)
                DCMotor.SpeedCtr(Car_Speed + 20, Car_Speed - 20);
              else if (black_offset > 135)
                DCMotor.SpeedCtr(Car_Speed + 10, Car_Speed - 10);
              else if (black_offset >= 121 && black_offset <= 135)
                DCMotor.SpeedCtr(Car_Speed - 3, Car_Speed + 4);
              else if (black_offset >= 112)
                DCMotor.SpeedCtr(Car_Speed - 10, Car_Speed + 10);
              else if (black_offset >= 97)
                DCMotor.SpeedCtr(Car_Speed - 20, Car_Speed + 20);
              else if (black_offset < 97)
                DCMotor.SpeedCtr(Car_Speed - 30, Car_Speed + 30);
              break;
            case 0x10:
              DCMotor.SpeedCtr(Car_Speed - 20, Car_Speed + 35);
              break;
            case 0x20:
              DCMotor.SpeedCtr(-30, 40);
              break;
            case 0x40:
              DCMotor.SpeedCtr(-40, 50);
              break;
            case 0x80:
              DCMotor.SpeedCtr(-40, 50);
              break;
            }
          }
        }
        else
        {
          DCMotor.SpeedCtr(50, 50);
          // delay(Speed50_Delay * 4);
          while (1)
          {
            if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00) // 检测循迹状态
            {
              ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf,
                                                8); // 读取K210循迹
              track_value = Data_OpenMVBuf[6];      // 循迹值
              whilt_flag = Data_OpenMVBuf[3];       // 白边判断
              if (whilt_flag == 0)                  // 由白边到没白边
              {
                if (OpenMV_TrackValue_Type(Data_OpenMVBuf[6]) != 6 || OpenMV_TrackValue_Type(Data_OpenMVBuf[4]) != 6)
                {
                  cross_flag = 1;
                  DCMotor.Stop();
                  delay(50);
                  K210_TrackControl(0x02);
                  delay(500);
                  Space_Distinguish(); // 特殊地形识别
                }
                break;
              }
            }
          }
          if (cross_flag == 1) // 白卡十字路口或者过完特殊地形结束
          {
            cross_flag = 0;
            DCMotor.Stop();
            delay(50);
            K210_TrackControl(0x02);
            break;
          }
        }
      }
      last_state = foword_state;
    }
  }
  DCMotor.Stop();
}
/************************************************************************************************************
 【函 数】:OpenMV_Track_Distance
 【参 数】:Car_Speed 车速   distance 距离
 【返 回】:
 【简 例】:
 【说 明】:视觉调整
*************************************************************************************************************/
void OpenMV_Track_Distance(uint8_t Car_Speed, uint8_t distance)
{
  uint32_t timeout = 0, track_time = 0;
  int32_t code_value = 0, diffrence_code = 0;
  uint8_t firstbit[8] = {0}, tp = 0, track_value = 0, clear_buf[21] = {0};
  uint8_t last_state = 0, foword_state = 0;
  uint8_t cross_flag = 0;
  uint8_t whilt_flag = 0;
  uint8_t black_offset = 0;
  uint8_t black_angle = 0;
  uint8_t whilt_value = 0;
  int16_t long_C = 0;

  // 清空串口缓存
  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, clear_buf, 21);
  Clear_Array_Date(clear_buf, 21); // 清空数组 Data_OpenMVBuf

  // 发送循迹查询请求
  K210_TrackControl(0x01);
  // 判断当前是否能识别到黑色直线
  delay(50);
  while (1)
  {
    // 接收Zigbee数据
    Zigbee_Recive();
    Clear_Code(); // 清除码盘值
    // 检测循迹状态
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    {
      ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 8);
      // 关闭循迹查询
      K210_TrackControl(0x02);
      // 如果没有识别到白黑白
      if (OpenMV_TrackValue_Type(Data_OpenMVBuf[6]) != 6 || OpenMV_TrackValue_Type(Data_OpenMVBuf[4]) != 6 || Count_BlackNumber(Data_OpenMVBuf[6], 8) > 2 || Count_BlackNumber(Data_OpenMVBuf[4], 8) > 2)
      {
        // DCMotor.Back(50,150);//倒退150
        DCMotor.BackSelfDefine(47, 54, 150);
        timeout += 250;
        delay(500);
      }
      break;
    }
  }
  // 时间循迹
  timeout = timeout + (int)(29 * distance);
  // 清空缓存
  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, clear_buf, 21);
  Clear_Array_Date(clear_buf, 21); // 清空数组 Data_OpenMVBuf

  delay(200);
  K210_TrackControl(0x01); // 发送循迹请求
  delay(50);
  DCMotor.SpeedCtr(Car_Speed - 3, Car_Speed + 4);
  track_time = millis();
  // 真正循迹部分
  while (1)
  {
    // 延时循迹
    if ((millis() - track_time) >= timeout)
    {
      DCMotor.Stop();
      delay(50);
      K210_TrackControl(0x02);
      break;
    }
    // zigbee数据读取
    Zigbee_Recive();
    // 读取循迹值
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00) // 检测循迹状态
    {
      // 读取K210 循迹状态
      ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 8);
      track_value = Data_OpenMVBuf[6];                                    // 循迹值
      black_offset = Data_OpenMVBuf[5];                                   // 循迹线偏移量
      black_angle = Data_OpenMVBuf[4];                                    // 循迹线角度
      whilt_flag = Data_OpenMVBuf[3];                                     // 白边判断
      if (!(whilt_flag == 1 && OpenMV_TrackValue_Type(track_value) != 6)) // 是否有白色
      {
        tp = 0;
        firstbit[0] = 0;                          // 黑色状态位置(右边到左边)
        for (size_t i = 0x01; i < 0x100; i <<= 1) // 右到左
        {
          if ((track_value & i) == 0)
          {
            firstbit[tp++] = uint8_t(i);
          }
        }

        if (tp >= 0x05)
        {
          DCMotor.SpeedCtr(47, 54);
        }
        else
        {
          switch (firstbit[0])
          {
          case 0x00:
            DCMotor.SpeedCtr(47, 54);
            break;
          case 0x01:
            DCMotor.SpeedCtr(50, -40);
            break;
          case 0x02:
            DCMotor.SpeedCtr(40, -30);
            break;
          case 0x04:
            // 可能在循迹在直线上,精细化然后使车身调整
            DCMotor.SpeedCtr(Car_Speed + 20, Car_Speed - 20);
            break;
          case 0x08:
            // 可能在循迹在直线上,精细化然后使车身调整
            if (black_offset > 157)
              DCMotor.SpeedCtr(Car_Speed + 30, Car_Speed - 30);
            else if (black_offset > 147)
              DCMotor.SpeedCtr(Car_Speed + 20, Car_Speed - 20);
            else if (black_offset > 135)
              DCMotor.SpeedCtr(Car_Speed + 10, Car_Speed - 10);
            else if (black_offset >= 121 && black_offset <= 135)
              DCMotor.SpeedCtr(Car_Speed - 3, Car_Speed + 4);
            else if (black_offset >= 112)
              DCMotor.SpeedCtr(Car_Speed - 10, Car_Speed + 10);
            else if (black_offset >= 97)
              DCMotor.SpeedCtr(Car_Speed - 20, Car_Speed + 20);
            else if (black_offset < 97)
              DCMotor.SpeedCtr(Car_Speed - 30, Car_Speed + 30);
            break;
          case 0x10:
            DCMotor.SpeedCtr(Car_Speed - 20, Car_Speed + 20);
            break;
          case 0x20:
            DCMotor.SpeedCtr(-30, 40);
            break;
          case 0x40:
            DCMotor.SpeedCtr(-40, 50);
            break;
          case 0x80:
            DCMotor.SpeedCtr(-40, 50);
            break;
          }
        }
      }
      else
      {
        DCMotor.SpeedCtr(50, 50);
        // delay(Speed50_Delay * 4);
        while (1)
        {
          if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00) // 检测循迹状态
          {
            ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf,
                                              8); // 读取K210循迹
            track_value = Data_OpenMVBuf[6];      // 循迹值
            whilt_flag = Data_OpenMVBuf[3];       // 白边判断
            if (whilt_flag == 0)                  // 由白边到没白边
            {
              break;
            }
          }
        }
      }
    }
  }
  DCMotor.Stop();
}
/************************************************************************************************************
 【函 数】:Special_Road_Identify
 【参 数】:
 【返 回】:返回结果 1是0非
 【简 例】:
 【说 明】:特殊地形识别
*************************************************************************************************************/
uint8_t Special_Road_Identify(void)
{
  if (!Find_SpeicalRoad)
  {
    return false;
  }

  uint8_t space_buf[8];  // 接收识别结果
  uint8_t clear_buf[21]; // 清空地址缓存
  uint8_t Space_Result = 0;

  // 清空地址，长度21，因为二维码识别中最长发送长度为二一
  delay(50);
  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, clear_buf, 21);
  Clear_Array_Date(clear_buf, 21); // 清空数组 Data_OpenMVBuf
  delay(50);
  Clear_Code(); // 清除码盘值
  delay(50);
  K210_Check_Space(); // 发送识别特殊地形请求
  while (1)
  {
    // Zigbee接收程序
    Zigbee_Recive();
    // 检测地址是否有数据
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    {
      ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 8); // 读取地址数据
      Find_SpeicalRoad = false;
      return Data_OpenMVBuf[3]; // 特殊地形识别结果
    }
  }
}
/************************************************************************************************************
 【函 数】:Space_Distinguish
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:特殊地形识别并处理
*************************************************************************************************************/
void Space_Distinguish(void)
{
  uint8_t space_buf[8];  // 接收识别结果
  uint8_t clear_buf[21]; // 清空地址缓存
  uint8_t Space_Result = 0;

  // 清空地址，长度21，因为二维码识别中最长发送长度为二一
  delay(50);
  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, clear_buf, 21);
  Clear_Array_Date(clear_buf, 21); // 清空数组 Data_OpenMVBuf
  delay(50);
  Clear_Code(); // 清除码盘值
  delay(50);
  K210_Check_Space(); // 发送识别特殊地形请求
  while (1)
  {
    // Zigbee接收程序
    Zigbee_Recive();
    // 检测地址是否有数据
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    {
      ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 8); // 读取地址数据
      Space_Result = Data_OpenMVBuf[3];                             // 特殊地形识别结果
      break;
    }
  }
  // 存在特殊地形,直接调整车身,冲过去
  if (Space_Result == 1)
  {
    BackDis_GoSpace(4, 12, 0);         // 调整三次
    DCMotor.GoSelfDefine(47, 54, 960); // 前进960码，直接冲过特殊地形
    cross_spacefloor = 1;
    find_space = 1;
  }
}
/************************************************************************************************************
 【函 数】:BackDis_GoSpace
 【参 数】:count 调正次数   distance 距离   mode 循迹模式
 【返 回】:
 【简 例】:
 【说 明】:调整车身
*************************************************************************************************************/
void BackDis_GoSpace(uint8_t count, uint8_t distance, uint8_t mode)
{
  for (uint8_t i = 0; i < count; i++)
  {
    DCMotor.BackSelfDefine(47, 54, (int)(distance * Code_ToLength));
    delay(500);
    Space_Adjust(50, (int)(distance * Code_ToLength), mode);
    delay(500);
  }
}
/************************************************************************************************************
 【函 数】:Space_Adjust
 【参 数】:Car_Speed 车速   distance 距离   mode 循迹模式
 【返 回】:
 【简 例】:
 【说 明】:调整车身
*************************************************************************************************************/
void Space_Adjust(uint8_t Car_Speed, uint8_t distance, uint8_t mode)
{
  uint32_t timeout = 0, track_time = 0;
  int32_t code_value = 0, diffrence_code = 0;
  uint8_t firstbit[8] = {0}, tp = 0, track_value = 0, clear_buf[21] = {0};
  uint8_t cross_flag = 0;
  uint8_t whilt_flag = 0;
  uint8_t black_offset = 0;
  uint8_t black_angle = 0;
  uint8_t whilt_value = 0;
  // 清空串口缓存
  while (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00)
    ExtSRAMInterface.ExMem_Read_Bytes(0x6038, clear_buf, 21);
  Clear_Array_Date(clear_buf, 21); // 清空数组 Data_OpenMVBuf
  // 模式选择
  switch (mode)
  {
  case 0:
    break;
  case 1:
    timeout = (int)(29 * distance);
    break;
  default:
    break;
  }

  delay(200);
  K210_TrackControl(0x01); // 发送循迹请求
  delay(50);
  DCMotor.SpeedCtr(Car_Speed - 3, Car_Speed + 4);
  track_time = millis();
  // 真正循迹部分
  while (1)
  {
    // 延时循迹
    if (mode == 1 && (millis() - track_time) >= timeout)
    {
      K210_TrackControl(0x02);
      DCMotor.Stop();
      break;
    }
    // zigbee数据读取
    Zigbee_Recive();
    // 读取循迹值
    if (ExtSRAMInterface.ExMem_Read(0x6038) != 0x00) // 检测循迹状态
    {
      // 读取K210 循迹状态
      ExtSRAMInterface.ExMem_Read_Bytes(0x6038, Data_OpenMVBuf, 8);
      track_value = Data_OpenMVBuf[6];  // 循迹值
      black_offset = Data_OpenMVBuf[5]; // 循迹线偏移量
      black_angle = Data_OpenMVBuf[4];  // 循迹线角度
      whilt_flag = Data_OpenMVBuf[3];   // 白边判断

      if (!(whilt_flag == 1 && OpenMV_TrackValue_Type(track_value) != 6)) // 是否有白色
      {
        tp = 0;
        firstbit[0] = 0;                          // 黑色状态位置(右边到左边)
        for (size_t i = 0x01; i < 0x100; i <<= 1) // 右到左
        {
          if ((track_value & i) == 0)
          {
            firstbit[tp++] = uint8_t(i);
          }
        }

        if (tp >= 0x05)
        {
          K210_TrackControl(0x02); // 发送循迹请求
          DCMotor.Stop();
          break;
        }
        else
        {
          switch (firstbit[0])
          {
          case 0x00:
            DCMotor.SpeedCtr(47, 54);
            break;
          case 0x01:
            DCMotor.SpeedCtr(50, -40);
            break;
          case 0x02:
            DCMotor.SpeedCtr(40, -30);
            break;
          case 0x04:
            // 可能在循迹在直线上,精细化然后使车身调整
            DCMotor.SpeedCtr(Car_Speed + 20, Car_Speed - 20);
            break;
          case 0x08:
            // 可能在循迹在直线上,精细化然后使车身调整
            if (black_offset > 157)
              DCMotor.SpeedCtr(Car_Speed + 30, Car_Speed - 30);
            else if (black_offset > 147)
              DCMotor.SpeedCtr(Car_Speed + 20, Car_Speed - 20);
            else if (black_offset > 135)
              DCMotor.SpeedCtr(Car_Speed + 10, Car_Speed - 10);
            else if (black_offset >= 121 && black_offset <= 135)
              DCMotor.SpeedCtr(Car_Speed - 3, Car_Speed + 4);
            else if (black_offset >= 112)
              DCMotor.SpeedCtr(Car_Speed - 10, Car_Speed + 10);
            else if (black_offset >= 97)
              DCMotor.SpeedCtr(Car_Speed - 20, Car_Speed + 20);
            else if (black_offset < 97)
              DCMotor.SpeedCtr(Car_Speed - 30, Car_Speed + 30);
            break;
          case 0x10:
            DCMotor.SpeedCtr(Car_Speed - 20, Car_Speed + 35);
            break;
          case 0x20:
            DCMotor.SpeedCtr(-30, 40);
            break;
          case 0x40:
            DCMotor.SpeedCtr(-40, 50);
            break;
          case 0x80:
            DCMotor.SpeedCtr(-40, 50);
            break;
          }
        }
      }
      else
      {
        K210_TrackControl(0x02); // 发送循迹请求
        DCMotor.Stop();
        break;
      }
    }
  }

  DCMotor.Stop();
}
/************************************************************************************************************
 【函 数】:OpenMV_TrackValue_Type
 【参 数】:trackvalue k210计算的循迹值
 【返 回】:循迹状态
 【简 例】:
 【说 明】:计算循迹状态,此处用于视觉循迹
*************************************************************************************************************/
uint8_t OpenMV_TrackValue_Type(uint8_t trackvalue)
{
  uint8_t array_track[8], change_number = 0, res;
  // 循迹状态赋值
  for (uint8_t i = 0; i < 8; i++)
  {
    if ((trackvalue % 2) == 1)
      array_track[i] = 1;
    else
      array_track[i] = 0;
    if (i != 0 && (array_track[i - 1] != array_track[i]))
    {
      change_number += 1;
    }
    trackvalue = trackvalue / 2;
  }
  // 计算循迹状态
  switch (change_number)
  {
  case 0:
    if (array_track[0] == 0)
      res = 1; // 全黑
    else
      res = 2; // 全白
    break;
  case 1:
    if (array_track[0] == 0)
      res = 3; // 黑白
    else
      res = 4; // 白黑
    break;
  case 2:
    if (array_track[0] == 0)
      res = 5; // 黑白黑
    else
      res = 6; // 白黑白
    break;
  default:
    res = 0; // 其他状态
    break;
  }
  return res;
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 主车通讯 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:Send_AGV_Zigbee
 【参 数】:cmd 主指令   cmd1~3 副指令
 【返 回】:
 【简 例】:
 【说 明】:发送数据给主车,但不实用,主车也没写接收模块
*************************************************************************************************************/
void Send_AGV_Zigbee(uint8_t cmd, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3)
{
  Send_AGV[2] = cmd;
  Send_AGV[3] = cmd1;
  Send_AGV[4] = cmd2;
  Send_AGV[5] = cmd3;
  Command.Judgment(Send_AGV); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, Send_AGV, 8);
}
/************************************************************************************************************
 【函 数】:Sent_QR_ToMainCar
 【参 数】:sentlength 二维码数据长度   cmd 二维码信息
 【返 回】:
 【简 例】:Sent_QR_ToMainCar(QRcode_Length[0],QRcode_Array[0])
 【说 明】:将二维码一信息发送给主车,如果二维码信息自己能处理,可处理完将信息用下面,下面两个函数发送至主车,协议得跟主车定义
*************************************************************************************************************/
void Sent_QR_ToMainCar(uint8_t sentlength, uint8_t *cmd)
{
  uint8_t i = 0;
  Define_Zero48[0] = 0x55; // 将二维码信息加入帧头帧尾
  Define_Zero48[1] = 0x02;
  Define_Zero48[2] = 0x92;
  Define_Zero48[3] = sentlength;
  for (i = 0; i < sentlength; i++)
  {
    Define_Zero48[i + 4] = cmd[i];
  }
  Define_Zero48[i + 4] = 0xBB;
  // 上传至主车
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, Define_Zero48, sentlength + 5);
  // 清空数组
  Clear_Array_Date(Define_Zero48, 48);
}
/************************************************************************************************************
 【函 数】:Sent_SingleDate_ToMainCar
 【参 数】:main_cmd 主指令(类型为Tomain_Maincmd)    cmd 数据
 【返 回】:
 【简 例】:Sent_SingleDate_ToMainCar(ToMainCar_Movement,0); 向主车发送启动指令
 【说 明】:发送单个数据主车,或者只是主指令,数据为任意(例如0XB0请求主车退出避让)
*************************************************************************************************************/
void Sent_SingleDate_ToMainCar(uint8_t main_cmd, uint8_t data)
{
  uint8_t i = 0;
  Define_Zero48[0] = 0x55;
  Define_Zero48[1] = 0x02;
  Define_Zero48[2] = main_cmd; // 主指令
  Define_Zero48[3] = data;
  Define_Zero48[4] = 0xBB; // 数据
  // 上传至主车
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, Define_Zero48, 5);
  // 清空数组
  Clear_Array_Date(Define_Zero48, 48);
}
/************************************************************************************************************
 【函 数】:Sent_MultiDate_ToMainCar
 【参 数】:main_cmd 主指令  cmd 数据  length 数据长度
 【返 回】:
 【简 例】:Sent_MultiDate_ToMainCar(ToMainCar_CarPlate, Road_Code, 6); 向主车发送6位车牌
 【说 明】:发送多个数据给主车
*************************************************************************************************************/
void Sent_MultiDate_ToMainCar(uint8_t main_cmd, uint8_t *data, uint8_t length)
{
  uint8_t i = 0;
  Define_Zero48[0] = 0x55; // 将二维码信息加入帧头帧尾
  Define_Zero48[1] = 0x02;
  Define_Zero48[2] = main_cmd; // 主指令
  for (i = 0; i < length; i++)
  {
    Define_Zero48[i + 3] = data[i];
  }
  // 尾帧
  Define_Zero48[i + 3] = 0xBB;
  // 上传至主车
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, Define_Zero48, length + 4);
  // 清空数组
  for (i = 0; i < (length + 4); i++)
  {
    Define_Zero48[i] = 0x00;
  }
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 道闸标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:RoadGate_Control
 【参 数】:cmd 道闸开关(1开/2关)
 【返 回】:
 【简 例】:RoadGate_Control(1); 打开道闸
 【说 明】:道闸控制
*************************************************************************************************************/
void RoadGate_Control(uint8_t cmd)
{
  road_buf[2] = 0x01;
  road_buf[3] = cmd;
  road_buf[4] = 0;
  road_buf[5] = 0;
  Command.Judgment(road_buf);                              // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, road_buf, 8); // 发送协议
}
/************************************************************************************************************
 【函 数】:Road_Gate_Adjust
 【参 数】:cmd 道闸升降(1升/2降)
 【返 回】:
 【简 例】:
 【说 明】:调节闸门角度
*************************************************************************************************************/
void RoadGate_Adjust(uint8_t cmd)
{
  road_buf[2] = 0x09;
  road_buf[3] = cmd;
  road_buf[4] = 0;
  road_buf[5] = 0;
  Command.Judgment(road_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, road_buf, 8);
}
/************************************************************************************************************
 【函 数】:RoadGate_Plate
 【参 数】:plate 车牌
 【返 回】:
 【简 例】:RoadGate_Plate("A123B4"); 显示车牌并开启
 【说 明】:显示车牌并打开道闸
*************************************************************************************************************/
void RoadGate_Plate(uint8_t plate[6])
{
  delay(1000);

  // 发送前三位车牌
  road_buf[2] = 0x10;
  road_buf[3] = plate[0];
  road_buf[4] = plate[1];
  road_buf[5] = plate[2];
  Command.Judgment(road_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, road_buf, 8);
  delay(1000);

  // 发送后三位车牌
  road_buf[2] = 0x11;
  road_buf[3] = plate[3];
  road_buf[4] = plate[4];
  road_buf[5] = plate[5];
  Command.Judgment(road_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, road_buf, 8);
  delay(1000);
}
/************************************************************************************************************
 【函 数】:Road_Gate_Flag
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:查询道闸状态
*************************************************************************************************************/
void RoadGate_Check(void)
{
  // 请求回传道闸信息
  road_buf[2] = 0x20;
  road_buf[3] = 0x01;
  road_buf[4] = 0x00;
  road_buf[5] = 0x00;
  Command.Judgment(road_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, road_buf, 8);
  delay(500);
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ LED显示标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:LEDDisplay_Date
 【参 数】:cmd 主指令(1第一排数据/2第二排数据)    cmd1~3(数据位)
 【返 回】:
 【简 例】:LEDDisplay_Date(1,0xff,0xff,0xff); 第一排显示ffffff
 【说 明】:LED标志物显示数据
*************************************************************************************************************/
void LEDDisplay_Date(uint8_t cmd, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3)
{
  led_data_buf[2] = cmd;  // 主指令
  led_data_buf[3] = cmd1; // 后面三位数据
  led_data_buf[4] = cmd2;
  led_data_buf[5] = cmd3;

  Command.Judgment(led_data_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, led_data_buf, 8);
}
/************************************************************************************************************
 【函 数】:LEDDisplay_TimerControl
 【参 数】:cmd 指令(0关闭/1开启/2清除)
 【返 回】:
 【简 例】:LEDDisplay_TimerControl(1); 开启计时模式
 【说 明】:LED显示标志物进行更改计时模式
*************************************************************************************************************/
void LEDDisplay_TimerControl(uint8_t cmd)
{
  led_time_buf[2] = 0x03; // 主指令
  led_time_buf[3] = cmd;  // 副指令

  Command.Judgment(led_time_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, led_time_buf, 8);
}
/************************************************************************************************************
 【函 数】:LED_Display_Distence
 【参 数】:
 【返 回】:
 【简 例】:LED_Display_Distence(); LED标志物显示超声波距离
 【说 明】:LED标志物显示超声波距离
*************************************************************************************************************/
void LEDDisplay_Distence(void)
{
  float distance_date = 0;

  Ultrasonic_Ranging(); // 超声波测距

  led_distance_buf[4] = Ultrasonic_Distance[0];                                 // 显示距离百位
  led_distance_buf[5] = Ultrasonic_Distance[1] * 0x10 + Ultrasonic_Distance[2]; // 显示距离十位个位

  Command.Judgment(led_distance_buf);                              // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, led_distance_buf, 8); // 发送Zigbee
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 立体车库标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:StereoGarage_ToLayer
 【参 数】:garage_x ('A'车库A/'B'车库B)   layer 代表第几层
 【返 回】:
 【简 例】:StereoGarage_ToLayer('A',1); 车库A在到达第一层
 【说 明】:立体车库到达第X层
*************************************************************************************************************/
void StereoGarage_ToLayer(uint8_t garage_x, uint8_t layer)
{
  switch (garage_x)
  {
  case 'A':
    stereogarage_buf[1] = 0x0D;
    break;
  case 'B':
    stereogarage_buf[1] = 0x05;
    break;
  default:
    break;
  }
  stereogarage_buf[2] = 0x01;
  stereogarage_buf[3] = layer; // 到达N层

  Command.Judgment(stereogarage_buf);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, stereogarage_buf, 8);
}
/************************************************************************************************************
 【函 数】:StereoGarage_Aquire_Return
 【参 数】:garage_x ('A'车库A/'B'车库B)
 【返 回】:
 【简 例】:StereoGarage_Aquire_Return('A'); 车库A在第几层
 【说 明】:发送返回请求
*************************************************************************************************************/
void StereoGarage_Aquire_Return(uint8_t garage_x, uint8_t cmd)
{
  switch (garage_x)
  {
  case 'A':
    stereosarage_quire_buf[1] = 0x0D;
    break;
  case 'B':
    stereosarage_quire_buf[1] = 0x05;
    break;
  default:
    break;
  }
  stereosarage_quire_buf[2] = 0x02;
  stereosarage_quire_buf[3] = cmd;

  Command.Judgment(stereosarage_quire_buf);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, stereosarage_quire_buf, 8);
}
/************************************************************************************************************
 【函 数】:StereoGarage_WaitingToFloor
 【参 数】:garage_x ('A'车库A/'B'车库B)
 【返 回】:
 【简 例】:
 【说 明】:从车等待车库降至一层
*************************************************************************************************************/
void StereoGarage_WaitingToFloor(uint8_t garage_x)
{
  uint16_t judge = 0, Flag_AB, flag_floor = 0;
  uint8_t judgment_agreement[8];

  while (!flag_floor)
  {
    // 向立体车库请求返回目前位于第几层//
    if (judge == 0)
    {
      switch (garage_x)
      {
      case 'A':
        StereoGarage_Aquire_Return('A', 1);
        Flag_AB = 0X0D;
        break;
      case 'B':
        StereoGarage_Aquire_Return('B', 1);
        Flag_AB = 0X05;
        break;
      default:
        break;
      }
    }

    delay(500);
    // 读取车库当前层数//
    ExtSRAMInterface.ExMem_Read_Bytes(0x6100, judgment_agreement, 8);
    // 判断车库是否为第一层
    if (judgment_agreement[1] == Flag_AB && judgment_agreement[3] == 0x01)
    {
      if (judgment_agreement[4] != 0x01)
      { // 执行一次升降
        if (judge == 0)
        {
          switch (garage_x)
          {
          case 'A':
            Init_AGarage_Floor = judgment_agreement[4];
            break; // 取当前'A'车库首次层数
          case 'B':
            Init_BGarage_Floor = judgment_agreement[4];
            break; // 取当前'B'车库首次层数
          default:
            break;
          }
          StereoGarage_ToLayer(garage_x, 1); // 下降车库,并且只执行一次
          judge = 1;
        }
        StereoGarage_Aquire_Return(Flag_AB, 1);
      }
      else
      {
        flag_floor = 1;
        break; // 车库为第一层就退出
      }
    }
  }
  // 第一层赋值
  switch (garage_x)
  {
  case 'A':
    if (Init_AGarage_Floor == 0)
      Init_AGarage_Floor = 1;
    break; // 取当前'A'车库首次层数
  case 'B':
    if (Init_BGarage_Floor == 0)
      Init_BGarage_Floor = 1;
    break; // 取当前'B'车库首次层数
  default:
    break;
  }
}
/************************************************************************************************************
 【函 数】:StereoGarage_BackToHome
 【参 数】:garage_x ('A'车库A/'B'车库B)
 【返 回】:
 【简 例】:
 【说 明】:检测车库红外信息
*************************************************************************************************************/
void StereoGarage_BackToHome(uint8_t garage_x)
{
  uint8_t Flag_AB = 0;
  switch (garage_x) // 确认B车入库车库
  {
  case 'A':
    Flag_AB = 0X0D;
    break; // 车库A
  case 'B':
    Flag_AB = 0X05;
    break; // 车库B
  default:
    break;
  }
  StereoGarage_Aquire_Return(garage_x, 2); // 发送红外协议
  if (Flag_AB == 0X0D)
  {
    // 倒车判断
    while (garageA_quit_move != 2) // 确认A车库入库退出标志位
    {
      // zigbee数据读取
      Zigbee_Recive();

      if (garageA_move_front == 1)
      {
        DCMotor.SpeedCtr(-30, -30);              // 倒退
        StereoGarage_Aquire_Return(garage_x, 2); // 发送红外协议
        garageA_move_front = 0;
      }
      else if (garageA_move_front == 2)
      {
        DCMotor.Stop();
        DCMotor.Go(50, 30);                      // 前进
        StereoGarage_Aquire_Return(garage_x, 2); // 发送红外协议
        garageA_move_front = 0;
      }
      else
      {
        StereoGarage_Aquire_Return(garage_x, 2); // 发送红外协议
        delay(100);
      }
    }
    garageA_quit_move = 0;
    garageA_move_front = 0;
  }
  else
  {
    while (garageB_quit_move != 2) // 确认B车库入库退出标志位
    {
      // zigbee数据读取
      Zigbee_Recive();
      if (garageB_move_front == 1)
      {
        DCMotor.SpeedCtr(-30, -30);              // 倒退
        StereoGarage_Aquire_Return(garage_x, 2); // 发送红外协议
        garageB_move_front = 0;
      }
      else if (garageB_move_front == 2)
      {
        DCMotor.Stop();
        DCMotor.Go(50, 30);                      // 前进
        StereoGarage_Aquire_Return(garage_x, 2); // 发送红外协议
        garageB_move_front = 0;
      }
      else
      {
        StereoGarage_Aquire_Return(garage_x, 2); // 发送红外协议
        delay(200);
      }
    }
    garageB_quit_move = 0;
    garageB_move_front = 0;
  }
  DCMotor.Stop(); // 小车停止
}
/************************************************************************************************************
 【函 数】:StereoGarage_Quire_InitFloor
 【参 数】:garage_x ('A'车库A/'B'车库B)
 【返 回】:
 【简 例】:
 【说 明】:获取车库初始层数
*************************************************************************************************************/
void StereoGarage_Quire_InitFloor(uint8_t garage_x)
{
  StereoGarage_Aquire_Return(garage_x, 0x01);
  while (1)
  {
    // zigbee数据读取
    Zigbee_Recive();
    if (garage_x == 'A')
    {
      if (Init_AGarage_Floor != 0 && Init_AGarage_Floor >= 1 && Init_AGarage_Floor <= 4)
      {
        break;
      }
      else
      {
        StereoGarage_Aquire_Return(garage_x, 0x01);
        delay(200);
      }
    }
    else
    {
      if (Init_BGarage_Floor != 0 && Init_BGarage_Floor >= 1 && Init_BGarage_Floor <= 4)
      {
        break;
      }
      else
      {
        StereoGarage_Aquire_Return(garage_x, 0x01);
        delay(200);
      }
    }
  }
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 语音播报标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:Voice_Announcements
 【参 数】: cmd 0x01:播报“技能成才”                  0x02:播报“匠心筑梦”
               0x03:播报“逐梦扬威”                  0x04:播报“技行天下”
               0x05:播报“展行业百技”                0x06:播报“树人才新观”
 【返 回】:
 【简 例】:
 【说 明】:指定播放语音
*************************************************************************************************************/
void Voice_Announcements(uint8_t cmd)
{
  voice_buf[2] = 0x10;
  voice_buf[3] = cmd;
  Command.Judgment(voice_buf);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, voice_buf, 8);
}
/************************************************************************************************************
 【函 数】:Voice_Random_Broadcast
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:随机播报语音编号
*************************************************************************************************************/
void Voice_Random_Broadcast(void)
{
  voice_buf[2] = 0x20;
  voice_buf[3] = 0x01;
  Command.Judgment(voice_buf);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, voice_buf, 8);
}
/************************************************************************************************************
 【函 数】:Voice_Set_Time
 【参 数】:cmd~cmd2 RTC起始日期   cmd3~cmd5 RTC起始时间
 【返 回】:
 【简 例】:Voice_Set_Time(0x23,0x03,0x01,0x23,0x03,0x02); 设置RTC起始日期为2023年03月1日,设置RTC起始时间为2023年03月2日
 【说 明】:设置RTC起始日期,设置RTC起始时间
*************************************************************************************************************/
void Voice_Set_Time(uint8_t cmd, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t cmd4, uint8_t cmd5)
{
  uint8_t voice_time_buf[8] = {0x55, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBB};
  voice_time_buf[2] = 0x30; // 设置RTC起始日期
  voice_time_buf[3] = cmd;  // 年
  voice_time_buf[4] = cmd1; // 月
  voice_time_buf[5] = cmd2; // 日
  Command.Judgment(voice_time_buf);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, voice_time_buf, 8);

  delay(500);

  voice_time_buf[2] = 0x40; // 设置RTC起始时间
  voice_time_buf[3] = cmd3; // 时
  voice_time_buf[4] = cmd4; // 分
  voice_time_buf[5] = cmd5; // 秒
  Command.Judgment(voice_time_buf);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, voice_time_buf, 8);

  delay(500);
}
/************************************************************************************************************
 【函 数】:voice_Set_Weather
 【参 数】:cmd:大风：0x00   多云：0x01    晴天：0x02    cmd1：温度(十六进制数,两者都是)
              小雪：0x03   小雨：0x04    阴天：0x05
 【返 回】:
 【简 例】:Voice_SetWeather(0x01,0x19); 设置天气为多云，25℃
 【说 明】:设置天气,设置温度
*************************************************************************************************************/
void Voice_SetWeather(uint8_t cmd, uint8_t cmd1)
{
  voice_weather_buf[3] = cmd;  // 天气
  voice_weather_buf[4] = cmd1; // 温度
  Command.Judgment(voice_weather_buf);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, voice_weather_buf, 8);
}
/************************************************************************************************************
 【函 数】:Voice_Check_Task
 【参 数】:cmd: 0x01 查询 RTC 当前日期 0x02：查询 RTC 当前时间
 【返 回】:
 【简 例】:Voice_Check_Task(1); 查询 RTC 当前日期
 【说 明】:查询RTC起始日期,查询RTC起始时间
*************************************************************************************************************/
void Voice_Check_Task(uint8_t cmd)
{
  uint8_t recieve_data[8];
  switch (cmd)
  {
  case 1:
    voice_check_buf[2] = 0x31; // 查询 RTC 当前日期
    voice_check_buf[3] = 0x01;
    break;
  case 2:
    voice_check_buf[2] = 0x41; // 查询 RTC 当前时间
    voice_check_buf[3] = 0x01;
    break;
  case 3:
    voice_check_buf[2] = 0x43; // 请求回传天气数据与温度数据(16 进制，单位℃)
    voice_check_buf[3] = 0x00;
    break;
  default:
    break;
  }
  Command.Judgment(voice_check_buf);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, voice_check_buf, 8);
  while (1)
  {
    if (ExtSRAMInterface.ExMem_Read(0x6100) != 0x00)
    {
      ExtSRAMInterface.ExMem_Read_Bytes(0x6100, recieve_data, 8);
      if (recieve_data[0] == 0x55 && recieve_data[7] == 0xBB)
      {
        switch (recieve_data[2]) // 语音播报标志物
        {
        case 0x01:
          break;
        case 0x02:
          Rtc_Data[0] = recieve_data[3]; // 返回RTC日期
          Rtc_Data[1] = recieve_data[4];
          Rtc_Data[2] = recieve_data[5];
          break;
        case 0x03:
          Rtc_Data[3] = recieve_data[3]; // 返回RTC时间
          Rtc_Data[4] = recieve_data[4];
          Rtc_Data[5] = recieve_data[5];
          break;
        case 0x04:
          Weather_Temp[0] = recieve_data[3]; // 返回温度和天气
          Weather_Temp[1] = recieve_data[4];
          break;
        default:
          break;
        }
        break;
      }
    }
  }
}
/************************************************************************************************************
 【函 数】:Speech_Sounds_Ctr
 【参 数】:
 【返 回】:
 【简 例】:Speech_Sounds_Ctr();
 【说 明】:发送语音合成的数据
*************************************************************************************************************/
void Speech_Sounds_Ctr(void)
{
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, test_buf, sizeof(test_buf));
}
/************************************************************************************************************
 【函 数】:Voice_RTC_Synthesis
 【参 数】:参数和英文一致
 【返 回】:
 【简 例】:Voice_RTC_Synthesis(0x23,0x03,0x31,0x23,0x23,0x23); 当前时间二零二三年三月三十一日二十三十时二十三分二十三秒
 【说 明】:播报天气温度
*************************************************************************************************************/
void Voice_RTC_Synthesis(uint8_t year, uint8_t money, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) // 年月日时分秒
{
  uint16_t number_data[2] = {0, 0};
  year_voice[17] = Number_Voice[(year / 16) * 2]; // 年赋值
  year_voice[18] = Number_Voice[(year / 16) * 2 + 1];
  year_voice[19] = Number_Voice[(year % 16) * 2];
  year_voice[20] = Number_Voice[(year % 16) * 2 + 1];

  Change_Voice_Content(date_voice, 11, day);
  Change_Voice_Content(time_voice, 5, hour);
  Change_Voice_Content(time_voice, 13, minute);
  Change_Voice_Content(time_voice, 21, second);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, year_voice, sizeof(year_voice));
  delay(2000);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, date_voice, sizeof(date_voice));
  delay(2000);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, time_voice, sizeof(time_voice));
  delay(2500);
}
/************************************************************************************************************
 【函 数】:Whether_Dispaly
 【参 数】:wethear 0大风1多云2晴3小雪    temp 温度
 【返 回】:
 【简 例】:Whether_Dispaly(0x00,0x19); 当前温度大风二十五摄氏度
 【说 明】:播报天气温度
*************************************************************************************************************/
void Whether_Dispaly(uint8_t wethear, uint8_t temp)
{
  temp = (temp / 10 * 16) + (temp % 10);

  for (int i = 0; i < 4; i++)
  {
    whether_voice[13 + i] = Weather_Code[wethear][i];
  }
  Change_Voice_Content(whether_voice, 17, temp);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6180, whether_voice,
                                     sizeof(whether_voice));
  delay(2500);
}
/************************************************************************************************************
 【函 数】:Change_Voice_Content
 【参 数】:array 开数组首地址   location 改变的位置   number_value改变值
 【返 回】:
 【简 例】:Change_Voice_Content(time_voice, 13, 0x31); 0x31读作三十一
 【说 明】:语音播报更改处
*************************************************************************************************************/
void Change_Voice_Content(char *array, uint8_t location, uint8_t number_value)
{
  uint16_t number_data[2] = {0, 0};

  number_data[0] = number_value / 16;
  number_data[1] = number_value % 16;

  if (number_data[1] != 0)
  {
    array[location + 4] = Number_Voice[number_data[1] * 2];
    array[location + 5] = Number_Voice[number_data[1] * 2 + 1];
    if (number_data[0] == 0) // 1-10
    {
      array[location] = 0x01;
      array[location + 1] = 0x01;
      array[location + 2] = 0x01;
      array[location + 3] = 0x01;
    }
    else if (number_data[0] == 1) // 11-19
    {
      array[location] = 0x01;
      array[location + 1] = 0x01;
      array[location + 2] = Number_Voice[10 * 2];
      array[location + 3] = Number_Voice[10 * 2 + 1];
    }
    else // 21-31
    {
      array[location] = Number_Voice[number_data[0] * 2];
      array[location + 1] = Number_Voice[number_data[0] * 2 + 1];
      array[location + 2] = Number_Voice[10 * 2];
      array[location + 3] = Number_Voice[10 * 2 + 1];
    }
  }
  else if (number_data[1] == 0 && number_data[0] != 0)
  {
    if (number_data[0] != 1)
    {
      array[location] = Number_Voice[number_data[0] * 2];
      array[location + 1] = Number_Voice[number_data[0] * 2 + 1];
      array[location + 2] = Number_Voice[10 * 2];
      array[location + 3] = Number_Voice[10 * 2 + 1];
    }
    else
    {
      array[location] = 0x01;
      array[location + 1] = 0x01;
      array[location + 2] = Number_Voice[10 * 2];
      array[location + 3] = Number_Voice[10 * 2 + 1];
    }
    array[location + 4] = 0x01;
    array[location + 5] = 0x01;
  }
  else if (number_data[1] == 0 && number_data[0] == 0)
  {
    array[location] = 0x01;
    array[location + 1] = 0x01;
    array[location + 2] = 0x01;
    array[location + 3] = 0x01;
    array[location + 4] = Number_Voice[0];
    array[location + 5] = Number_Voice[1];
  }
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 烽火台报警标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:Beacon_Infrare_Alarm
 【参 数】:cmd 开启码
 【返 回】:
 【简 例】:Beacon_Infrare_Alarm("A123B4"); 烽火台报警默认开启码
 【说 明】:烽火台开启码，通讯方式红外
*************************************************************************************************************/
void Beacon_Infrare_Alarm(char *cmd)
{
  Infrare.Transmition(cmd, 6); // 烽火台报警默认开启码
}
/************************************************************************************************************
 【函 数】:Beacon_SendQuire
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:请求回传随机救援位置坐标点
*************************************************************************************************************/
void Beacon_SendQuire(void)
{
  Command.Judgment(beacon_sendquire);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, beacon_sendquire, 8);
}
/************************************************************************************************************
 【函 数】:Beacon_Send_Front_Code
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:发送烽火台前三位开启码
*************************************************************************************************************/
void Beacon_Send_Front_Code(uint8_t cmd, uint8_t cmd2, uint8_t cmd3)
{
  beacon_bendcode_buf[2] = 0X10;
  beacon_bendcode_buf[3] = cmd;
  beacon_bendcode_buf[4] = cmd2;
  beacon_bendcode_buf[5] = cmd3;
  Command.Judgment(beacon_bendcode_buf);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, beacon_bendcode_buf, 8);
}
/************************************************************************************************************
 【函 数】:Beacon_Send_Back_Code
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:发送烽火台后三位开启码
*************************************************************************************************************/
void Beacon_Send_Back_Code(uint8_t cmd, uint8_t cmd2, uint8_t cmd3)
{
  beacon_bendcode_buf[2] = 0X11;
  beacon_bendcode_buf[3] = cmd;
  beacon_bendcode_buf[4] = cmd2;
  beacon_bendcode_buf[5] = cmd3;
  Command.Judgment(beacon_bendcode_buf);
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, beacon_bendcode_buf, 8);
}
/************************************************************************************************************
 【函 数】:Beacon_AllCode_Zigbee
 【参 数】:
 【返 回】:
 【简 例】:
 【说 明】:发送全部开启码
*************************************************************************************************************/
void Beacon_AllCode_Zigbee(uint8_t *cmd)
{
  delay(500);
  Beacon_Send_Front_Code(cmd[0], cmd[1], cmd[2]);
  delay(500);
  Beacon_Send_Front_Code(cmd[3], cmd[4], cmd[5]);
  delay(500);
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 智能TFT屏标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:TFT_PictureDisplay
 【参 数】:name 标志物('A'/'B'/'C')
          cmd1:          cmd2：             cmd3：
          0x00           0x01~0x20          0x00   显示指定图片
          0x01           0x00               0x00   图片显示模式向上翻页
          0x02           0x00               0x00   图片显示模式向下翻页
          0x03           0x00               0x00   图片自动翻页模式
 【返 回】:
 【简 例】:TFT_PictureDisplay('A',0X00,0X01,0X00); TFT标志物A显示第一张图片
 【说 明】:发送全部开启码
*************************************************************************************************************/
void TFT_PictureDisplay(char name, uint8_t cmd1, uint8_t cmd2)
{
  switch (name) // 确定发送的TFT标志物是"A" 还是"B",默认为'A'
  {
  case 'A':
    tft_picture_buf[1] = 0x0B;
    break;
  case 'B':
    tft_picture_buf[1] = 0x08;
    break;
  case 'C':
    tft_picture_buf[1] = 0x12;
    break;
  default:
    tft_picture_buf[1] = 0x0B;
    break;
  }

  tft_picture_buf[3] = cmd1;         // 副指令1
  tft_picture_buf[4] = cmd2;         // 副指令2
  Command.Judgment(tft_picture_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, tft_picture_buf, 8);
}
/************************************************************************************************************
 【函 数】:TFT_Licenseplate_Front
 【参 数】:name 标志物('A'/'B'/'C')   cmd1~3 ASCII码数据
 【返 回】:
 【简 例】:
 【说 明】:显示前三位车牌
*************************************************************************************************************/
void TFT_Licenseplate_Front(char name, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3)
{
  switch (name) // 确定发送的TFT标志物是"A" 还是"B",默认为'A'
  {
  case 'A':
    tft_licenseplate_buf[1] = 0x0B;
    break;
  case 'B':
    tft_licenseplate_buf[1] = 0x08;
    break;
  case 'C':
    tft_licenseplate_buf[1] = 0x12;
    break;
  default:
    tft_licenseplate_buf[1] = 0x0B;
    break;
  }

  tft_licenseplate_buf[2] = 0X20; // 主指令赋值
  tft_licenseplate_buf[3] = cmd1; // 显示前三位车牌
  tft_licenseplate_buf[4] = cmd2;
  tft_licenseplate_buf[5] = cmd3;
  Command.Judgment(tft_licenseplate_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, tft_licenseplate_buf, 8);
}
/************************************************************************************************************
 【函 数】:TFT_Licenseplate_Front
 【参 数】:name 标志物('A'/'B'/'C')   cmd1~3 ASCII码数据
 【返 回】:
 【简 例】:
 【说 明】:显示前后位车牌
*************************************************************************************************************/
void TFT_Licenseplate_Back(char name, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3)
{
  switch (name) // 确定发送的TFT标志物是"A" 还是"B",默认为'A'
  {
  case 'A':
    tft_licenseplate_buf[1] = 0x0B;
    break;
  case 'B':
    tft_licenseplate_buf[1] = 0x08;
    break;
  case 'C':
    tft_licenseplate_buf[1] = 0x12;
    break;
  default:
    tft_licenseplate_buf[1] = 0x0B;
    break;
  }

  tft_licenseplate_buf[2] = 0X21; // 主指令赋值
  tft_licenseplate_buf[3] = cmd1; // 显示前三位车牌
  tft_licenseplate_buf[4] = cmd2;
  tft_licenseplate_buf[5] = cmd3;
  Command.Judgment(tft_licenseplate_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, tft_licenseplate_buf, 8);
}
/************************************************************************************************************
 【函 数】:Beacon_AllCode_Zigbee
 【参 数】:name 标志物('A'/'B'/'C')   cmd 车牌数据
 【返 回】:
 【简 例】:TFT_Licenseplate_All('A',License_Plate); TFT标志位A显示车牌
 【说 明】:显示完整车牌
*************************************************************************************************************/
void TFT_Licenseplate_All(char name, uint8_t *cmd)
{
  delay(500);
  TFT_Licenseplate_Front(name, cmd[0], cmd[1], cmd[2]);
  delay(500);
  TFT_Licenseplate_Back(name, cmd[3], cmd[4], cmd[5]);
  delay(500);
}
/************************************************************************************************************
 【函 数】:TFT_TimeDisplay
 【参 数】:name 标志物('A'/'B'/'C')   cmd (0关/1开/2关)
 【返 回】:
 【简 例】:
 【说 明】:计时
*************************************************************************************************************/
void TFT_TimeDisplay(char name, uint8_t cmd)
{
  switch (name) // 确定发送的TFT标志物是"A" 还是"B",默认为'A'
  {
  case 'A':
    tft_time_buf[1] = 0x0B;
    break;
  case 'B':
    tft_time_buf[1] = 0x08;
    break;
  case 'C':
    tft_time_buf[1] = 0x12;
    break;
  default:
    tft_time_buf[1] = 0x0B;
    break;
  }
  tft_time_buf[3] = cmd;
  Command.Judgment(tft_time_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, tft_time_buf, 8);
}
/************************************************************************************************************
 【函 数】:TFT_HEXDisplay
 【参 数】:name 标志物('A'/'B'/'C')   abc 数据位
 【返 回】:
 【简 例】:TFT_HEXDisplay('A',0X12,0X34,0X56); TFT标志物A显示123456
 【说 明】:六位 HEX 格式数据显示
*************************************************************************************************************/
void TFT_HEXDisplay(char name, uint8_t a, uint8_t b, uint8_t c)
{
  switch (name) // 确定发送的TFT标志物是"A" 还是"B",默认为'A'
  {
  case 'A':
    tft_date_buf[1] = 0x0B;
    break;
  case 'B':
    tft_date_buf[1] = 0x08;
    break;
  case 'C':
    tft_date_buf[1] = 0x12;
    break;
  default:
    break;
  }
  // 发送十六进制数
  tft_date_buf[3] = a;
  tft_date_buf[4] = b;
  tft_date_buf[5] = c;
  Command.Judgment(tft_date_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, tft_date_buf, 8);
}
/************************************************************************************************************
 【函 数】:Beacon_AllCode_Zigbee
 【参 数】:name 标志物('A'/'B'/'C')
 【返 回】:
 【简 例】:
 【说 明】:显示距离
*************************************************************************************************************/
void TFT_Distance_Display(char name)
{
  switch (name) // 确定发送的TFT标志物是"A" 还是"B",默认为'A'
  {
  case 'A':
    tft_distance_buf[1] = 0x0B;
    break;
  case 'B':
    tft_distance_buf[1] = 0x08;
    break;
  case 'C':
    tft_distance_buf[1] = 0x12;
    break;
  default:
    tft_distance_buf[1] = 0x0B;
    break;
  }
  Ultrasonic_Ranging(); // 测距
  delay(500);
  tft_distance_buf[4] = Ultrasonic_Distance[0]; // 百位
  tft_distance_buf[5] =
      Ultrasonic_Distance[1] * 16 + Ultrasonic_Distance[2]; // 十个

  Command.Judgment(tft_distance_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, tft_distance_buf, 8);
}
/************************************************************************************************************
 【函 数】:TFT_DisplayTraffic
 【参 数】:name 标志物('A'/'B'/'C')
          cmd 0x01：直行 0x02：左转    0x03：右转
              0x04：掉头 0x05：禁止直行 0x06：禁止通行
 【返 回】:
 【简 例】:
 【说 明】:发送全部开启码
*************************************************************************************************************/
void TFT_DisplayTraffic(char name, uint8_t cmd)
{
  switch (name) // 确定发送的TFT标志物是"A" 还是"B",默认为'A'
  {
  case 'A':
    tft_traffic_buf[1] = 0x0B;
    break;
  case 'B':
    tft_traffic_buf[1] = 0x08;
    break;
  case 'C':
    tft_traffic_buf[1] = 0x12;
    break;
  default:
    break;
  }

  tft_traffic_buf[3] = cmd;
  Command.Judgment(tft_traffic_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, tft_traffic_buf, 8);
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 智能路灯标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数】:Streetlight_FindLevel
 【参 数】:记录各个档位的数组
 【返 回】:
 【简 例】:
 【说 明】:返回前路灯最小档位下标
*************************************************************************************************************/
uint8_t Streetlight_FindLevel(void)
{
  int led_level[4];
  uint8_t min_level = 0;

  for (uint8_t i = 0; i < 4; i++)
  {
    led_level[i] = BH1750.ReadLightLevel(true);  // 获取当前智能路灯档位
    Infrare.Transmition(Command.HW_Dimming1, 4); // 将路灯档位+1
    delay(1000);
    if (led_level[i] < led_level[min_level])
    {
      min_level = i;
    }
  }
  // 返回当前档位及初始档位
  switch (min_level)
  {
  case 0:
    return 1;
    break;
  case 1:
    return 4;
    break;
  case 2:
    return 3;
    break;
  case 3:
    return 2;
    break;
  default:
    break;
  }
}
/************************************************************************************************************
 【函 数】:Streetlight_SetLevel
 【参 数】:led_x 挡位1~4
 【返 回】:
 【简 例】:Streetlight_SetLevel(1); 设置路灯档位为1
 【说 明】:设置智能路灯档位
*************************************************************************************************************/
void Streetlight_SetLevel(int led_x)
{
  Light_Level[0] = Streetlight_FindLevel(); // 获取当前level档位
  Light_Level[1] = led_x;                   // 需要设置档位
  // 调节到所需设置档位
  switch (Light_Level[1] - Light_Level[0])
  {
  case -3:
    Infrare.Transmition(Command.HW_Dimming1, 4);
    break;
  case -2:
    Infrare.Transmition(Command.HW_Dimming2, 4);
    break;
  case -1:
    Infrare.Transmition(Command.HW_Dimming3, 4);
    break;
  case 0:
    break;
  case 1:
    Infrare.Transmition(Command.HW_Dimming1, 4);
    break;
  case 2:
    Infrare.Transmition(Command.HW_Dimming2, 4);
    break;
  case 3:
    Infrare.Transmition(Command.HW_Dimming3, 4);
    break;
  }
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 老版无线充电控制 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
【函 数 名】：	Wireless_Control	            控制无线充电装置
【参数说明】：	cmd:  0x01：开启无线充电       0x02：关闭无线充电
【返 回 值】：	无返回
【简    例】：  Wireless_Control(1);          开启无线充电功能
************************************************************************************************************/
void Wireless_Control(uint8_t cmd)
{
  wireless_buf[3] = cmd;
  Command.Judgment(wireless_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, wireless_buf, 8);
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 新版本无线充电控制 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
【函 数 名】：	Wireless_Open	                              开启无线充电装置
【参数说明】：	cmd: 开启控制码
【返 回 值】：	无返回
【简    例】：  Wireless_Open();                            开启无线充电功能
************************************************************************************************************/
void Wireless_Open(uint8_t *cmd)
{
  wireless_buf_5_2[2] = 0X02;
  wireless_buf_5_2[3] = cmd[0];
  wireless_buf_5_2[4] = cmd[1];
  wireless_buf_5_2[5] = cmd[2];
  Command.Judgment(wireless_buf_5_2); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, wireless_buf_5_2, 8);
}
/************************************************************************************************************
【函 数 名】：	Wireless_Change 更改无线充电装置开启码 【参数说明】：	cmd:
开启控制码 【返 回 值】：	无返回 【简    例】：  Wireless_Change();
更改无线充电装置开启码
************************************************************************************************************/
void Wireless_Change(uint8_t *cmd)
{
  wireless_buf_5_2[2] = 0x03;
  wireless_buf_5_2[3] = cmd[0];
  wireless_buf_5_2[4] = cmd[1];
  wireless_buf_5_2[5] = cmd[2];
  Command.Judgment(wireless_buf_5_2); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, wireless_buf_5_2, 8);
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ ETC系统标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
【函 数 名】：	ETC_Control	                   ETC控制
【参数说明】：	cmd:                           cmd1
               0x01：左侧闸门上升              0x01：右侧闸门上升
               0x02：左侧闸门下降              0x02：右侧闸门下降
【返 回 值】：	无返回
【简    例】：  ETC_Control(2,2);              左侧闸门下降，右侧闸门下降
************************************************************************************************************/
void ETC_Control(uint8_t cmd, uint8_t cmd1)
{
  etc_buf[3] = cmd;
  etc_buf[4] = cmd1;
  Command.Judgment(etc_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, etc_buf, 8);
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 智能交通灯标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
 【函 数 名】：	Traffic_Distinguish 交通灯进入识别模式 【参数说明】：
 flag：A：识别智能交通路灯A                          B：识别智能交通路灯B
               flag：C：识别智能交通路灯C D：识别智能交通路灯D 【返 回 值】：
 无返回 【简    例】： Traffic_Distinguish（'A'） 交通灯A进入识别模式
 ************************************************************************************************************/
void Traffic_Distinguish(char flag)
{
  uint8_t reutrn_traffic[8];
  switch (flag)
  {
  case 'A':
    traffic_buf[1] = 0X0E;
    break;
  case 'B':
    traffic_buf[1] = 0X0F;
    break;
  case 'C':
    traffic_buf[1] = 0X13;
    break;
  case 'D':
    traffic_buf[1] = 0X14;
    break;
  default:
    break;
  }
  traffic_buf[2] = 0x01;
  traffic_buf[3] = 0x00;
  delay(200);
  Command.Judgment(traffic_buf);                              // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, traffic_buf, 8); // 发送进入识别命令
}
/************************************************************************************************************
 【函 数 名】：	Traffic_ResponseResult 识别结果请求确认 【参数说明】：
 flag：A：识别智能交通路灯A                             B：识别智能交通路灯B
               flag：C：识别智能交通路灯C D：识别智能交通路灯D 【返 回 值】：
 无返回 【简    例】： Traffic_ResponseResult（'A'） 交通路灯A识别结果请求确认
 ************************************************************************************************************/
void Traffic_ResponseResult(char flag)
{
  switch (flag)
  {
  case 'A':
    traffic_result_buf[1] = 0x0E;
    break;
  case 'B':
    traffic_result_buf[1] = 0x0F;
    break;
  case 'C':
    traffic_result_buf[1] = 0x13;
    break;
  case 'D':
    traffic_result_buf[1] = 0x14;
    break;
  default:
    break;
  }

  traffic_result_buf[2] = 0x02;
  switch (traffic_status)
  {
  case 1:
    traffic_result_buf[3] = 0x01;
    break; // “红色”识别结果请求确认
  case 2:
    traffic_result_buf[3] = 0x02;
    break; // “绿色”识别结果请求确认
  case 3:
    traffic_result_buf[3] = 0x03;
    break; // “黄色”识别结果请求确认
  default:
    break;
  }
  Command.Judgment(traffic_result_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, traffic_result_buf, 8);
  delay(500);
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 特殊地形标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
【函 数 名】：	Special_Send_Check 发送查询车辆通行状态 【参数说明】：	无参数
【返 回 值】：	无返回
【简    例】：  Special_Send_Check(); 发送查询车辆通行状态
************************************************************************************************************/
void Special_Send_Check(void)
{
  Command.Judgment(special_terrain); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, special_terrain, 8);
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 立体显示标志物 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
【函 数 名】：	RotationLED_PlateAndCoord 旋转LED显示车牌和坐标 【参数说明】：
plate：六位车牌,                                     coord:信息坐标 【返 回
值】：	无返回 【简    例】：  RotationLED_PlateAndCoord();
旋转LED显示车牌和坐标
************************************************************************************************************/
void RotationLED_PlateAndCoord(uint8_t plate[6], uint8_t coord[2])
{
  infare_rotationled_buf[1] = 0x20;
  infare_rotationled_buf[2] = plate[0];
  infare_rotationled_buf[3] = plate[1];
  infare_rotationled_buf[4] = plate[2];
  infare_rotationled_buf[5] = plate[3];

  Infrare.Transmition(infare_rotationled_buf, 6); // 发送前三位车牌

  delay(600);

  infare_rotationled_buf[1] = 0x10;
  infare_rotationled_buf[2] = plate[4];
  infare_rotationled_buf[3] = plate[5];
  infare_rotationled_buf[4] = coord[0];
  infare_rotationled_buf[5] = coord[1];
  Infrare.Transmition(infare_rotationled_buf, 6);
  delay(600);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Distance 旋转LED显示距离 【参数说明】：
cmd：十位距离                                      cmd：个位距离 【返 回 值】：
无返回 【简    例】：  RotationLED_Distance(0x11); 旋转LED显示11
************************************************************************************************************/
void RotationLED_Distance(uint8_t cmd)
{
  infare_rotationled_buf[1] = 0x11;
  infare_rotationled_buf[2] = (cmd / 16) + 0x30; // 十位距离
  infare_rotationled_buf[3] = (cmd % 16) + 0x30; // 个位距离
  infare_rotationled_buf[4] = 0x00;
  infare_rotationled_buf[5] = 0x00;
  Infrare.Transmition(infare_rotationled_buf, 6);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Form 旋转LED显示距 【参数说明】：	cmd： 0x01：矩形
0x02：圆形 0x03：三角形                                           0x04：菱形
               0x05：五角星
【返 回 值】：	无返回
【简    例】：  RotationLED_Form(1); 旋转LED显示矩形
************************************************************************************************************/
void RotationLED_Form(uint8_t cmd)
{
  infare_rotationled_buf[1] = 0x12;
  infare_rotationled_buf[2] = cmd; // 形状
  infare_rotationled_buf[3] = 0x00;
  infare_rotationled_buf[4] = 0x00;
  infare_rotationled_buf[5] = 0x00;
  Infrare.Transmition(infare_rotationled_buf, 6);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Color 旋转显示颜色 【参数说明】：	cmd： 颜色
               0x01：红色                                            0x02：绿色
               0x03：蓝色                                            0x04：黄色
               0x05：品色                                            0x06：青色
               0x07：黑色                                            0x08：白色
【返 回 值】：	无返回
【简    例】：  RotationLED_Color(1); 旋转显示红色
************************************************************************************************************/
void RotationLED_Color(uint8_t cmd)
{
  infare_rotationled_buf[1] = 0x13;
  infare_rotationled_buf[2] = cmd; // 形状
  infare_rotationled_buf[3] = 0x00;
  infare_rotationled_buf[4] = 0x00;
  infare_rotationled_buf[5] = 0x00;
  Infrare.Transmition(infare_rotationled_buf, 6);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Information	                        显示字体
【参数说明】：	cmd：
              0x01：前方学校 减速慢行                          0x02：前方施工 禁止通行
              0x03：塌方路段 注意安全                          0x04：追尾危险 保持车距
              0x05：严禁 酒后驾车！                            0x06：严禁 乱扔垃圾！
【返 回 值】：	无返回
【简    例】：  RotationLED_Information(1);                     旋转显示前方学校 减速慢行
************************************************************************************************************/
void RotationLED_Information(uint8_t cmd)
{
  infare_rotationled_buf[1] = 0x14;
  infare_rotationled_buf[2] = cmd; // 形状
  infare_rotationled_buf[3] = 0x00;
  infare_rotationled_buf[4] = 0x00;
  infare_rotationled_buf[5] = 0x00;
  Infrare.Transmition(infare_rotationled_buf, 6);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Traffic	                        显示文字
【参数说明】：	cmd： cmd
               0x01：直行                                   0x02：左转
               0x03：右转                                   0x04：掉头
               0x05：禁止直行                               0x06：禁止通行
【返 回 值】：	无返回
【简    例】：  RotationLED_Traffic(1);                     旋转显示直行
************************************************************************************************************/
void RotationLED_Traffic(uint8_t cmd)
{
  infare_rotationled_buf[1] = 0x15;
  infare_rotationled_buf[2] = cmd; // 形状
  infare_rotationled_buf[3] = 0x00;
  infare_rotationled_buf[4] = 0x00;
  infare_rotationled_buf[5] = 0x00;
  Infrare.Transmition(infare_rotationled_buf, 6);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Default	                        显示默认信息
【参数说明】：	无参数
【返 回 值】：	无返回
【简    例】：  RotationLED_Default();                      显示默认信息
************************************************************************************************************/
void RotationLED_Default(void)
{
  infare_rotationled_buf[1] = 0x16;
  infare_rotationled_buf[2] = 0x01; // 形状
  infare_rotationled_buf[3] = 0x00;
  infare_rotationled_buf[4] = 0x00;
  infare_rotationled_buf[5] = 0x00;
  Infrare.Transmition(infare_rotationled_buf, 6);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Set_Color	                             显示RGB颜色
【参数说明】：	cmd:R                      cmd1:G                  cmd2:B
【返 回 值】：	无返回
【简    例】：  RotationLED_Set_Color(0xff,0xff,0xff);             显示白色
************************************************************************************************************/
void RotationLED_Set_Color(uint8_t cmd, uint8_t cmd1, uint8_t cmd2)
{
  infare_rotationled_buf[1] = 0x17;
  infare_rotationled_buf[2] = 0x01;
  infare_rotationled_buf[3] = cmd;
  infare_rotationled_buf[4] = cmd1;
  infare_rotationled_buf[5] = cmd2;
  Infrare.Transmition(infare_rotationled_buf, 6);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Increase_TXT 累计添加显示文本 【参数说明】：
cmd和cmd1为汉字编码                                 cmd2： 0x00:表示合成字未结束
0x55:合成字结束 【返 回 值】：	无返回 【简    例】：
RotationLED_Increase_TXT(,);                       显示
************************************************************************************************************/
void RotationLED_Increase_TXT(uint8_t cmd, uint8_t cmd1, uint8_t cmd2)
{
  infare_rotationled_buf[1] = 0x31;
  infare_rotationled_buf[2] = cmd;
  infare_rotationled_buf[3] = cmd1;
  infare_rotationled_buf[4] = cmd2;
  infare_rotationled_buf[5] = 0x00;
  Infrare.Transmition(infare_rotationled_buf, 6);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Display 累计添加显示文本 【参数说明】：
cmd：GBK汉字编码 【返 回 值】：	无返回 【简    例】：
RotationLED_Display("你好李晨杨");                         显示你好李晨杨
************************************************************************************************************/
void RotationLED_Display(uint8_t *cmd)
{
  uint8_t lenth = strlen(cmd) / 2, i;
  for (i = 0; i < (lenth - 1); i++)
  {
    RotationLED_Increase_TXT(cmd[i * 2], cmd[i * 2 + 1], 0x00);
    delay(100);
  }
  RotationLED_Increase_TXT(cmd[i * 2], cmd[i * 2 + 1], 0x55);
  delay(100);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Clear_TXT 结束累加文本 【参数说明】：	cmd：
1：结束累加不清除,现有显示内容                  cmd：
2：结束累加并清除现有显示内容 【返 回 值】：	无返回 【简    例】：
RotationLED_Clear_TXT(1);                           结束累加不清除,现有显示内容
************************************************************************************************************/
void RotationLED_Clear_TXT(uint8_t cmd)
{
  infare_rotationled_buf[1] = 0x32;
  infare_rotationled_buf[2] = cmd;
  infare_rotationled_buf[3] = 0x00;
  infare_rotationled_buf[4] = 0x00;
  infare_rotationled_buf[5] = 0x00;
  Infrare.Transmition(infare_rotationled_buf, 6);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_Incresae_WordZigbee 累计添加显示文本 【参数说明】：
word,word1 :汉字文本GBK编码 cmd：0x55结束文字累加，0x00没有结束文字累加 【返 回
值】：	无返回 【简    例】：  RotationLED_Incresae_WordZigbee(1);
结束累加不清除,现有显示内容
************************************************************************************************************/
void RotationLED_Incresae_WordZigbee(uint8_t word, uint8_t word1, uint8_t cmd)
{
  rotationled_zigbee_buf[2] = 0x31;
  rotationled_zigbee_buf[3] = word;
  rotationled_zigbee_buf[4] = word1;
  rotationled_zigbee_buf[5] = cmd;
  Command.Judgment(rotationled_zigbee_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, rotationled_zigbee_buf, 8);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_DisplayWords 显示汉字 【参数说明】：	cmd 【返
回 值】：	无返回 【简    例】：  RotationLED_DisplayWords(1);
结束累加不清除,现有显示内容
************************************************************************************************************/
void RotationLED_DisplayWords(uint8_t *cmd)
{
  uint8_t lenth = strlen(cmd) / 2, i;
  for (i = 0; i < (lenth - 1); i++)
  {
    RotationLED_Incresae_WordZigbee(cmd[i * 2], cmd[i * 2 + 1], 0x00);
    delay(100);
  }
  RotationLED_Incresae_WordZigbee(cmd[i * 2], cmd[i * 2 + 1], 0x55);
  delay(100);
}
/************************************************************************************************************
【函 数 名】：	RotationLED_ClearWord 累计添加显示文本 【参数说明】：	cmd：
1：结束累加不清除,现有显示内容                    cmd：
2：结束累加并清除现有显示内容 【返 回 值】：	无返回 【简    例】：
RotationLED_ClearWord(1); 结束累加不清除,现有显示内容
************************************************************************************************************/
void RotationLED_ClearWord(uint8_t cmd)
{
  rotationled_zigbee_buf[2] = 0x32;
  rotationled_zigbee_buf[3] = cmd;
  rotationled_zigbee_buf[4] = 0x00;
  rotationled_zigbee_buf[5] = 0x00;
  Command.Judgment(rotationled_zigbee_buf); // 计算校验和
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, rotationled_zigbee_buf, 8);
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 小创语音识别 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
/************************************************************************************************************
【函 数 名】：	Speech_Disc 语音标志物播报识别 【参数说明】：
cmd:0为随机播放识别                                      1：指定播报识别 【返 回
值】：	无返回 【简    例】：  Speech_Disc(0); 随机播报识别
************************************************************************************************************/
void Speech_Disc(uint8_t cmd)
{
  uint8_t sph_id = 0;
  sph_id = BKRC_Voice.BKRC_Voice_Extern(cmd); // 0为控制语音播报标志物随机播报并开启识别，
                                              // 1-7为指定词条识别，具体信息可查看通信协议
  delay(200);
  Serial.print("识别到的词条ID为：");
  Serial.println(sph_id, HEX);
  trm_buf[2] = sph_id;
  ExtSRAMInterface.ExMem_Write_Bytes(0x6008, trm_buf, 8); // 上传识别结果至评分终端
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 字符转换 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
// 以二进制打印数据
void Content_Print_OCT(uint8_t *array, uint8_t lenth)
{
  for (int i = 0; i < lenth; i++)
  {
    Serial.print("0x");
    Serial.print(array[i], OCT); // 以八进制打印
    Serial.print("  ");
  }
  Serial.println(); // 数据换行
}
// 以八进制打印数据
void Content_Print_BIN(uint8_t *array, uint8_t lenth)
{
  for (int i = 0; i < lenth; i++)
  {
    Serial.print("0x");
    Serial.print(array[i], HEX); // 以二进制打印
    Serial.print("  ");
  }
  Serial.println(); // 数据换行
}
// 以十进制打印数据
void Content_Print_DEC(uint8_t *array, uint8_t lenth)
{
  for (int i = 0; i < lenth; i++)
  {
    Serial.print("0x");
    Serial.print(array[i], DEC); // 以十六进制打印
    Serial.print("  ");
  }
  Serial.println(); // 数据换行
}
// 以十六进制打印数据
void Content_Print_HEX(uint8_t *array, uint8_t lenth)
{
  for (int i = 0; i < lenth; i++)
  {
    Serial.print("0x");
    Serial.print(array[i], HEX); // 以十六进制打印
    Serial.print("  ");
  }
  Serial.println(); // 数据换行
}
/************************************************************************************************************
【函 数 名】：	Clear_Array_Date 清空数组 【参数说明】：	array:数组首地址
lenth数组长度 【返 回 值】：	无返回 【简    例】：
Clear_Array_Date(Define_Zero48,48)                        清空数组Define_Zero48
************************************************************************************************************/
void Clear_Array_Date(uint8_t *array, uint8_t lenth)
{
  for (int i = 0; i < lenth; i++)
  {
    array[i] = 0;
  }
}

// * ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ 二维码处理 ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
void QR_Detal_Task1(void)
{
  uint8_t k = 0;
  uint8_t biaodashi[100];
  uint8_t flag_detal_shift = 0; // 字符串切换处理
  uint8_t Lenth_Expression = 0;
  for (uint8_t pa = 0; pa < QRcode_Length[0];
       pa++) // 假设二维码1为表达式，计算表达式的长度
  {
    if ((QRcode_Array[0][pa] >= '0' && QRcode_Array[0][pa] <= '9') || QRcode_Array[0][pa] == '(' || QRcode_Array[0][pa] == ')' || QRcode_Array[0][pa] == '+' || QRcode_Array[0][pa] == '-' || QRcode_Array[0][pa] == '*' || QRcode_Array[0][pa] == '/' || QRcode_Array[0][pa] == '^')
    {
      Lenth_Expression += 1;
    }
  }
  // 如果长度相等则表明表达式即为二维码一
  if (Lenth_Expression == QRcode_Length[0])
  {
    flag_detal_shift = 1;
  }
  // 二维一为表达式,二维码二为车牌
  if (flag_detal_shift == 1)
  {
    // 复制表达式
    for (int p = 0; p < QRcode_Length[0]; p++)
    {
      biaodashi[p] = QRcode_Array[0][p];
    }
    // 提取车牌或者道闸
    for (int pa = 0; pa < QRcode_Length[1]; pa++) // 读取车牌
    {
      if ((QRcode_Array[1][pa] <= 'Z' && QRcode_Array[1][pa] >= 'A') || (QRcode_Array[1][pa] <= '9' && QRcode_Array[1][pa] >= '0'))
      {
        QR_Length6[k++] = QRcode_Array[1][pa];
        Serial.print(QRcode_Array[1][pa]);
      }
    }
  }
  // 二维一为车牌,二维码二为表达式
  else
  {
    // 提取车牌或者道闸
    for (int pa = 0; pa < QRcode_Length[0]; pa++) // 读取车牌
    {
      if ((QRcode_Array[0][pa] <= 'Z' && QRcode_Array[0][pa] >= 'A') || (QRcode_Array[0][pa] <= '9' && QRcode_Array[0][pa] >= '0'))
      {
        QR_Length6[k++] = QRcode_Array[0][pa]; // 将二维码信息赋值给QR_Length6数组(道闸开启码或车牌)
        Serial.print(QRcode_Array[0][pa]);
      }
    }

    // 复制表达式
    for (int p = 0; p < QRcode_Length[1]; p++)
    {
      biaodashi[p] = QRcode_Array[1][p];
    }
  }
  // 发送车牌至主车
  Sent_MultiDate_ToMainCar(ToMainCar_CarPlate, QR_Length6, 6); // 发送车牌给主车
  delay(500);
  // 将正则表达式发送至主车
  Sent_QR_ToMainCar(QRcode_Length[1], biaodashi);
  delay(500);
}
void QR_Detal_1(void)
{
  int k = 0;
  uint8_t biaodashi[100];
  if (QRcode_Length[0] == 15) // 八位长度为车牌
  {
    for (int pa = 0; pa < QRcode_Length[0]; pa++) // 读取车牌
    {
      if ((QRcode_Array[0][pa] <= 'Z' && QRcode_Array[0][pa] >= 'A') || (QRcode_Array[0][pa] <= '9' && QRcode_Array[0][pa] >= '0'))
      {
        QR_Length6[k++] = QRcode_Array[0][pa]; // 将二维码信息赋值给QR_Length6数组
        Serial.println(QRcode_Array[0][pa]);
      }
    }
    for (int p = 0; p < QRcode_Length[1]; p++) // 将表达式发给主车
    {
      biaodashi[p] = QRcode_Array[1][p];
    }
    Sent_QR_ToMainCar(QRcode_Length[1], biaodashi); // 将正则表达式发送至主车
  }
  else
  {
    for (int pa = 0; pa < QRcode_Length[1]; pa++) // 读取车牌
    {
      if ((QRcode_Array[1][pa] <= 'Z' && QRcode_Array[1][pa] >= 'A') || (QRcode_Array[1][pa] <= '9' && QRcode_Array[1][pa] >= '0'))
      {
        QR_Length6[k++] = QRcode_Array[1][pa]; // 将二维码信息赋值给QR_Length6数组
        Serial.println(QRcode_Array[1][pa]);
      }
    }
    for (int p = 0; p < QRcode_Length[0]; p++)
    {
      biaodashi[p] = QRcode_Array[0][p];
    }
    Sent_QR_ToMainCar(QRcode_Length[0], biaodashi); // 将正则表达式发送至主车
  }
  Content_Print_HEX(QR_Length6, 6);
  delay(500);
  Sent_MultiDate_ToMainCar(ToMainCar_CarPlate, QR_Length6, 6); // 发送车牌给主车
  delay(500);
}
void WIFI_qr(void)
{
  unsigned char *string1;
  string1 = getCharge(QRcode_Array[0], QRcode_Length[0], QRcode_Array[1],
                      QRcode_Length[1]);
  WIFI_start[0] = string1[3];
  WIFI_start[1] = string1[4];
  WIFI_start[2] = string1[5];
  Content_Print_HEX(WIFI_start, 3);
}
unsigned char *getCharge(char *chars1, int length1, char *chars2, int length2)
{

  int *pLen1 = &length1;
  int *pLen2 = &length2;

  char *string = getArray(chars1, pLen1);
  char *string2 = getArray(chars2, pLen2);
  char *result = compare(string, *pLen1, string2);

  unsigned char *chars = malloc(sizeof(unsigned char) * 8);
  chars[0] = 0x55;
  chars[1] = 0x0A;
  chars[2] = 0x02;
  chars[6] = 0x49;
  chars[7] = 0xBB;
  for (int i = 3, j = 0; i < 3 + 3; ++i, j += 2)
  {
    chars[i] = result[j] * 16 + result[j + 1];
  }
  return chars;
}
char *getArray(char *chars, int *length)
{
  int start = 0;
  int end = 0;
  for (int i = 0; i < *length; ++i)
  {
    if (chars[i] == '<')
    {
      start = i + 1;
    }
    else if (chars[i] == '>')
    {
      end = i;
    }
  }
  *length = end - start;
  char *currentString = (char *)malloc(sizeof(char) * (end - start));

  for (int i = 0, j = start; i < end - start; ++i, j++)
  {
    currentString[i] = chars[j];
  }
  return currentString;
}
char *compare(char *ch1, int length, char *ch2)
{
  char *bytes = malloc(sizeof(char) * 6);
  for (int i = 0, j = 0; i < length; ++i)
  {
    if (ch1[i] != ch2[i])
    {
      bytes[j] = ch1[i];
      bytes[j + 3] = ch2[i];
      j++;
    }
  }
  for (int i = 0; i < 6; ++i)
  {
    if (bytes[i] >= '0' && bytes[i] <= '9')
    {
      bytes[i] -= '0';
    }
    else if (bytes[i] >= 'A' && bytes[i] <= 'Z')
    {
      bytes[i] = bytes[i] + 10 - 'A';
    }
    else if (bytes[i] >= 'a' && bytes[i] <= 'z')
    {
      bytes[i] = bytes[i] + 10 - 'a';
    }
  }
  return bytes;
}
