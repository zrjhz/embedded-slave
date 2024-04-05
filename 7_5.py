from machine import Timer, PWM, UART, Timer
import sensor, image, time, lcd
from fpioa_manager import fm
from Maix import GPIO
import KPU as kpu
import utime
import binascii
import sys
import math
import os
import gc, sys

print(os.listdir("/sd"))

# ---------------------全局变量--------------------------------
"""
    识别交通灯方式为：先定点识别交通灯红绿灯相对位置,
                   后面在真正识别灯相对于红绿灯那个近的位置
"""

# 交通灯
traffic_array = [0x55, 0x02, 0x91, 0x04, 0x00, 0x00, 0x00, 0xbb]  # 存放红绿灯识别返回Arduino
Traffic_Flag = False  # 交通灯识别标志位

# 定时器计时
Timer_number = 0

# 二维码
QR_Distinguish = [0x55, 0x02, 0x92, 0x02, 0x00, 0x00, 0x00, 0xbb]  # 二维码返回识别结果
QR_Flag = False  # 二维码识别开启关闭标志位
QR_Sendflag = False  # QR发送标志位
QR_Mode = 0

# 二维码模式一(定时扫描)
QR_result = {""}  # 定义储存QR集合

# 二维码模式二(张数扫描,并按从左到右,从上往下顺序返回)
QR_two_result = []  # 储存二维码的空列表
QR_X = []  # 储存二维码的X
QR_Y = []  # 储存二维码的Y
QR_Number = 0  # 扫描二维码的个数

# 二维码模式三(指定颜色扫描)
QR_three_result = []  # 储存二维码的空列表
QR_Color = 0  # 扫描二维码的颜色

# 循迹
Flag_track = False  # 循迹识别开启标志位
is_debug = True  # Flase:取消画矩形框图,True:画矩形框图
is_need_send_data = False  # 循迹信号发送标志物
Center_Black = 0
bols = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  # 八路循迹值
bols_b = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  # 八路循迹值

# 自动阈值匹配扫描
Black_Blos = 0  # 黑色阈值基础
Whilt_Blos = 0  # 白色阈值基础
Whilt_Juge = [(255, 45)]  # 白色块阈值
Black_Juge = [(0, 42)]  # 黑色块阈值

# 映射串口引脚
fm.register(6, fm.fpioa.UART1_RX, force=True)
fm.register(7, fm.fpioa.UART1_TX, force=True)

# 初始化串口
uart = UART(UART.UART1, 115200, read_buf_len=4096)

# 交通灯模型识别
model_addr = "/sd/traffic.kmodel"
labels = ["green", "red", "yellow"]
anchors = [0.96875, 4.5625, 0.875, 2.9375, 0.75, 2.40625, 0.6875, 4.0625, 0.59375, 2.28125]


# 识别直线的摄像头参数
def Sensor_Track():
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_brightness(-2)
    sensor.set_contrast(0)
    sensor.set_saturation(-2)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_gain(True)
    sensor.set_vflip(True)
    sensor.set_hmirror(True)
    lcd.rotation(0)


# 识别交通灯的摄像头参数
def Sensor_Traffic():
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_brightness(-2)
    sensor.set_contrast(-2)
    sensor.set_saturation(-2)
    sensor.set_auto_whitebal(True)
    sensor.set_auto_gain(False)
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    lcd.rotation(2)


# 识别特殊地形的摄像头参数
def Sensor_Space():
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_brightness(0)
    sensor.set_contrast(2)
    sensor.set_saturation(2)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_gain(True)
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    lcd.rotation(2)


# 识别二维码模式一二
def Sensor_QR_OTMode():
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_brightness(-1)
    sensor.set_contrast(2)
    sensor.set_saturation(-1)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_gain(True)
    sensor.set_hmirror(True)
    sensor.set_vflip(True)
    lcd.rotation(0)


# 识别二维码模式三(颜色识别)
def Sensor_QR_RGBMode():
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_brightness(-2)
    sensor.set_contrast(2)
    sensor.set_saturation(2)
    sensor.set_auto_whitebal(False)
    sensor.set_auto_gain(True)
    sensor.set_hmirror(True)
    sensor.set_vflip(True)
    lcd.rotation(0)


lcd.init(freq=20000000)  # 初始化LCD

# ---------------------摄像头初始化--------------------------------
# 摄像头初始化
sensor.reset()  # 复位和初始化摄像头，执行sensor.run(0)停止。
sensor.set_vflip(1)  # 将摄像头设置成后置方式（所见即所得）
# Sensor_Track()                     #循迹模式
# Sensor_Traffic()                   #交通灯识别模式
Sensor_QR_RGBMode()
# Sensor_QR_OTMode()                 #QR识别模式
# Sensor_QR_RGBMode()                 #指定颜色识别
sensor.set_framesize(sensor.QVGA)  # 设置帧大小为 QVGA (320x240)
sensor.skip_frames(time=1000)  # 等待设置生效


# --------------定时器部分 START -------------------
def uart_time_trigger(tim):
    # 全局变量调用
    global is_need_send_data, QR_Flag, Timer_number, QR_Sendflag, Traffic_Flag
    # 检测是否是 二维码识别(10S) 红绿灯识别(6s)
    if QR_Flag or Traffic_Flag:
        Timer_number += 1
    if Timer_number >= 120 and Traffic_Flag:  # 交通灯超时检测 7秒计时
        traffic_return(2)  # 发送一个绿色识别结果
        sensor.set_pixformat(sensor.GRAYSCALE)
        Traffic_Flag = False  # 识别结束标志位
        Timer_number = 0
        tim1.stop()
    elif Timer_number >= 200 and QR_Flag:  # 二维码15s识别时间
        QR_Sendflag = True  # 发送开始标志
        QR_Flag = False  # 识别结束标志位
        Timer_number = 0
        tim1.stop()
    is_need_send_data = True


tim1 = Timer(Timer.TIMER1, Timer.CHANNEL1, mode=Timer.MODE_PERIODIC, period=50, callback=uart_time_trigger)


# --------------定时器部分 END --------------------


# --------------舵机配置  START -------------------

def Servo(angle):
    tim_pwm = Timer(Timer.TIMER0, Timer.CHANNEL0, mode=Timer.MODE_PWM)
    S1 = PWM(tim_pwm, freq=50, duty=0, pin=17)  # 频率freq:50HZ  占空比duty:0
    S1.duty((angle + 90) / 180 * 10 + 2.5)


Servo(5)  # 初始化45朝下,为了后面摄像头初始获取阈值
time.sleep(1)


# --------------舵机配置  END -------------------


# --------------串口UART部分  START -------------------

# 舵机角度'+','-';以及左右电机'+','-'判断
def get_symbol(num):
    if num >= 0:
        return ord('+')
    else:
        return ord('-')


# 循迹反馈回传Arduino
def data_format_wrapper(space_flag, deflection, angle, track_value):
    global is_debug

    send_data = [0x55, 0x02, 0x91, space_flag, deflection, angle, track_value, 0xbb]

    if is_debug:
        print(send_data)
    return bytes(send_data)


# 串口信息发送
def UsartSend(str_data):
    uart.write(str_data)


# --------------串口UART部分 END -------------------

# --------------二维码识别  START -------------------

# 返回二维码识别信号
def QR_ReturnResoult(buf):
    QR_Distinguish[3] = buf
    uart.write(bytes(QR_Distinguish))


# 二维码信息发送Arduino
def QR_Send(string, array_number):
    uart.write(bytes([0x55]))
    uart.write(bytes([0x02]))
    uart.write(bytes([0x92]))
    uart.write(bytes([0x01]))
    uart.write(bytes([len(string) + 1]))
    print("长度：{}".format(len(string) + 1))
    for qrdata in string:
        uart.write(bytes([qrdata]))
    uart.write(bytes([array_number]))
    uart.write(bytes([0xbb]))


# 信息切割发送
def sp(str1):
    if (len(str1) > 14):  # 回调切割
        print(str1[0:14])
        QR_Send(str1[0:14], 0x99)
        time.sleep(0.2)
        str2 = str1[14:]
        sp(str2)
    else:
        print(str1)
        QR_Send(str1, 0x88)
        time.sleep(0.4)


# 二维码识别
def Scan_QR_ModeOne(img):
    global QR_result
    imge = img.copy()  # 赋值图片
    res = imge.find_qrcodes()  # 寻找二维码
    if len(res) > 0:
        print(res)
        for k in res:
            QR_result.add(k.payload())  # 赋值到集合中
            print(k.payload())


def Send_QR_ModeOne():
    global QR_result, QR_Sendflag, Timer_number
    QR_result.remove("")  # 移除空字符串
    print(QR_result)
    list1 = list(QR_result)  # 将集合转为列表
    for s in list1:  # 切割加发送
        s = s.encode('gbk')
        sp(s)
    QR_result.clear()  # 清除集合
    QR_result.add("")
    QR_ReturnResoult(0x02)
    Sensor_Track()  # 将模式转换为巡线模式
    QR_Sendflag = False  # 发送模式关闭
    Timer_number = 0


def Scan_QR_ModeTwo(img):
    global QR_two_result, QR_X, QR_Y, QR_Number, QR_Sendflag, QR_Flag
    imge = img.copy()
    res = imge.find_qrcodes()  # 寻找二维码
    if len(res) > 0:
        print(res)
        for k in res:
            if k.payload() in QR_two_result:
                continue
            else:
                QR_two_result.append(k.payload())
                QR_X.append(k.x())
                QR_Y.append(k.y())
                if (len(QR_two_result) == QR_Number):
                    QR_Flag = False
                    QR_Sendflag = True
                    tim1.stop()


def Send_QR_ModeTwo():
    global QR_Sendflag, QR_result, QR_X, QR_Y, QR_two_result, QR_Number
    X_Copy = QR_X[:]
    Y_Copy = QR_Y[:]
    if (QR_Number == 2 and len(QR_two_result) == QR_Number):
        sp(QR_two_result[Y_Copy.index(min(Y_Copy))].encode('gbk'))  # 先发送左边的
        sp(QR_two_result[Y_Copy.index(max(Y_Copy))].encode('gbk'))  # 在发送右边的
    elif (QR_Number == 3 and len(QR_two_result) == QR_Number):
        sp(QR_two_result[X_Copy.index(max(X_Copy))].encode('gbk'))  # 先发送上面的
        sp(QR_two_result[Y_Copy.index(min(Y_Copy))].encode('gbk'))  # 在发送左下
        sp(QR_two_result[Y_Copy.index(max(Y_Copy))].encode('gbk'))  # 在发送右下
    elif (QR_Number == 4 and len(QR_two_result) == QR_Number):
        array_left = sorted(Y_Copy)[0:2]
        array_right = sorted(Y_Copy)[2:4]
        print(array_left, array_right)
        left_a = Y_Copy.index(array_left[0])
        left_b = Y_Copy.index(array_left[1])
        right_a = Y_Copy.index(array_right[0])
        right_b = Y_Copy.index(array_right[1])

        print(QR_two_result)
        print(left_a, left_b, right_a, right_b)
        if (X_Copy[left_a] > X_Copy[left_b]):
            sp(QR_two_result[left_a].encode('gbk'))  # 先发左上
            sp(QR_two_result[left_b].encode('gbk'))  # 后发左下
        else:
            sp(QR_two_result[left_b].encode('gbk'))  # 先发左上
            sp(QR_two_result[left_a].encode('gbk'))  # 后发左下
        if (X_Copy[right_a] > X_Copy[right_b]):
            sp(QR_two_result[right_a].encode('gbk'))  # 先发左上
            sp(QR_two_result[right_b].encode('gbk'))  # 后发左下
        else:
            sp(QR_two_result[right_b].encode('gbk'))  # 先发左上
            sp(QR_two_result[right_a].encode('gbk'))  # 后发左下
    else:  # 其他个数直接发送
        for s in QR_two_result:  # 切割加发送
            s = s.encode('gbk')
            sp(s)
    QR_ReturnResoult(0x02)  # 发送识别结束回传Arduino
    QR_Number = 0  # 清空数据
    QR_two_result.clear()
    QR_X.clear()
    QR_Y.clear()
    QR_Sendflag = False  # 发送模式关闭
    Sensor_Track()


QR_Color = 5
def Scan_QR_ModeThree(imge):
    global QR_three_result, QR_Color, QR_Sendflag, QR_Flag
    img = imge.copy()
    img_resize = img.resize(224, 224)
    objects = FindCode(img)
    if objects:
        for obj in objects:
            overlap = 0
            pos = obj.rect()
            if QR_Color == 5:
                single_code = CropImage(img_resize, pos)
                overlap = OverlapModel(single_code)
                if overlap == 1:
                    QR_three_result.append(obj.classid())
                    QR_Flag = False
                    QR_Sendflag = True
                    tim1.stop()
                img_resize.draw_rectangle(pos)
                img_resize.draw_string(pos[0], pos[1], "%s" %(overlap), scale=2, color=(255, 0, 0))
                lcd.display(img_resize)
                break
            else:
                pos = [pos[0] / 0.7, pos[1] / 0.9, pos[2] / 0.7, pos[3] / 0.9]
                pos = [int(x) for x in pos]
                pos = [pos[0] - 20, pos[1] - 20, pos[2] + 30, pos[3] + 30]
                single_code = CropImage(img, pos)
                res = single_code.find_qrcodes()
                color_rlt = labels3[obj.classid()]
                if res:
                    if res[0].payload() in QR_three_result:
                        continue
                    else:
                        print("%s : %s" % (color_rlt, res[0].payload()))
                        if obj.classid() == QR_Color:
                            QR_three_result.append(res[0].payload())
                            QR_Flag = False
                            QR_Sendflag = True
                            tim1.stop()
                img.draw_rectangle(pos)
                img.draw_string(pos[0], pos[1], "%s : %.2f" %(color_rlt, obj.value()), scale=2, color=(255, 0, 0))
                lcd.display(img)


def Send_QR_ModeThree():
    global QR_three_result, QR_Color, QR_Sendflag
    if (len(QR_three_result) != 0):
        sp(QR_three_result[0].encode('gbk'))  # 发送识别颜色的二维码内容
    QR_ReturnResoult(0x02)  # 发送识别结束回传Arduino
    QR_Color = 0  # 清空数据
    QR_three_result.clear()
    QR_Sendflag = False  # 发送模式关闭
    Sensor_Track()


labels4 = ["normal", "overlap"]
def OverlapModel(img):
    task4 = kpu.load("/sd/overlap.kmodel")
    #img = imge.copy()
    img = img.resize(224, 224)   # 使用图像处理函数处理原图像
    img.pix_to_ai()              # 同步 `RGB888` 内存块
    fmap = kpu.forward(task4, img)

    plist=fmap[:]
    print(plist)
    pmax=max(plist)
    max_index=plist.index(pmax)
    kpu.deinit(task4)
    return max_index


labels3 = ["black", "blue", "green", "red", "yellow"]
anchors3 = [0.3125, 0.5625, 0.625, 1.125, 0.65625, 1.125, 0.65625, 1.125, 1.25, 2.0625]
def FindCode(imge):
    task3 = kpu.load("/sd/qrcode.kmodel")
    kpu.init_yolo2(task3, 0.5, 0.3, 5, anchors3)
    try:
        img_copy = imge.copy()
        img = img_copy.resize(224, 224)
        img.pix_to_ai()
        objects = kpu.run_yolo2(task3, img)
        kpu.deinit(task3)
        return(objects)
    except Exception as e:
        raise e



def CropImage(img, pos):
    img_copy = img.copy()
    return img_copy.crop(pos)


# --------------二维码  END -------------------

# --------------交通灯识别 START --------------------

# 回传Arduino信息
def traffic_return(buf):
    traffic_array[4] = buf
    uart.write(bytes(traffic_array))


def Traffic_Model():
    gc.collect()
    task = kpu.load(model_addr)
    kpu.init_yolo2(task, 0.9, 0.4, 5, anchors)  # threshold:[0,1], nms_value: [0, 1]
    Index_Array = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    Color_Array = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    Color_Number = [0, 0, 0]
    for i in range(20):
        imge = sensor.snapshot()
        imge = imge.resize(224, 224)  # 使用图像处理函数处理原图像
        imge.pix_to_ai()  # 同步 `RGB888` 内存块
        t = time.ticks_ms()
        objects = kpu.run_yolo2(task, imge)
        t = time.ticks_ms() - t
        if objects:
            for obj in objects:
                pos = obj.rect()
                print(pos)
                imge.draw_rectangle(pos)
                print(labels[obj.classid()], obj.value())
                imge.draw_string(pos[0], pos[1], "%s : %.2f" % (labels[obj.classid()], obj.value()), scale=2,
                                 color=(255, 0, 0))
                if (labels[obj.classid()] == 'red'):
                    Color_Number[0] += 1
                elif (labels[obj.classid()] == 'yellow'):
                    Color_Number[2] += 1
                else:
                    Color_Number[1] += 1
                Index_Array[i] = pos[0]
                Color_Array[i] = labels[obj.classid()]
        imge.draw_string(0, 200, "t:%dms" % (t), scale=2, color=(255, 0, 0))
        lcd.display(imge)
    kpu.deinit(task)
    del task
    if (Color_Number[0] == 0 and Color_Number[1] == 0 and Color_Number[2] == 0):
        return 3
    else:
        return Color_Number.index(max(Color_Number))
    # gc.collect()
    # print(gc.mem_free() / 1024) # stack mem
    # print(gc.mem_free() / 1024) # stack mem


def Space_Moddel():
    Sensor_Space()
    task2 = kpu.load("/sd/space.kmodel")
    typepic = ['other', 'space']
    Type_Map = [0, 0]
    for i in range(5):
        imge = sensor.snapshot()
        imge = imge.resize(224, 224)  # 使用图像处理函数处理原图像
        imge.pix_to_ai()
        t = time.ticks_ms()
        fmap = kpu.forward(task2, imge)
        t = time.ticks_ms() - t
        plist = fmap[:]
        pmax = max(plist)
        max_index = plist.index(pmax)
        if (max_index == 0):
            Type_Map[0] += 1
        else:
            Type_Map[1] += 1
        img.draw_string(0, 0, "%.2f : %s" % (pmax, typepic[max_index].strip()), scale=2, color=(255, 0, 0))
        img.draw_string(0, 200, "t:%dms" % (t), scale=2, color=(255, 0, 0))
        lcd.display(img)
    print(typepic[max_index])
    kpu.deinit(task2)
    Sensor_Track()
    del task2
    if (Type_Map[0] > Type_Map[1]):  # 1为特殊地形，0为非特殊地形
        return 0
    else:
        return 1


# --------------交通灯识别 END --------------------

# --------------直线检测部分 START -------------------
Angle_ROIS = {
    'up': (60, 0, 10, 240),  # 横向取样-上方
    'middel_up': (40, 0, 10, 240),  # 横向取样-上方
    'middel_down': (20, 0, 10, 240),  # 横向取样-上方
    'down': (0, 0, 10, 240),  # 横向取样-下方
}


def Cumulate_Line_Angle():
    imge = sensor.snapshot()
    roi_blobs_result = {}  # 在各个ROI中寻找色块的结果记录
    deflection_angle = 0
    for roi_direct in Angle_ROIS.keys():
        roi_blobs_result[roi_direct] = {'cx': 0, 'cy': 0, 'w': 0, 'blob_flag': False, 'area': 0}
    for roi_direct, roi in Angle_ROIS.items():  # 找黑色区块
        blobs = imge.find_blobs(Black_Juge, roi=roi)
        if len(blobs) != 0:
            largest_blob = max(blobs, key=lambda b: b.pixels())
            if largest_blob.area() > 100:
                x, y, width, height = largest_blob[:4]
                roi_blobs_result[roi_direct]['cx'] = largest_blob.cy()  # 中心y
                roi_blobs_result[roi_direct]['cy'] = largest_blob.cx()  # 中心x
                roi_blobs_result[roi_direct]['w'] = largest_blob.h()  # 黑线宽度，w黑线长度
                roi_blobs_result[roi_direct]['blob_flag'] = True
                roi_blobs_result[roi_direct]['area'] = largest_blob.area()
                if is_debug:
                    img.draw_rectangle((x, y, width, height), color=(255))

    if roi_blobs_result['down']['blob_flag']:
        if roi_blobs_result['up']['blob_flag'] and roi_blobs_result['up']['w'] < 80:
            a = math.atan2((roi_blobs_result['up']['cy'] - roi_blobs_result['down']['cy']), \
                           (roi_blobs_result['up']['cx'] - roi_blobs_result['down']['cx']))
            deflection_angle = (int)(a / math.pi * 180 + 0.5)
        elif roi_blobs_result['middel_up']['blob_flag'] and roi_blobs_result['middel_up']['w'] < 80:
            a = math.atan2((roi_blobs_result['middel_up']['cy'] - roi_blobs_result['down']['cy']), \
                           (roi_blobs_result['middel_up']['cx'] - roi_blobs_result['down']['cx']))
            deflection_angle = (int)(a / math.pi * 180 + 0.5)
        elif roi_blobs_result['middel_down']['blob_flag'] and roi_blobs_result['middel_down']['w'] < 80:
            a = math.atan2((roi_blobs_result['middel_down']['cy'] - roi_blobs_result['down']['cy']), \
                           (roi_blobs_result['middel_down']['cx'] - roi_blobs_result['down']['cx']))
            deflection_angle = (int)(a / math.pi * 180 + 0.5)
        else:
            print("defualt1")
            deflection_angle = 90
    else:
        print("defualt2")
        deflection_angle = 90
    print(deflection_angle)
    return deflection_angle


# 返回特殊地形识别结果
angle_buf = [0x55, 0x02, 0x91, 0x00, 0x00, 0x00, 0x00, 0xBB]


def Space_Result(buf):
    angle_buf[3] = buf
    uart.write(bytes(angle_buf))


def stat_return(img):
    imge = img.copy()  # 图片copy
    angle = 0
    track_value = 0  # 循迹值
    track_value_b = 0
    '''  阈值判断    '''
    for i in range(8):
        mean = imge.get_statistics(roi=(0, 32 * i, 20, 16))  # 平均阈值
        bols[i] = mean.min()  # 最低阈值
    min_values = sorted(bols)[:3]  # 选择最低三个阈值
    max_value = max(bols)  # 找出最大阈值
    min_value = min(bols)  # 找出最小阈值
    avrage_number = (sum(bols) - min_values[0] - min_values[1] - min_values[2]) / 5  # 取平均阈值
    if (min_value <= (Black_Blos + 20)):  # 小于90表示有黑色直线
        for k in range(8):
            if ((bols[k] - min_values[0]) < 20):
                track_value = (track_value << 1)
            else:
                track_value = (track_value << 1) + 1
    else:  # 没有小于90表示存在
        track_value = 0xff

    for j in range(8):
        mean_b = imge.get_statistics(roi=(35, 32 * j, 20, 16))  # 平均阈值
        bols_b[j] = mean_b.min()  # 最低阈值
    min_values_b = sorted(bols_b)[:3]  # 选择最低三个阈值
    max_value_b = max(bols)  # 找出最大阈值
    min_value_b = min(bols)  # 找出最小阈值
    avrage_number_b = (sum(bols) - min_values_b[0] - min_values_b[1] - min_values_b[2]) / 5  # 取平均阈值
    if (min_value_b <= (Black_Blos + 20)):  # 小于90表示有黑色直线
        for k in range(8):
            if ((bols_b[k] - min_values_b[0]) < 20):
                track_value_b = (track_value_b << 1)
            else:
                track_value_b = (track_value_b << 1) + 1
    else:  # 没有小于90表示存在
        track_value_b = 0xff

    '''  黑白判断    '''
    whilt_flag = 0
    black_cy = 0
    black_h = 0
    track_angle = 0
    black_center = 0
    left_black = 0
    right_black = 0
    whilt_w = 0

    whilt = imge.find_blobs(Whilt_Juge, roi=[0, 0, 30, 240])  # 白边识别
    if len(whilt) != 0:
        largest_blob = max(whilt, key=lambda b: b.pixels(), merge=True, color_margin=10)
        if largest_blob.area() > 200:
            x, y, width, height = largest_blob[:4]
            whilt_w = largest_blob.h()  # 黑线宽度，w黑线长度
            if is_debug:
                print("whilt：", whilt_w)
                img.draw_rectangle((x, y, width, height), color=(255))
    black = imge.find_blobs(Black_Juge, roi=[0, 0, 80, 240])  # 黑色中心识别

    if len(black) != 0:
        largest_blob = max(black, key=lambda b: b.pixels())
        if largest_blob.area() > 900:
            x, y, width, height = largest_blob[:4]
            black_cy = largest_blob.cy()
            black_h = largest_blob.h()
            left_black = imge.get_pixel(0, (int)(black_cy + 1 - black_h / 2))
            right_black = imge.get_pixel(0, (int)(black_cy - 1 + black_h / 2))

            # if (left_black>right_black) and  ((black_h-58)>=0) and ((black_h-58)<=80):
            # angle =(int)(180-(int)(math.atan2(80, black_h-58)*180/math.pi))
            # elif (left_black<right_black) and  ((black_h-58)>=0) and ((black_h-58)<=80):
            # angle =(int)(math.atan2(80, black_h-58)*180/math.pi)
            # print("angle:",angle)
            # print((largest_blob.h()+largest_blob.y()),largest_blob.cy()+32)
            black_center = (int)(black_cy)  # 黑线宽度，w黑线长度

            if is_debug:
                img.draw_rectangle((x, y, width, height), color=(255))

    if whilt_w > 220:  # and Jugement_Value(track_value)==6:#and:
        whilt_flag = 1

    print(whilt_w, whilt_flag, bin(track_value_b), bin(track_value), black_center)
    return whilt_flag, track_value_b, black_center, track_value


# 此函数用于初始化获取基础阈值
def Scan_Bols(img):
    global Black_Blos, Whilt_Blos
    imge = img.copy()
    max_imge = imge.get_statistics(roi=(0, 0, 320, 240))
    bols_min = [0, 0, 0, 0, 0, 0, 0, 0]
    track_value = 0  # 循迹值
    for i in range(8):
        mean = imge.get_statistics(roi=(0, 32 * i, 30, 16))  # 平均阈值
        bols_min[i] = mean.min()  # 最低阈值
    Black_Blos += min(bols_min)
    Whilt_Blos += max(bols_min)
    print("bols_min:", bols_min)
    print("max_imge:", max_imge.mean())


# 初始化获取阈值
for i in range(10):
    img = sensor.snapshot()
    Scan_Bols(img)

Black_Blos = Black_Blos / 10
Whilt_Blos = Whilt_Blos / 10


def Jugement_Value(track):
    array_track = [0, 0, 0, 0, 0, 0, 0, 0]
    change_number = 0
    for i in range(8):
        if ((track % 2) == 1):
            array_track[i] = 1
        else:
            array_track[i] = 0
    if (i != 0 and (array_track[i - 1] != array_track[i])):
        change_number += 1
    track = track // 2

    print(change_number * 2 + array_track[0] + 1, array_track)
    return change_number * 2 + array_track[0] + 1;


# --------------直线检测部分 End -------------------

# --------------直线与路口检测部分 END -------------------
while True:
    #Scan_QR_ModeThree(img)
    # Cumulate_Line_Angle()
    # OverlapModel(img)
    lcd.display(img)                # 在LCD上显示
    if (uart.any()):
        data = uart.read(8)
        print(binascii.hexlify(data).decode('utf_8'))
        if (len(data) >= 8):
            if ((data[0] == 0x55) & (data[1] == 0x02) & (data[7] == 0xBB)):
                if (data[2] == 0x91):
                    if (data[3] == 0x01):  # 启动循迹
                        Flag_track = True
                        tim1.start()
                    elif (data[3] == 0x02):  # 关闭循迹
                        Flag_track = False
                        Timer_number = 0
                        tim1.stop()
                    elif (data[3] == 0x03):  # 舵机调
                        angle = data[5]
                        if data[4] == ord('-'):
                            if angle > 80:
                                angle = 80
                            angle = -angle
                        elif data[4] == ord('+'):
                            if angle > 20:
                                angle = 20
                            angle = angle
                        Servo(angle)
                    elif (data[3] == 0x04):  # 交通灯
                        tim1.start()
                        Sensor_Traffic()
                        Timer_number = 0
                        Traffic_Flag = True
                    elif (data[3] == 0x05):  # 特殊地形
                        a = Space_Moddel()
                        Space_Result(a)

                elif (data[2] == 0x92):  # 二维码
                    if (data[3] == 0x01):
                        QR_Mode = data[4]  # 获取二维码扫描模式
                        if (data[4] == 0x01):  # 扫描模式一：扫描十秒钟结束
                            Sensor_QR_OTMode()  # 无差别扫描
                        elif (data[4] == 0x02):
                            Sensor_QR_OTMode()  # 扫描指定位置
                            QR_Number = data[5]
                        elif (data[4] == 0x03):
                            Sensor_QR_RGBMode()  # 扫描指定位置
                            QR_Color = data[5]  # 扫描指定颜色
                        tim1.stop()
                        Timer_number = 0
                        tim1.start()
                        QR_Flag = True
                    elif (data[3] == 0x02):
                        QR_Flag = False
                        print("关闭二维码识别")
    img = sensor.snapshot()
    if Traffic_Flag:
        Index = Traffic_Model()
        Traffic_Flag = False
        traffic_return(Index + 1)
        Sensor_Track()
        tim1.stop()  # 关闭定时器
    if QR_Flag:
        if QR_Mode == 0x01:
            Scan_QR_ModeOne(img)
        elif QR_Mode == 0x02:
            Scan_QR_ModeTwo(img)
        elif QR_Mode == 0x03:
            Scan_QR_ModeThree(img)
    if QR_Sendflag:
        tim1.stop()
        if QR_Mode == 0x01:
            Send_QR_ModeOne()
        elif QR_Mode == 0x02:
            Send_QR_ModeTwo()
        elif QR_Mode == 0x03:
            Send_QR_ModeThree()
        Sensor_Track()

    if Flag_track:  # 循迹
        space_flag, cross_flag, deflection_angle, track_value = stat_return(img)
        if is_need_send_data:  # 发送循迹数据
            UsartSend(data_format_wrapper(space_flag, cross_flag, deflection_angle, track_value))
            is_need_send_data = False
