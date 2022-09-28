# ----------------------------------------------------------------------------------------------------------------------
# !/usr/bin/env python
# -*- coding: UTF-8 -*-
import struct

"""
IMU同步数据格式，为16进制数据，格式如下：
0x57 0x48(2bytes) + Len(byte) + DataType(byte) + UTC(9bytes) + IMU data (24bytes) + GPS_lock(1byte)
+  ODO_numb (4byte) + CheckSum(1byte)
1)	“0x57 0x48”： 帧头0x57、0x48；
2)	Len：数据包长度43，包括0x57 0x48和校验，为0x2B。
3)	DataType：数据类型 0x03。
4)	UTC时间：Year(byte) + Month(byte) + Day(byte) + Hour(byte) + Minute(byte) + Second(byte) + Millionsecond(3byte,Unit:us)。
5)	IMU data：惯性测量设备数据包括以下内容
    (1)	X轴陀螺仪(4bytes)：高位在前，下同
    (2)	Y轴陀螺仪(4bytes)：
    (3)	Z轴陀螺仪(4bytes)：
    (4)	X轴加速度(4bytes)：
    (5)	Y轴加速度(4bytes)：
    (6)	Z轴加速度(4bytes)：
6)	GPS_lock数据：0x01为GPS收星数大于3颗，0x00为GPS收星数小于3颗。。
7)	ODO_numb (4byte) ：编码器的值。（编码器的初始值为7fffffff，正转加1，反转减1）。
8)	CheckSum：前面所有数据按字节求异或得到的校验码。
"""

# ----------------------------------------------------------------------------------------------------------------------
# 规定读取文件的格式，用于按格式读取文件
imu_header = '4c 508x'
imu_info = '4x 33c 7x'


# ----------------------------------------------------------------------------------------------------------------------
# 主函数：读取IMU文件
def imu_read(filepath):
    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
    # 读取文件头
    f = open(filepath, 'rb')
    print('start to read file!')
    data = f.read(struct.calcsize(imu_header))  # 512bytes
    print('start to read the header!')
    print('%d bytes' % struct.calcsize(imu_header))
    print(data)
    print('__________')
    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
    # 读取IMU数据帧部分
    point_list = []
    idx = 0
    print('start to read the info!')
    print('%d bytes' % struct.calcsize(imu_info))
    while True:
        data = f.read(struct.calcsize(imu_info))
        print(data)
        if len(data) != 44:
            break
        data_tuple = struct.unpack(imu_info, data)
        print(data_tuple)
        '''
        Year(byte) + Month(byte) + Day(byte) + Hour(byte) + Minute(byte) + Second(byte) + Millionsecond(3byte,Unit:us)。
        IMU data：惯性测量设备数据包括以下内容
        (1)	X轴陀螺仪(4bytes)：高位在前，下同
        (2)	Y轴陀螺仪(4bytes)：
        (3)	Z轴陀螺仪(4bytes)：
        (4)	X轴加速度(4bytes)：
        (5)	Y轴加速度(4bytes)：
        (6)	Z轴加速度(4bytes)：
        '''
        time = [
            int.from_bytes(data_tuple[0], byteorder='big'),
            int.from_bytes(data_tuple[1], byteorder='big'),
            int.from_bytes(data_tuple[2], byteorder='big'),
            int.from_bytes(data_tuple[3], byteorder='big'),
            int.from_bytes(data_tuple[4], byteorder='big'),
            int.from_bytes(data_tuple[5], byteorder='big'),
            int.from_bytes(data_tuple[6] + data_tuple[7] + data_tuple[8], byteorder='big')
        ]
        print('time: ', time)
        imu = [
            (0.00699411 / 65536) * struct.unpack('i', data_tuple[12] + data_tuple[11] + data_tuple[10] + data_tuple[9])[0],
            (0.00699411 / 65536) * struct.unpack('i', data_tuple[16] + data_tuple[15] + data_tuple[14] + data_tuple[13])[0],
            (0.00699411 / 65536) * struct.unpack('i', data_tuple[20] + data_tuple[19] + data_tuple[18] + data_tuple[17])[0],
            (0.2 / 65536) * struct.unpack('i', data_tuple[24] + data_tuple[23] + data_tuple[22] + data_tuple[21])[0],
            # 单位mg
            (0.2 / 65536) * struct.unpack('i', data_tuple[28] + data_tuple[27] + data_tuple[26] + data_tuple[25])[0],
            (0.2 / 65536) * struct.unpack('i', data_tuple[32] + data_tuple[31] + data_tuple[30] + data_tuple[29])[0]
        ]
        print('imu: ', imu)
        point = [time, imu]
        point_list.append(point)

        if f.read(struct.calcsize(imu_info)) == b'':
            break

    print('__________')
    total_len = len(point_list)
    print('共采集到：', total_len, '条完整数据')
    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
    # 关闭文件
    f.close()
    return point_list

