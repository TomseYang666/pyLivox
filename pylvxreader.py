# ----------------------------------------------------------------------------------------------------------------------
# -*- coding: UTF-8 -*-
import struct
import math


# ----------------------------------------------------------------------------------------------------------------------
# 函数1：笛卡尔坐标与球面坐标互换
def dpt_to_xyz(depth, theta, phi):
    x = depth * math.sin((theta / 100) * (math.pi / 180)) * math.cos((phi / 100) * (math.pi / 180))
    y = depth * math.sin((theta / 100) * (math.pi / 180)) * math.sin((phi / 100) * (math.pi / 180))
    z = depth * math.cos((theta / 100) * (math.pi / 180))
    return x, y, z


# ----------------------------------------------------------------------------------------------------------------------
# lvx文件头的数据类型定义
LvxFilePublicHeader = '<16s 4c I'  # 公共文件头
LvxFilePrivateHeader = '<IB'  # 私有文件头
LvxDeviceInfo = '<16s 16s 3B 6f'  # 设备信息
FrameHeader = '<3q'  # 构架头
LvxBasePackHeader = '<5B I 2B 8s'  # 数据包头
# 点数据类型定义
Point0 = '<3i B'
Point1 = '<i 2H B'
Point2 = '<3i 2B'
Point3 = '<i 2H 2B'
Point4 = '<3i 2B 3i 2B'
Point5 = '<2H i 2B i 2B'
Point6 = '<6f'


# ----------------------------------------------------------------------------------------------------------------------
# 主函数：读取lvx文件
def lvx_read(filepath):
    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
    # 获得文件总长与文件数据
    f = open(filepath, 'rb')
    data = f.read()
    total_len = len(data)
    print(f'total_len:{total_len}')
    f.close()

    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
    # 读取lvx文件头
    f = open(filepath, 'rb')

    data = f.read(struct.calcsize(LvxFilePublicHeader))  # 24
    print("size LvxFilePublicHeader: ", struct.calcsize(LvxFilePublicHeader))
    FILE_SIGNATURE, VERSION_A, VERSION_B, VERSION_C, VERSION_D, MAGIC_CODE = struct.unpack(LvxFilePublicHeader, data)
    VERSION_A = int.from_bytes(VERSION_A, "little")
    VERSION_B = int.from_bytes(VERSION_B, "little")
    VERSION_C = int.from_bytes(VERSION_C, "little")
    VERSION_D = int.from_bytes(VERSION_D, "little")
    print(
        f"signature:{FILE_SIGNATURE}, version:{VERSION_A}_{VERSION_B}_{VERSION_C}_{VERSION_D}, magic code:{MAGIC_CODE}")

    data = f.read(struct.calcsize(LvxFilePrivateHeader))  # 24+5
    print("size LvxFilePrivateHeader: ", struct.calcsize(LvxFilePrivateHeader))
    dFRAME_DURATION, DEVICE_COUNT = struct.unpack(LvxFilePrivateHeader, data)
    print(f"Device:{DEVICE_COUNT}")

    for i in range(DEVICE_COUNT):
        data = f.read(struct.calcsize(LvxDeviceInfo))  # 24+5+59=88
        print("size LvxDeviceInfo: ", struct.calcsize(LvxDeviceInfo))
        LIDAR_SN, HUB_SN, DEVICE_IDX, DEVICE_TYPE, EXTRINSIC_ENABLE, ROLL, PITCH, YAW, X, Y, Z = struct.unpack(
            LvxDeviceInfo, data)
        print(i, LIDAR_SN, HUB_SN, DEVICE_IDX, DEVICE_TYPE, EXTRINSIC_ENABLE, ROLL, PITCH, YAW, X, Y, Z)

    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
    # 读取点云数据部分:
    point_list = []
    imu_list = []
    frame_points = []
    num_point = [0, 0, 0, 0, 0, 0, 0]
    idx = 0
    print(struct.calcsize(LvxFilePublicHeader) + struct.calcsize(LvxFilePrivateHeader) + DEVICE_COUNT * struct.calcsize(
        LvxDeviceInfo))
    current_offset = struct.calcsize(LvxFilePublicHeader) + struct.calcsize(
        LvxFilePrivateHeader) + DEVICE_COUNT * struct.calcsize(
        LvxDeviceInfo)

    while current_offset < total_len:
        current_frame_offset = 0
        data = f.read(struct.calcsize(FrameHeader))
        if len(data) != 24:  # 避免部分文件存在的空值
            break

        current_offset_file, next_offset, frame_index = struct.unpack(FrameHeader, data)

        if current_offset_file != current_offset:  # 防止可能的数据录入错误
            print(f'waring: current_offset_file: {current_offset_file}, current_offset: {current_offset}')

        current_frame_offset += struct.calcsize(FrameHeader)
        idx += 1

        while current_offset + current_frame_offset < next_offset:
            data = f.read(struct.calcsize(LvxBasePackHeader))
            if len(data) != 19:  # 避免部分文件存在的空值
                break

            device_idx, version, slot_id, lidar_id, reserved, status_code, timestamp_type, data_type, timestamp \
                = struct.unpack(LvxBasePackHeader, data)
            current_frame_offset += struct.calcsize(LvxBasePackHeader)

            # 对不同类型的数据进行判读
            if data_type == 0:
                for _ in range(100):
                    data = f.read(struct.calcsize(Point0))
                    if len(data) != struct.calcsize(Point0):
                        point_list.extend(frame_points)
                        break
                    x, y, z, r = struct.unpack(Point0, data)
                    point_list.append([x, y, z])
                    current_frame_offset += struct.calcsize(Point0)
                    num_point[0] += 1

            elif data_type == 1:
                for _ in range(100):
                    data = f.read(struct.calcsize(Point1))
                    if len(data) != struct.calcsize(Point1):
                        point_list.extend(frame_points)
                        break
                    depth, theta, phi, r = struct.unpack(Point1, data)
                    x, y, z = dpt_to_xyz(depth, theta, phi)
                    point_list.append([x, y, z])
                    current_frame_offset += struct.calcsize(Point1)
                    num_point[1] += 1

            elif data_type == 2:
                for _ in range(96):
                    data = f.read(struct.calcsize(Point2))
                    if len(data) != struct.calcsize(Point2):
                        point_list.extend(frame_points)
                        break
                    x, y, z, r, tag = struct.unpack(Point2, data)
                    point_list.append([x, y, z])
                    current_frame_offset += struct.calcsize(Point2)
                    num_point[2] += 1

            elif data_type == 3:
                for _ in range(96):
                    data = f.read(struct.calcsize(Point3))
                    if len(data) != struct.calcsize(Point3):
                        point_list.extend(frame_points)
                        break
                    depth, theta, phi, r, tag = struct.unpack(Point3, data)
                    x, y, z = dpt_to_xyz(depth, theta, phi)
                    point_list.append([x, y, z])
                    current_frame_offset += struct.calcsize(Point3)
                    num_point[3] += 1

            elif data_type == 4:
                for _ in range(48):
                    data = f.read(struct.calcsize(Point4))
                    if len(data) != struct.calcsize(Point4):
                        point_list.extend(frame_points)
                        break
                    x1, y1, z1, r1, tag1, x2, y2, z2, r2, tag2 = struct.unpack(Point4, data)
                    point_list.append([x1, y1, z1])
                    point_list.append([x2, y2, z2])
                    current_frame_offset += struct.calcsize(Point4)
                    num_point[4] += 1

            elif data_type == 5:
                for _ in range(48):
                    data = f.read(struct.calcsize(Point5))
                    if len(data) != struct.calcsize(Point5):
                        point_list.extend(frame_points)
                        break
                    theta, phi, depth1, r1, tag1, depth2, r2, tag2 = struct.unpack(Point5, data)
                    x1, y1, z1 = dpt_to_xyz(depth1, theta, phi)
                    x2, y2, z2 = dpt_to_xyz(depth2, theta, phi)
                    point_list.append([x1, y1, z1])
                    point_list.append([x2, y2, z2])
                    current_frame_offset += struct.calcsize(Point5)
                    num_point[5] += 1

            elif data_type == 6:
                data = f.read(struct.calcsize(Point6))
                if len(data) != struct.calcsize(Point6):
                    point_list.extend(frame_points)
                    break
                gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z = struct.unpack(Point6, data)
                imu_list.append([gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z])
                current_frame_offset += struct.calcsize(Point6)
                num_point[6] += 1
            else:
                print('Unknown: ', data_type)
        current_offset += current_frame_offset
    f.close()
    # -*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
    # 返回所需数据
    print(num_point)
    return point_list, imu_list
