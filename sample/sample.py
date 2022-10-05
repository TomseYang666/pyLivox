import pylvxreader

filepath = r'./data/162651_004/162658_004_LiDAR0.lvx'  # 这里替换成你的数据路径
point_list, imu_list = pylvxreader.lvx_read(filepath)
