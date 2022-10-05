import pylvxreader
import matplotlib.pyplot as plt
import numpy as np

filepath = 'data/162757_066_LiDAR0.lvx'
point_list, imu_list = pylvxreader.lvx_read(filepath)
length = len(imu_list)
example = [0, 0, 0, 0, 0, 0]
num = 0
for i in range(length):
    if imu_list[i] == [0, 0, 0, 0, 0, 0]:
        num += 1
for i in range(num):
    imu_list.remove(example)
length = len(imu_list)
x = np.zeros(length)
y = np.zeros(length)
z = np.zeros(length)
exam=np.zeros(length)
gyro_x= np.zeros(length)
gyro_y= np.zeros(length)
gyro_z= np.zeros(length)
t = np.zeros(length)
for i in range(length):
    gyro_x[i] = imu_list[i][0]
    gyro_y[i] = imu_list[i][1]
    gyro_z[i] = imu_list[i][2]
    x[i] = imu_list[i][3]
    y[i] = imu_list[i][4]
    z[i] = imu_list[i][5]
    exam[i]=-1
    t[i] = i
fig1, ax1 = plt.subplots()
ax1.plot(t, x, 'b.', label='accx')
ax1.plot(t, y, 'g.', label='accy')
ax1.plot(t, z, 'r.', label='accz')
ax1.plot(t, exam, 'k--')
ax1.set(xlabel='index', ylabel='acc (g)',
        title='Acc')
ax1.grid()
ax1.legend()
fig1.savefig("test.png")
plt.show()

fig2, ax2 = plt.subplots()
ax2.plot(t, gyro_x, 'b.', label='gyro_x')
ax2.plot(t, gyro_y, 'g.', label='gyro_y')
ax2.plot(t, gyro_z, 'r.', label='gyro_z')

ax2.set(xlabel='index', ylabel='gyro (deg/s)',
        title='Gyro')
ax2.grid()
ax2.legend()
fig2.savefig("test1.png")
plt.show()
