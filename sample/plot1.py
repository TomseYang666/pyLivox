import pyimureader
import matplotlib.pyplot as plt
import numpy as np

filepath = 'data/162750.imu'
point_list = pyimureader.imu_read(filepath)
length = len(point_list)
example = [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]]
num = 0
index = []
for i in range(length):
    if point_list[i][0] == [0, 0, 0, 0, 0, 0, 0]:
        num += 1
        index.append(i)
for i in range(num):
    point_list.remove(example)
length = len(point_list)
x = np.zeros(length)
y = np.zeros(length)
z = np.zeros(length)
t = np.zeros(length)
for i in range(length):
    x[i] = point_list[i][1][3]
    y[i] = point_list[i][1][4]
    z[i] = point_list[i][1][5]
    t[i] = i
fig1, ax1 = plt.subplots()
ax1.plot(t, x, 'b-', label='accx')
ax1.plot(t, y, 'g-', label='accy')
ax1.plot(t, -z, 'r-', label='accz')
ax1.set(xlabel='index', ylabel='acc (g)',
        title='Acc')
ax1.grid()
ax1.legend()
fig1.savefig("test.png")
plt.show()
