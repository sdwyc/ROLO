import numpy as np
import math
# 读取第一个TUM格式文件（基准文件）
file1_path = r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\Offroad3\olf.tum"  # 第一个文件路径
data1 = np.loadtxt(file1_path)

# 读取第二个TUM格式文件
file2_path = r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\Offroad3\olg.tum"  # 第二个文件路径
data2 = np.loadtxt(file2_path)

sum_squared_diffx = 0.0
sum_squared_diffy = 0.0
sum_squared_diffz = 0.0
count = 0
# 遍历第二个文件的时间戳，并进行计算
squared_diffx = 0
squared_diffy = 0
squared_diffz = 0
for row2 in data2:
    timestamp2 = row2[0]
    x2 = row2[1]
    y2 = row2[2]
    z2 = row2[3]
    
    # 在第一个文件中查找最接近的时间戳
    index = np.argmin(np.abs(data1[:, 0] - timestamp2))
    timestamp1 = data1[index, 0]
    x1 = data1[index, 1]
    y1 = data1[index, 2]
    z1 = data1[index, 3]
    
    # 计算时间戳差异
    time_diff = np.abs(timestamp2 - timestamp1)
    
    # 如果时间戳差异小于10毫秒，则进行计算
    if time_diff < 0.5:
        # squared_diff = (x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2
        squared_diffx = (x1 - x2) ** 2
        squared_diffy = (y1 - y2) ** 2
        squared_diffz = (z1 - z2) ** 2
        # print(squared_diff)
    sum_squared_diffx += squared_diffx
    sum_squared_diffy += squared_diffy
    sum_squared_diffz += squared_diffz
    count = count + 1
# print(count,sum_squared_diff)
sqrt_sum_squared_diffx = sum_squared_diffx ** 0.5
sqrt_sum_squared_diffy = sum_squared_diffy ** 0.5
sqrt_sum_squared_diffz = sum_squared_diffz ** 0.5
rmse_x = sqrt_sum_squared_diffx / count
rmse_y = sqrt_sum_squared_diffy / count
rmse_z = sqrt_sum_squared_diffz / count
print(rmse_x,rmse_y,rmse_z)
