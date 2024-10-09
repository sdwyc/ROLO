import numpy as np
import math
# 读取第一个TUM格式文件（基准文件）
file1_path = "/home/abin/tum_traj/off3/OL-HDL-SLAM.tum"  # 第一个文件路径
data1 = np.loadtxt(file1_path)

# 读取第二个TUM格式文件
file2_path = "/home/abin/tum_traj/off3/OL-GroundTruth.tum"  # 第二个文件路径
data2 = np.loadtxt(file2_path)

sum_squared_diff = 0.0
count = 0
squared_diff = 0
# 遍历第二个文件的时间戳，并进行计算
for row2 in data2:
    timestamp2 = row2[0]
    r2 = row2[1]
    p2 = row2[2]
    y2 = row2[3]
    
    # 在第一个文件中查找最接近的时间戳
    index = np.argmin(np.abs(data1[:, 0] - timestamp2))
    timestamp1 = data1[index, 0]
    r1 = data1[index, 1]
    p1 = data1[index, 2]
    y1 = data1[index, 3]
    
    # 计算时间戳差异
    time_diff = np.abs(timestamp2 - timestamp1)
    
    # 如果时间戳差异小于10毫秒，则进行计算
    if time_diff < 0.5:
        squared_diff = (r1 - r2) ** 2 + (p1 - p2) ** 2 + (y1 - y2) ** 2
        print(squared_diff)
    if squared_diff < 2:
        sum_squared_diff += squared_diff
        count = count + 1
print(count,sum_squared_diff)
sqrt_sum_squared_diff = sum_squared_diff ** 0.5
rmse_rpy = sqrt_sum_squared_diff / count
print(sqrt_sum_squared_diff,rmse_rpy)

jiaodu = rmse_rpy * 180 / math.pi
print(jiaodu)