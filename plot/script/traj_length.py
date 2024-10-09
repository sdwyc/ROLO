import numpy as np

# 从文件中读取数据
file_path = '/home/abin/tum_traj/off3/GroundTruth.tum'  # 指定TUM文件的路径

# 读取数据并计算轨迹长度
trajectory_length = 0.0
prev_position = None

# 打开文件并逐行处理数据
with open(file_path, 'r') as file:
    for line in file:
        # 按空格分割每一行的内容
        columns = line.split()
        # 解析坐标
        timestamp = float(columns[0])
        position = np.array([float(x) for x in columns[1:4]])

        # 计算位移并累加到轨迹长度
        if prev_position is not None:
            displacement = np.linalg.norm(position - prev_position)  # 计算位移
            trajectory_length += displacement

        # 更新前一个位置
        prev_position = position

print("轨迹的整体长度为:", trajectory_length)
