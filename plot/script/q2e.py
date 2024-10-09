import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R

# 读取TUM文件
data = pd.read_csv(r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\Offroad3\ROLO+LC.tum", sep=' ', header=None)

# 提取时间戳
timestamps = data.iloc[:, 0].values

# 提取四元数
quaternions = data.iloc[:, 4:8].values

# 将四元数转换为欧拉角
rotations = R.from_quat(quaternions)
euler_angles = rotations.as_euler('xyz', degrees=False)  # 'xyz'表示欧拉角顺序，默认单位为弧度

# 将时间戳和欧拉角组合
output_data = np.hstack((timestamps.reshape(-1, 1), euler_angles))

# 保存到新文件
output_df = pd.DataFrame(output_data, columns=['timestamp', 'roll', 'pitch', 'yaw'])
output_df.to_csv(r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\Offroad3\olrlc.tum", sep=' ', index=False, header=False)

print("转换完成，结果已保存至 'output_file.tum'")
