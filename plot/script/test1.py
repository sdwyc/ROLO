import numpy as np
import pandas as pd

# 读取TUM文件
data = pd.read_csv(r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\Offroad2\ol3.tum", sep=' ', header=None)

# 假设需要计算模长的三列数据是第2-4列
vectors = data.iloc[:, 0:3].values

# 计算每行的模长
magnitudes = np.linalg.norm(vectors, axis=1)
output_data = np.vstack((magnitudes)).T

# 保存为新的TUM文件
output_df = pd.DataFrame(magnitudes, columns=['magnitude'])
output_df.to_csv(r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\Offroad2\ol4.tum", sep=' ', index=False, header=False)
print("模长已保存至 'output_magnitudes.tum'")
# 计算统计量
filtered_magnitudes = magnitudes[magnitudes <= 0.035]

# 计算统计量
mean_magnitude = np.mean(filtered_magnitudes)
std_magnitude = np.std(filtered_magnitudes)
min_magnitude = np.min(filtered_magnitudes)
max_magnitude = np.max(filtered_magnitudes)

# 打印结果
print(f"模长均值: {mean_magnitude}")
print(f"模长最大值: {max_magnitude}")
print(f"模长标准差: {std_magnitude}")
