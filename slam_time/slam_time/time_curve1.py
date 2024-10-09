from cProfile import label
import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
# 读取合并后的数据
methods = ['rolo', 'ct', 'loam', 'hdl', 'lego']
datasets = ['xls'] #['kitti00', 'kitti05', 'kitti08', 'off1', 'off2', 'xls', 'qfs']


# 定义一个函数来读取文件夹中的数据并计算总时间间隔
def read_data(folder_path, ct_flag = False, method = ""):
    df = pd.read_csv(folder_path, sep=' ', names=['timestamp', 'interval1', 'interval2'])
    if(ct_flag):
        df = pd.read_csv(folder_path, sep=' ', names=['timestamp', 'total_interval', 'map_points'])
        # 删除第一个数据
        df = df.drop(0)
    else:
        df = pd.read_csv(folder_path, sep=' ', names=['timestamp', 'interval1', 'interval2', 'map_points'])
    # 计算总时间间隔
        # df['timestamp'] = df['timestamp'] - df['timestamp'].iloc[0]
        df['total_interval'] = df['interval1'] + df['interval2']
        df['method'] = method
    return df

# 定义一个函数来筛选不同方法中时间戳相近的数据
def filter_close_timestamps(dfs, threshold=5e-2):
    # 创建一个空的DataFrame来存储筛选后的数据
    filtered_dfs = []
    for i, df1 in enumerate(dfs):
        for j, df2 in enumerate(dfs):
            if i >= j:
                continue
            filtered_timestamps = []
            filtered_intervals1 = []
            filtered_intervals2 = []
            for _, row in df1.iterrows():
                t1 = row['timestamp']
                i1 = row['total_interval']
                # 在另一个数据框中找到最接近的时间戳
                idx = (df2['timestamp'] - t1).abs().idxmin()
                t2 = df2.at[idx, 'timestamp']
                i2 = df2.at[idx, 'total_interval']
                if abs(t1 - t2) < threshold:
                    filtered_timestamps.append(t1)
                    filtered_intervals1.append(i1)
                    filtered_intervals2.append(i2)
            filtered_df = pd.DataFrame({'timestamp': filtered_timestamps, 'interval1': filtered_intervals1, 'interval2': filtered_intervals2})
            filtered_dfs.append((f'Method {i+1} vs Method {j+1} (Method {i+1})', filtered_df['timestamp'], filtered_df['interval1']))
            filtered_dfs.append((f'Method {i+1} vs Method {j+1} (Method {j+1})', filtered_df['timestamp'], filtered_df['interval2']))
    return filtered_dfs

# 读取n个文件夹中的数据
dfs = []
dfs_with_method = {}
for method in methods:
    for dataset in datasets:
        file = 'full_' + method + '/' + method + '_' + dataset + '.tum'
        if (method == 'ct'):
            df = read_data(file, True, method)
        else:
            df = read_data(file, False, method)
        dfs.append(df)
        dfs_with_method[method] = df
        # dfs = [read_data(folder) for folder in folders]
    

# 筛选不同方法中时间戳相近的数据
# filtered_dfs = filter_close_timestamps(dfs)
min_size = min(df.shape[0] for df in dfs)
print(min_size)
dfs_sampled = []
for df1 in dfs:
    if(df1.shape[0] > min_size):
        step = len(df1) // min_size
        indices = np.linspace(0, len(df1) - 1, num=min_size, dtype=int)
        # 根据索引选取数据
        dfi = df1.iloc[indices]
        dfs_sampled.append(dfi)
    else:
        dfs_sampled.append(df1)

# print(dfs_sampled[0])
# 创建一个新的图表
plt.figure()

# 绘制筛选后的时间间隔曲线
for i, df in enumerate(dfs_sampled):
    print(i, df.shape[0])
    plt.plot(df['timestamp'], df['total_interval'], label=methods[i])
# 设置图表的标题和图例
plt.title('Total Interval Comparison')
plt.xlabel('Timestamp')
plt.ylabel('Total Interval (ms)')
plt.legend()

# 显示图表
plt.show()
