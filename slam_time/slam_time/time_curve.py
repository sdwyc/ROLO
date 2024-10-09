import pandas as pd
import matplotlib.pyplot as plt

# 读取合并后的数据
methods = ['rolo', 'ct', 'loam']
datasets = ['qfs'] #['kitti00', 'kitti05', 'kitti08', 'off1', 'off2', 'xls', 'qfs']
# 创建一个新的图表
# plt.figure()

# 设置第一个纵轴的颜色和标签
# color = 'tab:black'
plt.xlabel('Timestamp')
plt.ylabel('Total Interval')
for method in methods:
    for dataset in datasets:
        file = 'full_' + method + '/' + method + '_' + dataset + '.tum'
        if(method == 'ct'):
            df = pd.read_csv(file, sep=' ', names=['timestamp', 'total_interval', 'map_points'])
        else:
            df = pd.read_csv(file, sep=' ', names=['timestamp', 'interval1', 'interval2', 'map_points'])
        # 计算总时间间隔
            df['total_interval'] = df['interval1'] + df['interval2']
        plt.plot(df['timestamp'], df['total_interval'], label=method)
        plt.legend()
        

# 显示图表
# fig.tight_layout()
plt.grid()
plt.show()
