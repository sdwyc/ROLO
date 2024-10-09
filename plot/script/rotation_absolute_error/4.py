import pandas as pd

# 读取TUM格式文件
def read_tum_file(file_path):
    data = pd.read_csv(file_path, delim_whitespace=True, header=None)
    return data

# 将第九列添加到最前列
def add_column_to_front(data1, data2):
    # 提取第九列数据
    column_to_add = data1.iloc[:, 4]
    # 添加到第二个TUM文件的最前列
    data2.insert(0, 'column_from_1st_tum', column_to_add)
    return data2

# 将数据保存到新的TUM文件中
def save_to_tum_file(data, output_file):
    data.to_csv(output_file, sep=' ', index=False, header=False)

# 指定文件路径
tum_file1_path =   '/home/abin/tum_traj/ERROR/rotation/kitti00/result.tum'
tum_file2_path =   '/home/abin/tum_traj/ERROR/rotation/kitti00/error.tum'
output_file_path = '/home/abin/tum_traj/ERROR/rotation/kitti00/plot.tum'

# 读取TUM文件
data1 = read_tum_file(tum_file1_path)
data2 = read_tum_file(tum_file2_path)

# 将第九列添加到最前列
new_data2 = add_column_to_front(data1, data2)

# 将结果保存到新的TUM文件中
save_to_tum_file(new_data2, output_file_path)
