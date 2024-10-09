import pandas as pd
import numpy as np

# 读取TUM格式文件
def read_tum_file(file_path):
    columns = ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    data = pd.read_csv(file_path, delim_whitespace=True, header=None, names=columns)
    return data

# 找到与给定时间戳最接近的行
def find_nearest_timestamp(timestamp, timestamps):
    return min(timestamps, key=lambda x: abs(x - timestamp))

# 在第一个TUM文件中查找最接近的时间戳行，并将原始行号记录到新文件中
def process_tum_files(tum_file1, tum_file2, output_file):
    # 读取两个TUM文件
    data1 = read_tum_file(tum_file1)
    data2 = read_tum_file(tum_file2)
    
    # 提取时间戳列
    timestamps1 = data1['timestamp'].values
    timestamps2 = data2['timestamp'].values
    
    # 创建新数据框用于存储结果
    result_data = pd.DataFrame(columns=['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'original_row'])
    
    # 遍历第二个TUM文件的时间戳
    for timestamp2 in timestamps2:
        # 找到第一个TUM文件中最接近的时间戳
        nearest_timestamp = find_nearest_timestamp(timestamp2, timestamps1)
        
        # 找到最接近时间戳对应的行号
        original_row = data1[data1['timestamp'] == nearest_timestamp].index[0]
        
        # 获取对应行的数据
        row_data = data1.iloc[original_row]
        
        # 将原始行号记录到结果数据框中
        result_data = result_data._append({'timestamp': row_data['timestamp'], 
                                          'x': row_data['x'], 
                                          'y': row_data['y'], 
                                          'z': row_data['z'], 
                                          'qx': row_data['qx'], 
                                          'qy': row_data['qy'], 
                                          'qz': row_data['qz'], 
                                          'qw': row_data['qw'], 
                                          'original_row': original_row}, 
                                         ignore_index=True)
    
    # 将结果数据保存到新的TUM文件中
    result_data.to_csv(output_file, sep=' ', index=False, header=False)

# 指定文件路径
tum_file1_path =   r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\xinglong\ROLO+LC.tum"#多
tum_file2_path =   r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\xinglong\GroundTruth.tum"
output_file_path = r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\xinglong\1.tum"

# 处理TUM文件
process_tum_files(tum_file1_path, tum_file2_path, output_file_path)
