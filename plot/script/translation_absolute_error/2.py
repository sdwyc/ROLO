import pandas as pd

# 读取TUM格式文件
def read_tum_file(file_path):
    columns = ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    data = pd.read_csv(file_path, delim_whitespace=True, header=None, names=columns)
    return data

# 对第二、三、四列执行后一行减前一行的操作
def calculate_difference(data):
    diff_data = data.diff().fillna(0)  # 计算差值，并将NaN填充为0
    return diff_data

# 将数据保存到新的TUM文件中
def save_to_tum_file(data, output_file):
    data.to_csv(output_file, sep=' ', index=False, header=False)

# 指定文件路径
tum_file_path =    r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\xinglong\1.tum"
output_file_path = r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\xinglong\2.tum"
# 读取TUM文件
data = read_tum_file(tum_file_path)

# 对第二、三、四列执行后一行减前一行的操作
diff_data = calculate_difference(data[['x', 'y', 'z']])

# 将结果保存到新的TUM文件中
save_to_tum_file(diff_data, output_file_path)
