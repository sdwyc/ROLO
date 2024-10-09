import pandas as pd

# 读取TUM格式文件
def read_tum_file(file_path):
    data = pd.read_csv(file_path, delim_whitespace=True, header=None)
    return data

# 对两个TUM文件对应位置作差
def calculate_difference(data1, data2):
    diff_data = data1.subtract(data2).abs()
    return diff_data

# 将数据保存到新的TUM文件中
def save_to_tum_file(data, output_file):
    data.to_csv(output_file, sep=' ', index=False, header=False)

# 指定文件路径
tum_file1_path =   r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\Offroad2\ol2gt.tum"
tum_file2_path =   r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\Offroad2\ol2.tum"
output_file_path = r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\Offroad2\ol3.tum"

# 读取TUM文件
data1 = read_tum_file(tum_file1_path)
data2 = read_tum_file(tum_file2_path)

# 对两个TUM文件对应位置作差
diff_data = calculate_difference(data1, data2)
# print(diff_data,type(diff_data))
# 将结果保存到新的TUM文件中
save_to_tum_file(diff_data, output_file_path)
