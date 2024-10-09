# 打开原始文件
with open('/home/abin/Desktop/plot2.tum', 'r') as file:
    # 逐行读取文件内容
    lines = file.readlines()

# 初始化一个空列表，用于存储保留的行
filtered_lines = []

# 遍历文件的每一行
for line in lines:
    # 按空格分割每一行的内容
    columns = line.split()
    # 如果第二列小于等于0.1，则保留该行
    if len(columns) <= 1 or float(columns[3]) <= 0.05:
        filtered_lines.append(line)

# 将保留的行写入新文件
with open('/home/abin/Desktop/plot3.tum', 'w') as filtered_file:
    filtered_file.writelines(filtered_lines)

print("已删除第二列大于0.1的行，并保存到 filtered_file.tum")
