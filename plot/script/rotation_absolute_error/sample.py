def extract_every_nth_line(input_file, output_file, n):
    with open(input_file, 'r') as f:
        lines = f.readlines()

    selected_lines = [lines[i] for i in range(0, len(lines), n)]

    with open(output_file, 'w') as f:
        f.writelines(selected_lines)

# 使用示例：
input_file = '/home/abin/Desktop/plot3.tum'  # 您的输入文件名
output_file = '/home/abin/Desktop/plot4.tum'  # 新文件的文件名
n = 6  # 每隔几行取一行

extract_every_nth_line(input_file, output_file, n)
