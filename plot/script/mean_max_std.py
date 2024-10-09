import numpy as np

def read_tum_file(file_path, column, operation, threshold=np.inf):
    """
    Reads a TUM file and calculates the specified operation (std, mean, max) on a specified column,
    excluding outliers based on a given threshold before performing the operation.
    
    Parameters:
    - file_path: Path to the TUM file.
    - column: Index of the column to perform the operation on.
    - operation: The operation to perform ('std', 'mean', 'max').
    - threshold: Threshold for excluding outliers (used for 'std', 'mean', and 'max').
    
    Returns:
    - The result of the specified operation on the filtered data.
    """
    values = []
    with open(file_path, 'r') as file:
        for line in file:
            data = line.strip().split()
            if len(data) > column:
                value = float(data[column])
                if abs(value) < threshold:  # Only consider values within the threshold for all operations
                    values.append(value)

    if operation == 'std':
        return np.std(values)
    elif operation == 'mean':
        return np.mean(values)
    elif operation == 'max':
        return max(values)
    else:
        raise ValueError("Unsupported operation specified.")

# 示例用法
file_path = r"C:\Users\16961\Desktop\SDU\reserch\ROLO\plot\data\original\xinglong\ol3.tum"
# file_path = "/home/abin/tum_traj/ERROR/sdu/xinglong/pose/error/rolo.tum"

# 现在在求最大值之前也会剔除超出阈值的异常数据
col = 2
mean_value = read_tum_file(file_path, col, 'mean', 0.1)
# print("Mean value:", mean_value)

max_value = read_tum_file(file_path, col, 'max', 0.1)  # 假定超过100为异常值
# print("Max value:", max_value)

std_value = read_tum_file(file_path, col, 'std', 0.1)
# print("Standard deviation:", std_value)
print(str(round(mean_value,3))+'/'+str(str(round(round(max_value,3),3))+'/'+str(round(std_value,3))))
