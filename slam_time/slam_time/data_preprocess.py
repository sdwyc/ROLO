from os import makedirs
import os
import pandas as pd

methods = ['rolo', 'lego', 'hdl', 'loam'] #, 'ct']
datasets = ['kitti00', 'kitti05', 'kitti08', 'off1', 'off2', 'off3', 'xls', 'qfs']

def remove_first_line(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    with open(file_path, 'w') as f:
        f.writelines(lines[1:])

for method in methods:
    for dataset in datasets:
        if not os.path.exists('full_' + method):
            os.mkdir('full_' + method)
        # 读取两个文件
        file1 = method + '/'+ method + '_front_' + dataset + '.tum'
        file2 = method + '/'+ method + '_back_' + dataset + '.tum'

        df1 = pd.read_csv(file1, sep=' ', names=['timestamp', 'interval'])
        df2 = pd.read_csv(file2, sep=' ', names=['timestamp', 'interval', 'map_points'])

        # 为每个时间戳找到最接近的匹配项
        def find_closest_match(row, other_df):
            time_diff = (other_df['timestamp'] - row['timestamp']).abs()
            min_diff = time_diff.min()
            if min_diff < 5e-2:
                closest_row = other_df.loc[time_diff.idxmin()]
                return pd.Series({'interval': closest_row['interval'], 'map_points': closest_row['map_points']})
            else:
                return pd.Series({'interval': None, 'map_points': None})

        # 对每个数据框中的行应用匹配函数
        matches = df1.apply(lambda row: find_closest_match(row, df2), axis=1)

        # 将匹配结果添加到原始数据框中
        df1 = pd.concat([df1, matches], axis=1)

        # 删除没有匹配项的行
        df1.dropna(inplace=True)

        # 如果需要，可以将结果保存到新文件
        df1.to_csv('full_' + method + '/' + method + '_' + dataset + '.tum', index=False, sep=' ')
        remove_first_line('full_' + method + '/' + method + '_' + dataset + '.tum')
        # 打印合并后的数据框，以查看结果
        print(df1)
        