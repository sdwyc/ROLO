from operator import truediv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import os
from matplotlib.font_manager import FontProperties
from matplotlib.ticker import MultipleLocator


# 设置字体属性
font_legend = FontProperties(family='Times New Roman', size=12)  # 图例字体
font_labels = FontProperties(family='Times New Roman', size=24)  # 轴标签字体
font_ticks = FontProperties(family='Times New Roman', size=18)    # 轴刻度字体


def read_tum(tum_file):
    columns = ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    data = pd.read_csv(tum_file, delim_whitespace=True, header=None, names=columns)
    file_name = os.path.basename(tum_file)
    line_name = file_name[:-4]

    return data,line_name,file_name

def plot_trajectory(tum_file, dir_name=None, background_color='white', 
                    trajectory_color={}, trajectory_linestyle='-'):
    # 绘制轨迹
    fig = plt.figure(figsize=(10, 5))
    ax = fig.add_subplot(111)
    fig.subplots_adjust(left=0.115, right=0.985, top=0.945, bottom=0.130)

    # fig.patch.set_facecolor('#FFDEAD')
    tum_lenth=len(tum_file)
    # print(os.path.basename(tum_file[0]))
    if os.path.basename(tum_file[0]) == 'GroundTruth.tum':
        start_time = min(read_tum(tum_file[0])[0]['timestamp'])
        ax.plot(read_tum(tum_file[0])[0]['timestamp']-start_time,read_tum(tum_file[0])[0]['z'], color = '#808A87', 
                linestyle='--',label=read_tum(tum_file[0])[1])
        # start_point = (read_tum(tum_file[0])[0]['x'][0], 
        #                read_tum(tum_file[0])[0]['y'][0],read_tum(tum_file[0])[0]['z'][0])
        # ax.scatter(start_point[0], start_point[1],start_point[1], color='red', marker='*',s=100)

        for i in range(1,tum_lenth):
            filename_with_extension = os.path.basename(tum_file[i])
            method, extension = os.path.splitext(filename_with_extension)
            # if i == 2 or i == 5:
            #     ax.plot(0, 0, color = trajectory_color[method], linestyle=trajectory_linestyle,label=read_tum(tum_file[i])[1]+' (Failed)')
            # else:
            ax.plot(read_tum(tum_file[i])[0]['timestamp']-start_time,read_tum(tum_file[i])[0]['z'], 
                    color = trajectory_color[method], linestyle=trajectory_linestyle,label=read_tum(tum_file[i])[1])
    else:
        for i in range(0,tum_lenth):
            filename_with_extension = os.path.basename(tum_file[i])
            method, extension = os.path.splitext(filename_with_extension)
            start_time = min(read_tum(tum_file[i])[0]['timestamp'])
            ax.plot(read_tum(tum_file[i])[0]['timestamp']-start_time,read_tum(tum_file[i])[0]['z'], color = trajectory_color[i], 
                    linestyle=trajectory_linestyle,label=read_tum(tum_file[i])[1])
            # start_point = (read_tum(tum_file[i])[0]['x'][0], 
            #             read_tum(tum_file[i])[0]['y'][0],read_tum(tum_file[i])[0]['z'][0])
            # ax.scatter(start_point[0], start_point[1],start_point[1], color='red', marker='*',s=100)


    # 设置图例
    # ax.legend(prop=font_legend, loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=7)    # 设置坐标轴线型

    # 设置坐标轴线型
    ax.grid(True, color='#BDC3C7', linestyle='--', linewidth=0.8)

    # 设置坐标轴标签
    ax.set_xlabel('Time(s)',fontproperties=font_labels)
    ax.set_ylabel('Z(m)',fontproperties=font_labels)
    ax.tick_params(axis='x', direction='in', labelsize=font_ticks.get_size())  # x轴样式
    ax.tick_params(axis='y', direction='in', labelsize=font_ticks.get_size())  # x轴样式
    # coord_text = ax.text(0.5, 0.95, '', transform=ax.transAxes, ha='center')

    if dir_name=='KITTI00':
        ax.set_xlim(-100, 500)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-300, 300)  # 设置 y 轴范围为 0 到 30
    if dir_name=='KITTI05':
        ax.set_xlim(-100, 400)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-200, 200)  # 设置 y 轴范围为 0 到 30
    if dir_name=='KITTI08':
        ax.set_xlim(-200, 600)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-400, 400)  # 设置 y 轴范围为 0 到 30
    if dir_name=='qianfo':
        ax.set_xlim(-50,550)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-175, 110)  # 设置 y 轴范围为 0 到 30
    if dir_name=='xinglong':
        ax.set_xlim(-40, 550)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-500, 300)  # 设置 y 轴范围为 0 到 30
    if dir_name=='Offroad1':
        ax.set_xlim(-40, 199)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-300, 120)  # 设置 y 轴范围为 0 到 30
    if dir_name=='Offroad2':
        ax.set_xlim(-45, 220)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-150, 80)  # 设置 y 轴范围为 0 到 30
    if dir_name=='Offroad3':
        ax.set_xlim(-45, 220)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-150, 80)  # 设置 y 轴范围为 0 到 30
    
    plt.show()

# 示例用法

def plot(map):
    root_dir = os.path.abspath("..")
    tum_list = [
            root_dir+map+'/GroundTruth.tum',
            root_dir+map+'/LOAM.tum',
            root_dir+map+'/LeGO-LOAM.tum',
            root_dir+map+'/HDL-SLAM.tum',
            root_dir+map+'/CT-ICP.tum',
            root_dir+map+'/ROLO.tum',
            # root_dir+map+'/ROLO+LC.tum',
            # root_dir+map+'/ROLO-LC.tum'
           ]
    dir_name=os.path.basename(os.path.dirname(tum_list[0]))

    trajectories_color = {
        'ROLO': '#00FF00',
        'CT-ICP': '#EC7063',
        'LeGO-LOAM': '#C39BD3',
        'HDL-SLAM': '#F7DC6F',
        'LOAM': '#7FB3D5'}
    plot_trajectory(tum_list,dir_name, background_color='white', trajectory_color=trajectories_color, trajectory_linestyle='-')

# 判断当前模块是否作为主程序运行
if __name__ == "__main__":
    # 如果是主程序，则调用主函数
    plot('/data/original/xinglong')
