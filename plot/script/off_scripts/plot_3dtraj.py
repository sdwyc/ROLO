from logging import root
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import os
from matplotlib.font_manager import FontProperties
from matplotlib.ticker import MultipleLocator

# 设置字体属性
font_legend = FontProperties(family='Times New Roman', size=16)  # 图例字体
font_labels = FontProperties(family='Times New Roman', size=18)  # 轴标签字体
font_ticks = FontProperties(family='Times New Roman', size=14)    # 轴刻度字体

def read_tum(tum_file):
    columns = ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    data = pd.read_csv(tum_file, delim_whitespace=True, header=None, names=columns)
    # print(data,type(data))
    file_name = os.path.basename(tum_file)
    line_name = file_name[:-4]

    return data,line_name,file_name

def plot_trajectory(tum_file, dir_name=None, background_color='white', 
                    trajectory_color={}, trajectory_linestyle='-'):
    fig = plt.figure(figsize=(9, 9), facecolor='white')
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor('white')
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))  # RGB white, with alpha 1
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 1.0))
    fig.subplots_adjust(left=0.065, right=0.96, top=0.935, bottom=0.035)
    tum_lenth=len(tum_file)
    if os.path.basename(tum_file[0]) == 'GroundTruth.tum':
        start_point = (read_tum(tum_file[0])[0]['x'][0], 
                       read_tum(tum_file[0])[0]['y'][0],read_tum(tum_file[0])[0]['z'][0])
        ax.scatter(start_point[0], start_point[1],start_point[1], color='red', marker='*',s=100)
        ax.plot(*start_point, '*',color='red', label='Start')
        ax.plot(read_tum(tum_file[0])[0]['x'], read_tum(tum_file[0])[0]['y'], read_tum(tum_file[0])[0]['z'], 
                color = '#808A87', linestyle='--',linewidth=2.75,label=read_tum(tum_file[0])[1])

        for i in range(1,tum_lenth):
            # if i == 2 :
            #     ax.plot(0, 0, 0, color = trajectory_color[i-1], linestyle=trajectory_linestyle,label=read_tum(tum_file[i])[1]+' (Failed)')
            # else:
            filename_with_extension = os.path.basename(tum_file[i])
            method, extension = os.path.splitext(filename_with_extension)
            ax.plot(read_tum(tum_file[i])[0]['x'], read_tum(tum_file[i])[0]['y'], read_tum(tum_file[i])[0]['z'], 
                    color = trajectory_color[method], linestyle=trajectory_linestyle,linewidth=2,label=read_tum(tum_file[i])[1])
            start_point = (read_tum(tum_file[i])[0]['x'][0], 
                        read_tum(tum_file[i])[0]['y'][0],read_tum(tum_file[i])[0]['z'][0])
            # ax.scatter(start_point[0], start_point[1],start_point[1], color='red', marker='*',s=100)
    else:
        for i in range(0,tum_lenth):
            method = os.path.basename(tum_file[i])
            ax.plot(read_tum(tum_file[i])[0]['x'], read_tum(tum_file[i])[0]['y'], read_tum(tum_file[i])[0]['z'], 
                    color = trajectory_color[method], linestyle=trajectory_linestyle,linewidth=2,label=read_tum(tum_file[i])[1])
            start_point = (read_tum(tum_file[i])[0]['x'][0], 
                        read_tum(tum_file[i])[0]['y'][0],read_tum(tum_file[i])[0]['z'][0])
            ax.scatter(start_point[0], start_point[1],start_point[1], color='red', marker='*',s=100)


    # 设置图例
    # ax.legend(loc='upper right', bbox_to_anchor=(0.8, 0.8))
    # 获取当前所有图例句柄和标签
    handles, labels = ax.get_legend_handles_labels()

    # 假设 'Start' 已经是第一个句柄和标签
    # 创建一个空白的图例项
    empty_handle = Patch(facecolor='none', edgecolor='none')  # 创建一个透明的图例项
    empty_label = ''

    # 将空白图例项添加到第二行第一列
    print(len(handles))
    handles.insert(1, empty_handle)  # 假设4列，2行，我们需要在第5个位置添加空白图例
    labels.insert(1, empty_label)

    # 创建图例，分为两行四列
    ax.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.99), ncol=4, prop=font_legend)
    # ax.legend(prop=font_legend, loc='upper center', bbox_to_anchor=(0.5, 1.15), ncol=4)    # 设置坐标轴线型
    # ax.grid(color='gray', linestyle='dashed')
    grid_color = '#BDC3C7'  # 浅灰色
    grid_linestyle = '-'
    grid_linewidth = 0.5    # 线宽    
    ax.xaxis._axinfo["grid"].update(color=grid_color, linestyle=grid_linestyle, linewidth=grid_linewidth)
    ax.yaxis._axinfo["grid"].update(color=grid_color, linestyle=grid_linestyle, linewidth=grid_linewidth)
    ax.zaxis._axinfo["grid"].update(color=grid_color, linestyle=grid_linestyle, linewidth=grid_linewidth)

    # 设置视角
    ax.view_init(elev=22, azim=-116)

    # 设置坐标轴标签
    ax.set_xlabel('X (m)', fontproperties=font_labels, labelpad = 3)
    ax.set_ylabel('Y (m)', fontproperties=font_labels, labelpad = 10)
    ax.set_zlabel('Z (m)', fontproperties=font_labels, labelpad = 13)
    # ax.set_zlabel('')
    # ax.text(0, 0, -100.1, "Z (m)", fontproperties=font_labels, rotation=90, ha='center', va='top')
    ax.xaxis.set_major_locator(MultipleLocator(200))
    ax.yaxis.set_major_locator(MultipleLocator(200))
    ax.zaxis.set_major_locator(MultipleLocator(200))
    ax.tick_params(axis='both', which='major', labelsize=font_ticks.get_size())
    ax.tick_params(axis='x', pad=-2)  # X轴
    # ax.tick_params(axis='y', pad=15)  # Y轴
    ax.tick_params(axis='z', pad=8)  # Z轴
    x_min, x_max, y_min, y_max, z_min, z_max = 0, 0, 0, 0, 0, 0
    x_interval, y_interval, z_interval = 120, 120, 120
    if dir_name=='KITTI00':
        x_min, x_max = -100, 500
        y_min, y_max = -300, 300
        z_min, z_max = -300, 300
        
    if dir_name=='KITTI05':
        x_min, x_max = -100, 400
        y_min, y_max = -200, 200
        z_min, z_max = -200, 200

    if dir_name=='KITTI08':
        x_min, x_max = -200, 600
        y_min, y_max = -400, 400
        z_min, z_max = -400, 400

    if dir_name=='qianfo':
        x_min, x_max = 0, 300
        y_min, y_max = -50, 250
        z_min, z_max = -150,150

    if dir_name=='xinglong':
        x_min, x_max = 0, 700
        y_min, y_max = -300, 280
        z_min, z_max = -300, 270

    if dir_name=='Offroad1':
        x_min, x_max = -50, 350
        y_min, y_max = -450, 50
        z_min, z_max = -300,100

    if dir_name=='Offroad2':
        x_min, x_max = -100, 400
        y_min, y_max = 0, 600
        z_min, z_max = -300, 300

    if dir_name=='Offroad3':
        x_min, x_max = -50, 350
        y_min, y_max = -250, 250
        z_min, z_max = -200,200
    
    ax.set_xlim(x_min, x_max) 
    ax.set_ylim(y_min, y_max)  
    ax.set_zlim(z_min, z_max)
    ax.set_xticks(range(x_min, x_max + x_interval, x_interval))
    ax.set_yticks(range(y_min, y_max + y_interval, y_interval))
    ax.set_zticks(range(z_min, z_max + z_interval, z_interval))

    plt.show()

# 示例用法

def plot(map):
    root_dir = os.path.abspath("..")
    tum_list = [
            root_dir+map+'/GroundTruth.tum',
            # root_dir+map+'/LOAM.tum',
            # root_dir+map+'/LeGO-LOAM.tum',
            # root_dir+map+'/HDL-SLAM.tum',
            # root_dir+map+'/CT-ICP.tum',
            root_dir+map+'/FAST-LIO2.tum',
            # root_dir+map+'/ROLO+LC.tum',
            root_dir+map+'/ROLO.tum'
           ]
    dir_name=os.path.basename(os.path.dirname(tum_list[0]))
    print(dir_name)
    trajectories_color = {        
        # 'ROLO': (0, 1.0, 0.0),  # Blue
        'ROLO': (1.0, 0., 0.),  # Blue
        'LOAM': (0., 0., 1),   # Purple
        'LeGO-LOAM': (170.0/255.0, 0.0, 1),  # Red
        'CT-ICP': (1.0, 0., 0.),           # Orange
        'HDL-SLAM': (255.0/255.0, 156.0/255.0, 0.),   # Green

        # 'ROLO': '#FF0000',
        # 'CT-ICP': '#FFA500',
        # 'LeGO-LOAM': '#00FF00',
        # 'HDL-SLAM': '#00CED1',
        # 'LOAM': '#9400D3',
        # 'ROLO+LC':'#FF0000',
        'FAST-LIO2':'#0000FF'}
    plot_trajectory(tum_list,dir_name, background_color='black', trajectory_color=trajectories_color, trajectory_linestyle='-')

# 判断当前模块是否作为主程序运行
if __name__ == "__main__":
    # 如果是主程序，则调用主函数
    plot('/data/original/Offroad3')
