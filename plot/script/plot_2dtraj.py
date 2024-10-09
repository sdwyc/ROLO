import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import os
from matplotlib.font_manager import FontProperties
from matplotlib.ticker import MultipleLocator
from sys import path
# 指定字体路径
# 设置字体属性
font_legend = FontProperties(family='Times New Roman', size=18)  # 图例字体
font_labels = FontProperties(family='Times New Roman', size=20)  # 轴标签字体
font_ticks = FontProperties(family='Times New Roman', size=20)    # 轴刻度字体

def read_tum(tum_file):
    columns = ['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    data = pd.read_csv(tum_file, delim_whitespace=True, header=None, names=columns)
    file_name = os.path.basename(tum_file)
    line_name = file_name[:-4]
    if(line_name == 'ROLO+LC'): line_name = 'ROLO'

    return data,line_name,file_name

def plot_trajectory(tum_file, dir_name=None, background_color='white', 
                    trajectory_color=[], trajectory_linestyle='-'):
    fig = plt.figure(figsize=(10, 9), facecolor='white')
    fig.subplots_adjust(left=0.135, right=0.995, top=0.930, bottom=0.100)
    ax = fig.add_subplot(111)
    ax.set_facecolor('#FFFFFF')
    # ax.set_aspect((1))

    tum_lenth=len(tum_file)
    if os.path.basename(tum_file[0]) == 'GroundTruth.tum':
        start_point = (read_tum(tum_file[0])[0]['x'][0], 
                       read_tum(tum_file[0])[0]['y'][0],)
        ax.scatter(start_point[0], start_point[1],color='red', marker='*',s=300, zorder=3)
        ax.plot(*start_point, '*',color='red', label='Start')
        ax.plot(read_tum(tum_file[0])[0]['x'], read_tum(tum_file[0])[0]['y'],
                color = '#808A87', linestyle='--',label=read_tum(tum_file[0])[1],linewidth=2.75)

        for i in range(1,tum_lenth):
            filename_with_extension = os.path.basename(tum_file[i])
            method, extension = os.path.splitext(filename_with_extension)
            ax.plot(read_tum(tum_file[i])[0]['x'], read_tum(tum_file[i])[0]['y'],  
                    color = trajectory_color[method], linestyle=trajectory_linestyle,label=read_tum(tum_file[i])[1],linewidth=2)
            start_point = (read_tum(tum_file[i])[0]['x'][0], 
                        read_tum(tum_file[i])[0]['y'][0],)
            # ax.scatter(start_point[0], start_point[1], color='red', marker='*',s=300, zorder=3)
    else:
        start_point = (read_tum(tum_file[0])[0]['x'][0], 
                       read_tum(tum_file[0])[0]['y'][0],)
        ax.scatter(start_point[0], start_point[1],color='red', marker='*',s=300, zorder=3)        
        ax.plot(*start_point, '*',color='red', label='Start')
        for i in range(0,tum_lenth):
            filename_with_extension = os.path.basename(tum_file[i])
            method, extension = os.path.splitext(filename_with_extension)
            ax.plot(read_tum(tum_file[i])[0]['x'], read_tum(tum_file[i])[0]['y'], 
                    color = trajectory_color[method], linestyle=trajectory_linestyle,label=read_tum(tum_file[i])[1],linewidth=2)
            start_point = (read_tum(tum_file[i])[0]['x'][0], 
                        read_tum(tum_file[i])[0]['y'][0])
            ax.scatter(start_point[0], start_point[1], color='red', marker='*',s=300, zorder=3)


    # 设置图例
    ax.legend(loc='upper right',prop=font_legend,handlelength=2,markerscale=2)
  
    # 设置坐标轴线型
    ax.grid(True,linestyle='--')

    # 设置坐标轴标签
    ax.set_xlabel('X (m)',fontproperties=font_labels)
    ax.set_ylabel('Y (m)',fontproperties=font_labels)
    ax.tick_params(axis='x', direction='in',labelsize=font_ticks.get_size(),gridOn=True,size=5,pad=15)  # x轴样式
    ax.tick_params(axis='y', direction='in',labelsize=font_ticks.get_size(),gridOn=True,size=5,pad=15)  # x轴样式

    for spine in ax.spines.values():
        spine.set_linewidth(3)
    if dir_name=='KITTI00':
        ax.set_xlim(-45.22222388763428, 560.5350106613159) 
        ax.set_ylim(-320.45743203125005, 300.33707265625003) 
    if dir_name=='KITTI05':
        ax.set_xlim(-77.5, 420)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-250, 280)  # 设置 y 轴范围为 0 到 30
    if dir_name=='KITTI08':
        ax.set_xlim(-155, 520)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-450, 420)  # 设置 y 轴范围为 0 到 30
    if dir_name=='qianfo':
        ax.set_xlim(0, 300)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-50, 250)  # 设置 y 轴范围为 0 到 30
    if dir_name=='xinglong':
        ax.set_xlim(0, 700)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-300, 400)  # 设置 y 轴范围为 0 到 30
    if dir_name=='Offroad1':
        ax.set_xlim(0, 400)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(-400, 0)  # 设置 y 轴范围为 0 到 30
    if dir_name=='Offroad2':
        ax.set_xlim(-100, 400)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(0, 700)  # 设置 y 轴范围为 0 到 30

    if dir_name=='Offroad3':
        ax.set_xlim(-100, 400)  # 设置 x 轴范围为 0 到 6
        ax.set_ylim(0, 700)  # 设置 y 轴范围为 0 到 30
    
    plt.show()

# 示例用法

def plot(map):
    # root_dir = os.path.abspath("..")
    # dataset = ""
    # tum_list = [
    #         # '/home/abin/tum_traj/'+map+'/GroundTruth.tum',
    #         # '/home/abin/tum_traj/'+map+'/ROLO.tum',
    #         '/home/abin/tum_traj/'+map+'/ROLO+LC.tum',
    #         '/home/abin/tum_traj/'+map+'/LOAM.tum',
    #         '/home/abin/tum_traj/'+map+'/LeGO-LOAM.tum',
    #         '/home/abin/tum_traj/'+map+'/CT-ICP.tum',
    #         '/home/abin/tum_traj/'+map+'/HDL-SLAM.tum',
    #         # '/home/abin/tum_traj/'+map+'/ROLO-LC.tum'
    #        ]
    # dir_name=os.path.basename(os.path.dirname(tum_list[0]))

    # trajectories_color = ['#00FF00','#0000FF','#9933FA','#FF0000','#FFD700']

    root_dir = os.path.abspath("..")
    tum_list = [
            root_dir+map+'/GroundTruth.tum',
            root_dir+map+'/LOAM.tum',
            root_dir+map+'/LeGO-LOAM.tum',
            root_dir+map+'/HDL-SLAM.tum',
            root_dir+map+'/CT-ICP.tum',
            # root_dir+map+'/ROLO.tum',
            root_dir+map+'/ROLO+LC.tum',
            # root_dir+map+'/ROLO-LC.tum'
           ]
    dir_name=os.path.basename(os.path.dirname(tum_list[0]))
    print(dir_name)
    trajectories_color = {
        'ROLO': (0, 1.0, 0.0),  # Blue
        'LOAM': (0., 0., 1),   # Purple
        'LeGO-LOAM': (170.0/255.0, 0.0, 1),  # Red
        'CT-ICP': (1.0, 0., 0.),           # Orange
        'HDL-SLAM': (255.0/255.0, 156.0/255.0, 0.),   # Green

        # 'ROLO': '#00FF00',
        'ROLO+LC': (0, 1.0, 0.0),
        # 'CT-ICP': '#EC7063',
        # 'LeGO-LOAM': '#C39BD3',
        # 'HDL-SLAM': '#F7DC6F',
        # 'LOAM': '#7FB3D5'
        }
    plot_trajectory(tum_list,dir_name, background_color='white', trajectory_color=trajectories_color, trajectory_linestyle='-')

# 判断当前模块是否作为主程序运行
if __name__ == "__main__":
    # 如果是主程序，则调用主函数
    plot('/data/original/KITTI08')
