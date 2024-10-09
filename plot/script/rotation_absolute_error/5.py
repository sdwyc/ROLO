import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd
import os
from matplotlib.font_manager import FontProperties
from matplotlib.ticker import MultipleLocator


# 指定字体路径
font_path = '/home/abin/TimesNewRoman/TimesNewRoman.ttf' # 这里替换为你的字体文件路径
font_prop = FontProperties(fname=font_path,size=35)


def read_tum(tum_file):
    columns = ['timestamp', 'x', 'y', 'z']
    data = pd.read_csv(tum_file, delim_whitespace=True, header=None, names=columns)

    return data

def plot_trajectory(tum_file,tum_file1, background_color='white', 
                    trajectory_color=[], trajectory_linestyle='-'):
    # 绘制轨迹
    fig = plt.figure(figsize=(32, 7))
    fig.subplots_adjust(top=0.95)

    
    ax = fig.add_subplot(111)
    # ax.set_aspect(100)
    ax.set_facecolor('#FFFFFF')
    start_time = min(read_tum(tum_file)['timestamp'])
    ax.plot(1*(read_tum(tum_file)['timestamp']-start_time),read_tum(tum_file)['x'], color = trajectory_color[0], 
            linestyle=trajectory_linestyle,label='roll',linewidth=3)
    ax.plot(1*(read_tum(tum_file)['timestamp']-start_time),read_tum(tum_file)['y'], color = trajectory_color[1], 
            linestyle=trajectory_linestyle,label='pitch',linewidth=3)
    ax.plot(1*(read_tum(tum_file1)['timestamp']-start_time),read_tum(tum_file1)['z'], color = trajectory_color[2], 
            linestyle=trajectory_linestyle,label='yaw',linewidth=3)


    # 设置图例
    ax.legend(loc='upper left',fontsize=45)

    # 设置坐标轴线型
    ax.grid(True,linestyle='--')
    # ax.set_title('(j)Off2', loc='center', fontproperties=font_prop)
    # 设置坐标轴标签
    ax.set_xlabel(' ',fontproperties=font_prop,fontsize=35)
    ax.set_ylabel('  ',fontproperties=font_prop,fontsize=35)
    # ax.set_zlabel('Z (m)')
    ax.tick_params(axis='x', direction='in',labelsize=35,gridOn=True,size=5,pad=15)  # x轴样式
    ax.tick_params(axis='y', direction='in',labelsize=35,gridOn=True,size=5,pad=15)  # x轴样式
    # coord_text = ax.text(0.5, 0.95, '', transform=ax.transAxes, ha='center')
    for spine in ax.spines.values():
        spine.set_linewidth(3)
    
    ax.xaxis.set_major_locator(MultipleLocator(400))  # x轴每隔1一个刻度
    ax.yaxis.set_major_locator(MultipleLocator(0.1))  # y轴每隔5一个刻度
# 定义鼠标移动事件处理程序
    def on_move(event):
        if event.inaxes == ax:
            x = event.xdata
            y = event.ydata

# 连接鼠标移动事件
    fig.canvas.mpl_connect('motion_notify_event', on_move)

    ax.set_xlim(-75,900) 
    ax.set_ylim(-0.1, 0.1) 
    plt.show()

# 示例用法

tum_name = '/home/abin/Desktop/05_sample.tum'
tum_name1 = '/home/abin/Desktop/05_sample1.tum'

trajectories_color = ['#00FF00','#0000FF','#A020F0']
plot_trajectory(tum_name,tum_name1, background_color='white', trajectory_color=trajectories_color, trajectory_linestyle='-')

