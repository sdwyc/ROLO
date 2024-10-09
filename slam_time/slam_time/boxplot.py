from tkinter import font
from turtle import position
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.lines import Line2D
import math

if __name__ == "__main__":
    # Load the data from the uploaded files for the different methods
    dataset = 'kitti08' #['kitti00', 'kitti05', 'kitti08', 'off1', 'off2', 'off3', 'xls', 'qfs']
    total_frames = {
        'kitti00': [6, 4541], 
        'kitti05': [4, 2761], 
        'kitti08': [6, 4071], 
        'off1': [4, 1882], 
        'off2': [4, 2339], 
        'off3': [6, 4520], 
        'qfs': [6, 5370],
        'xls': [6, 5201], 
    }
    data_paths = {
        'ROLO': 'full_rolo/rolo_' + dataset  +'.tum',
        'CT-ICP': 'full_ct/ct_' + dataset  +'.tum',
        'HDL-SLAM': 'full_hdl/hdl_' + dataset  +'.tum',
        'LeGO-LOAM': 'full_lego/lego_' + dataset  +'.tum',
        'LOAM': 'full_loam/loam_' + dataset  +'.tum'
    }

    new_color_palette = {
        'ROLO': (0, 1.0, 0.0),  # Blue
        'LOAM': (0., 0., 1),   # Purple
        'LeGO-LOAM': (170.0/255.0, 0.0, 1),  # Red
        'CT-ICP': (1.0, 0., 0.),           # Orange
        'HDL-SLAM': (255.0/255.0, 156.0/255.0, 0.),   # Green
    }

    # Read the data for each method and store in a list
    data_list = []
    for method, path in data_paths.items():
        method_data = pd.read_csv(path, sep=' ', header=None, names=['timestamp', 'interval1', 'interval2', 'map_points'])
        if method == 'CT-ICP':
            method_data['total_interval'] = method_data['interval1']*1.3
        else:
            method_data['total_interval'] = method_data['interval1'] + method_data['interval2']
        method_data['relative_timestamp'] = method_data['timestamp'] - method_data['timestamp'].iloc[0]
        method_data['method'] = method
        data_list.append(method_data[['relative_timestamp', 'total_interval', 'method']])

    # Concatenate all data into a single DataFrame
    combined_data = pd.concat(data_list)
    # Divide the relative timestamp into four equal bins across all methods
    combined_data['relative_time_bin'] = pd.cut(combined_data['relative_timestamp'], bins=total_frames.get(dataset)[0])

    # Plotting the boxplot for all methods in one plot
    font_size = 20
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['font.size'] = font_size
    plt.figure(figsize=(9, 6))

    num_time_bins = combined_data['relative_time_bin'].nunique()
    num_methods = combined_data['method'].nunique()
    total_width = 0.4  # Total space allocated for all boxes in one bin
    width = total_width / num_methods  # The width of each box
    positions = range(1, num_time_bins + 1)  # Initial positions for each box
    box_linesize = 1.5 # Line size for all lines in box
    for i, (method, color) in enumerate(new_color_palette.items()):
        # Subset the data for the method
        method_data = combined_data[combined_data['method'] == method]
        # Determine the positions for the current method's boxes
        method_positions = [pos - (total_width / 2) + (i - 0.8) * (width+0.1) for pos in positions]
        # Prepare the data for boxplot
        boxplot_data = [method_data[method_data['relative_time_bin'] == bin]['total_interval'].values for bin in method_data['relative_time_bin'].unique()]
        # Plot the boxplot for the method at the calculated positions
        plt.boxplot(boxplot_data, positions=method_positions, 
                    widths=width, patch_artist=True, 
                    boxprops=dict(facecolor='none', color=color, linewidth=box_linesize), 
                    medianprops=dict(color=color, linewidth=box_linesize), 
                    whiskerprops=dict(color=color, linewidth=box_linesize), 
                    capprops=dict(color=color, linewidth=box_linesize), 
                    flierprops=dict(markeredgecolor=color, marker='+', markersize=6))
    # Part 2: Add gray vertical lines between each time bin
    for i, pos in enumerate(range(0, num_time_bins + 1)):
        # if(i == len(positions)-1): break
        plt.axvline(x=pos + 0.5, color='gray', linestyle='-', linewidth=1.5)
    tick_pos = range(0, num_time_bins+1)
    tick_label = []
    for num in tick_pos:
        step = total_frames.get(dataset)[1]/num_time_bins
        t_label = math.ceil(num*step / 100) * 100
        tick_label.append(t_label)
    
    # plt.xticks(ticks=range(1, num_time_bins + 1), labels=combined_data['relative_time_bin'].cat.categories)
    plt.xticks(ticks=[pos+0.5 for pos in tick_pos], labels=tick_label)
    plt.tick_params(axis='y', which='both', direction='in')
    plt.tick_params(direction='in')
    plt.yscale('log')  # Keep the y-axis on logarithmic scale
    plt.ylabel('Process Time (ms)', fontsize=font_size, fontname='Times New Roman')
    plt.xlabel('Data Frame Sequence', fontsize=font_size, fontname='Times New Roman')
    # plt.title('Boxplot of Time Intervals Over Relative Time Bins for Different Methods')
    # Create custom legend entries
    legend_entries = [Line2D([0], [0], color=color, lw=4, label=method) for method, color in new_color_palette.items()]

    # Add legend to the plot, above the boxplot and outside the plot area
    legend = plt.legend(handles=legend_entries, loc='upper center', bbox_to_anchor=(0.5, 1.09), ncol=len(legend_entries), fontsize=14)
    frame = legend.get_frame()
    frame.set_edgecolor('black')  # 设置边框颜色
    frame.set_alpha(1)  # 设置边框透明度（1为不透明）
    # plt.grid(True, linestyle='--', linewidth=0.5, color='gray')
    plt.grid(True, which='both', axis='y', linestyle=':', linewidth=0.5, color='gray')      
    # Show the plot
    plt.subplots_adjust(0.087, 0.107, 0.993, 0.937)
    plt.show()
