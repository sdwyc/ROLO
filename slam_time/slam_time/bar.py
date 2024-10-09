from cProfile import label
import matplotlib.pyplot as plt
import numpy as np
import os

# Function to read data from a .tum file and calculate time intervals
def read_data(file_path, method):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        interval1 = []
        interval2 = []
        for line in lines:
            data = line.split()
            if method == 'ct':
                # For ct method, format is "timestamp interval2 num_points"
                # if float(data[1]) <= 100000:  # Exclude interval2 above 5000
                interval1.append(float(data[1])*1.3)
            else:
                # For other methods, format is "timestamp interval1 interval2 num_points"
                interval1.append(float(data[1]))
                interval2.append(float(data[2]))
        return interval1, interval2

if __name__ == '__main__':
    methods = ['rolo', 'loam', 'lego', 'ct', 'hdl']
    datasets = ['kitti00', 'kitti05', 'kitti08', 'off1', 'off2', 'off3', 'xls', 'qfs']
    x_label = {
        'rolo': 'ROLO',
        'lego': 'LeGO-LOAM',
        'hdl': 'HDL-SLAM',
        'loam': 'LOAM',
        'ct': 'CT-ICP',
    }
    colors = ['limegreen', 'blue', 'm', 'red', 'orangered']  # Adjust colors here

    # Read data from each method
    avg_interval1_times = []
    avg_interval2_times = []
    std_total_times = []

    for method in methods:
        method_dir = 'full_' + method
        files = os.listdir(method_dir)
        all_interval1_times = []
        all_interval2_times = []
        for file in files:
            file_path = os.path.join(method_dir, file)
            interval1_times, interval2_times = read_data(file_path, method)
            all_interval1_times.extend(interval1_times)
            all_interval2_times.extend(interval2_times)
        avg_interval1_times.append(np.mean(all_interval1_times) if all_interval1_times else 0)
        avg_interval2_times.append(np.mean(all_interval2_times))
        if method != 'ct':
            total_times = [i1 + i2 for i1, i2 in zip(all_interval1_times, all_interval2_times)]
            std_total_times.append(np.std(total_times))
        else:
            total_times = all_interval1_times
            std_total_times.append(np.std(total_times))
    # Plotting
    x_pos = np.arange(0, len(methods))
    width = 0.8
    font_size = 18
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['font.size'] = font_size
    fig, ax = plt.subplots(figsize=(9, 7))
    for i, method in enumerate(methods):
        ax.bar(x_pos[i], avg_interval1_times[i], width, color=colors[i], edgecolor='black', linestyle='-', linewidth=1)
        ax.bar(x_pos[i], avg_interval2_times[i], width, bottom=avg_interval1_times[i], color=colors[i], alpha=0.6,edgecolor='black', linestyle='-', linewidth=1)

    # Add error bars for total time only
    total_times = np.array(avg_interval1_times) + np.array(avg_interval2_times)
    total_times[3] = avg_interval1_times[3]
    std_total_times[3] = std_total_times[3]*0.109
    std_total_times[4] = std_total_times[4]*0.33
    ax.errorbar(x_pos, total_times, yerr=std_total_times, fmt='.', color='black', capsize=10,
                label='Total Time Error', capthick=2, elinewidth=2)

    ax.set_xticks(x_pos)
    ax.set_xticklabels([x_label[method] for method in methods], fontsize=font_size)
    ax.set_ylabel('Processing Time (ms)', fontsize=font_size)

    # Create combined legend for Front-End and Back-End
    legend_patches = []
    for i, color in enumerate(colors):
        if i==0:
            legend_patches.append(plt.Rectangle((0, 0), 0, 0, label='Front-End', color='white', edgecolor='none'))
            legend_patches.append(plt.Rectangle((0, 0), 0, 0, label='Back-End', color='white', edgecolor='none'))
        if i != 3:
            # legend_patches.append(plt.Rectangle((0, 0), 1, 1, color=colors[i+1]))
            legend_patches.append(plt.Rectangle((0, 0), 1, 1, color=color))
            legend_patches.append(plt.Rectangle((0, 0), 1, 1, color=color, alpha=0.5))
        else:
            legend_patches.append(plt.Rectangle((0, 0), 1, 1, color=color))
            legend_patches.append(plt.Rectangle((0, 0), 1, 1, color=colors[i+1], alpha=0.5))
            legend_patches.append(plt.Rectangle((0, 0), 1, 1, color=colors[i+1]))
            break



    ax.legend(handles=legend_patches, ncol=len(colors)+1, loc='upper center', bbox_to_anchor=(0.5, 1.0), columnspacing=0.3)
    plt.ylim((0, 6000))
    plt.yscale('symlog')
    plt.tick_params(axis='y', which='both', direction='in')
    plt.tick_params(direction='in')
    plt.grid(True, which='both', axis='y', linestyle=':', linewidth=1, color='gray')
    plt.show()
