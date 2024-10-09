import matplotlib.pyplot as plt

fig, ax = plt.subplots()
# 绘制一条曲线
ax.plot([1, 2, 3], label='Line 1')

# 创建一个空白的图例对象
legend = ax.legend([], loc='upper right', bbox_to_anchor=(0, 0), frameon=True)

# 设置图例的边界框大小为1x1
legend.set_bbox_to_anchor((0, 0, 1, 1))

plt.show()
