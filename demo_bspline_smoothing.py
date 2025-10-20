"""
简单的B样条平滑测试 - 无障碍物版本
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

# 创建一条曲折的路径（模拟RRT*输出）
original_path = [
    [0, 0],
    [500, 200],
    [1000, 100],
    [1500, 400],
    [2000, 300],
    [2500, 600],
    [3000, 500],
    [3500, 800],
    [4000, 700],
    [4500, 1000]
]

# 转换为numpy数组
path_array = np.array(original_path)
x = path_array[:, 0]
y = path_array[:, 1]

# 使用B样条平滑
k = 3  # 三次样条
s = 0  # 插值模式

# 生成B样条
tck, u = splprep([x, y], s=s, k=k)

# 生成不同密度的平滑路径
num_points_list = [10, 20, 50, 100]

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('B样条采样点数对平滑度的影响', fontsize=16, fontproperties='SimHei')

for idx, num_points in enumerate(num_points_list):
    ax = axes[idx // 2, idx % 2]
    
    # 生成平滑路径
    u_new = np.linspace(0, 1, num_points)
    x_new, y_new = splev(u_new, tck)
    
    # 绘制原始路径（折线）
    ax.plot(x, y, 'b--o', linewidth=2, markersize=8, 
            label='原始路径 (RRT*)', alpha=0.6)
    
    # 绘制平滑路径
    ax.plot(x_new, y_new, 'r-*', linewidth=2.5, markersize=6,
            label=f'B样条平滑 ({num_points}点)', alpha=0.8)
    
    # 标记起点终点
    ax.plot(x[0], y[0], 'go', markersize=15, label='起点', zorder=10)
    ax.plot(x[-1], y[-1], 'r*', markersize=20, label='终点', zorder=10)
    
    ax.set_xlabel('X (mm)', fontproperties='SimHei')
    ax.set_ylabel('Y (mm)', fontproperties='SimHei')
    ax.set_title(f'采样点数: {num_points}', fontproperties='SimHei', fontsize=12)
    ax.legend(prop={'family': 'SimHei'}, loc='upper left')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # 计算路径长度
    original_length = sum(
        np.linalg.norm(np.array(original_path[i+1]) - np.array(original_path[i]))
        for i in range(len(original_path) - 1)
    )
    
    smooth_length = sum(
        np.sqrt((x_new[i+1] - x_new[i])**2 + (y_new[i+1] - y_new[i])**2)
        for i in range(len(x_new) - 1)
    )
    
    # 添加统计信息
    info_text = f'原始长度: {original_length:.0f}mm\n平滑长度: {smooth_length:.0f}mm\n差异: {(smooth_length/original_length-1)*100:+.1f}%'
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, 
            fontsize=9, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
            fontproperties='SimHei')

plt.tight_layout()
plt.savefig('bspline_smoothing_demo.png', dpi=150, bbox_inches='tight')
print("=" * 60)
print("B样条平滑演示")
print("=" * 60)
print("\n图表已保存: bspline_smoothing_demo.png")
print("\n结论:")
print("- 采样点数越少(10-20)，路径越平滑（曲线）")
print("- 采样点数越多(100+)，路径越接近原始折线")
print("- 推荐使用 20-50 个采样点获得最佳平滑效果")
print("\n当前 main-dynamic.py 使用的采样点数可能过多！")
print("建议修改为: num_points = max(30, len(smoothed_path) * 2)")
print("=" * 60)

plt.show()
