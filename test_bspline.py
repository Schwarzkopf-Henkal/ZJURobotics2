"""
B样条平滑功能测试脚本
独立测试B样条平滑器，不需要连接机器人
"""

import numpy as np
import matplotlib.pyplot as plt
from bspline_smoother import BSplineSmoother, VelocityProfiler


def test_bspline_smoothing():
    """测试B样条平滑功能"""
    
    print("=" * 60)
    print("B样条路径平滑测试")
    print("=" * 60)
    
    # 创建测试路径（模拟RRT*输出）
    # 一条曲折的路径
    test_path = [
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
    
    # 创建障碍物
    obstacles = [
        (1000, 500, 150),
        (2000, 100, 150),
        (3000, 800, 150)
    ]
    
    # 初始化平滑器
    smoother = BSplineSmoother(robot_radius=100)
    
    # 测试不同的平滑参数
    smoothing_params = [
        (0, 3, "插值模式 (s=0, k=3)"),
        (100, 3, "轻度平滑 (s=100, k=3)"),
        (500, 3, "中度平滑 (s=500, k=3)"),
    ]
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('B样条路径平滑测试', fontsize=16, fontproperties='SimHei')
    
    for idx, (s, k, label) in enumerate(smoothing_params):
        ax = axes[idx // 2, idx % 2]
        
        # 平滑路径
        smoothed = smoother.smooth_path(
            test_path, 
            obstacles, 
            s=s, 
            k=k, 
            num_points=100
        )
        
        # 绘制
        # 原始路径
        original_x = [p[0] for p in test_path]
        original_y = [p[1] for p in test_path]
        ax.plot(original_x, original_y, 'b--o', linewidth=2, 
                label='原始路径', alpha=0.6, markersize=8)
        
        # 平滑路径
        smooth_x = [p[0] for p in smoothed]
        smooth_y = [p[1] for p in smoothed]
        ax.plot(smooth_x, smooth_y, 'r-', linewidth=2, 
                label='平滑路径', alpha=0.8)
        
        # 障碍物
        for ox, oy, radius in obstacles:
            circle = plt.Circle((ox, oy), radius, color='gray', alpha=0.5)
            ax.add_patch(circle)
            ax.plot(ox, oy, 'kx', markersize=10)
        
        # 起点和终点
        ax.plot(test_path[0][0], test_path[0][1], 'go', 
                markersize=15, label='起点')
        ax.plot(test_path[-1][0], test_path[-1][1], 'r*', 
                markersize=20, label='终点')
        
        ax.set_xlabel('X (mm)', fontproperties='SimHei')
        ax.set_ylabel('Y (mm)', fontproperties='SimHei')
        ax.set_title(label, fontproperties='SimHei')
        ax.legend(prop={'family': 'SimHei'})
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # 统计信息
        original_length = sum(
            np.linalg.norm(np.array(test_path[i+1]) - np.array(test_path[i]))
            for i in range(len(test_path) - 1)
        )
        smooth_length = sum(
            np.linalg.norm(np.array(smoothed[i+1]) - np.array(smoothed[i]))
            for i in range(len(smoothed) - 1)
        )
        
        print(f"\n{label}:")
        print(f"  原始路径: {len(test_path)} 点, 长度 {original_length:.1f}mm")
        print(f"  平滑路径: {len(smoothed)} 点, 长度 {smooth_length:.1f}mm")
        print(f"  长度变化: {(smooth_length/original_length - 1)*100:+.1f}%")
    
    # 第四个子图：曲率分析
    ax = axes[1, 1]
    smoothed = smoother.smooth_path(test_path, obstacles, s=0, k=3, num_points=100)
    
    # 计算曲率（简化版）
    curvatures = []
    for i in range(1, len(smoothed) - 1):
        p1 = np.array(smoothed[i-1])
        p2 = np.array(smoothed[i])
        p3 = np.array(smoothed[i+1])
        
        v1 = p2 - p1
        v2 = p3 - p2
        
        # 使用向量叉积估计曲率
        cross = np.abs(v1[0]*v2[1] - v1[1]*v2[0])
        norm_product = np.linalg.norm(v1) * np.linalg.norm(v2)
        
        if norm_product > 1e-6:
            curvature = cross / norm_product
        else:
            curvature = 0
        
        curvatures.append(curvature)
    
    ax.plot(curvatures, 'b-', linewidth=2)
    ax.set_xlabel('路径点索引', fontproperties='SimHei')
    ax.set_ylabel('曲率 (归一化)', fontproperties='SimHei')
    ax.set_title('路径曲率分析 (越低越平滑)', fontproperties='SimHei')
    ax.grid(True, alpha=0.3)
    
    print(f"\n曲率统计:")
    print(f"  平均曲率: {np.mean(curvatures):.4f}")
    print(f"  最大曲率: {np.max(curvatures):.4f}")
    print(f"  曲率标准差: {np.std(curvatures):.4f}")
    
    plt.tight_layout()
    plt.savefig('bspline_test_result.png', dpi=150)
    print(f"\n图表已保存到: bspline_test_result.png")
    plt.show()


def test_velocity_profile():
    """测试速度规划功能"""
    
    print("\n" + "=" * 60)
    print("速度规划测试")
    print("=" * 60)
    
    # 简单路径
    path = [[i*100, 0] for i in range(50)]
    
    # 创建速度规划器
    profiler = VelocityProfiler(
        max_vel=1000,
        max_acc=500,
        max_angular_vel=3.0
    )
    
    # 计算速度曲线
    profile = profiler.compute_velocity_profile(path, dt=0.01)
    
    # 提取数据
    positions = [p[0] for p in profile]
    velocities = [p[2] for p in profile]
    times = [p[3] for p in profile]
    
    # 绘制
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # 速度-时间曲线
    ax1.plot(times, velocities, 'b-', linewidth=2)
    ax1.axhline(y=1000, color='r', linestyle='--', label='最大速度')
    ax1.set_xlabel('时间 (s)', fontproperties='SimHei')
    ax1.set_ylabel('速度 (mm/s)', fontproperties='SimHei')
    ax1.set_title('梯形速度曲线', fontproperties='SimHei')
    ax1.legend(prop={'family': 'SimHei'})
    ax1.grid(True, alpha=0.3)
    
    # 速度-位置曲线
    ax2.plot(positions, velocities, 'g-', linewidth=2)
    ax2.axhline(y=1000, color='r', linestyle='--', label='最大速度')
    ax2.set_xlabel('位置 (mm)', fontproperties='SimHei')
    ax2.set_ylabel('速度 (mm/s)', fontproperties='SimHei')
    ax2.set_title('速度随位置变化', fontproperties='SimHei')
    ax2.legend(prop={'family': 'SimHei'})
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('velocity_profile_test.png', dpi=150)
    print(f"\n图表已保存到: velocity_profile_test.png")
    plt.show()
    
    # 统计
    total_time = times[-1]
    total_dist = positions[-1]
    avg_vel = total_dist / total_time if total_time > 0 else 0
    
    print(f"\n速度规划统计:")
    print(f"  总距离: {total_dist:.1f} mm")
    print(f"  总时间: {total_time:.2f} s")
    print(f"  平均速度: {avg_vel:.1f} mm/s")
    print(f"  最大速度: {max(velocities):.1f} mm/s")


def normalize_angle(angle):
    """将角度归一化到[-pi, pi]"""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def compute_control_command(current_pos, current_orientation, target_pos, target_vel=1000):
    """计算控制指令（简化版用于测试）"""
    dx = target_pos[0] - current_pos[0]
    dy = target_pos[1] - current_pos[1]
    target_angle = np.arctan2(dy, dx)
    angle_error = normalize_angle(target_angle - current_orientation)
    
    Kp_angular = 4.0
    vw = Kp_angular * angle_error
    vw = np.clip(vw, -3.0, 3.0)
    
    angle_factor = max(0, np.cos(angle_error))
    distance = np.sqrt(dx**2 + dy**2)
    distance_factor = min(1.0, distance / 500)
    
    vx = target_vel * angle_factor * distance_factor
    vx = np.clip(vx, 0, 1000)
    
    if abs(angle_error) > np.pi / 3:
        vx = 0
    
    return vx, vw


def test_control_model():
    """测试控制模型"""
    
    print("\n" + "=" * 60)
    print("差速驱动控制模型测试")
    print("=" * 60)
    
    # 测试场景
    test_cases = [
        {
            'name': '正前方目标',
            'current_pos': [0, 0],
            'current_orient': 0,
            'target_pos': [1000, 0]
        },
        {
            'name': '左侧目标 (90度)',
            'current_pos': [0, 0],
            'current_orient': 0,
            'target_pos': [0, 1000]
        },
        {
            'name': '右侧目标 (-90度)',
            'current_pos': [0, 0],
            'current_orient': 0,
            'target_pos': [0, -1000]
        },
        {
            'name': '后方目标 (180度)',
            'current_pos': [0, 0],
            'current_orient': 0,
            'target_pos': [-1000, 0]
        },
        {
            'name': '斜向目标 (45度)',
            'current_pos': [0, 0],
            'current_orient': 0,
            'target_pos': [1000, 1000]
        }
    ]
    
    print("\n控制指令测试结果:")
    print("-" * 60)
    
    for case in test_cases:
        vx, vw = compute_control_command(
            case['current_pos'],
            case['current_orient'],
            case['target_pos']
        )
        
        # 计算目标角度
        dx = case['target_pos'][0] - case['current_pos'][0]
        dy = case['target_pos'][1] - case['current_pos'][1]
        target_angle = np.arctan2(dy, dx)
        angle_error = normalize_angle(target_angle - case['current_orient'])
        
        print(f"\n{case['name']}:")
        print(f"  目标角度: {np.degrees(target_angle):.1f}°")
        print(f"  角度误差: {np.degrees(angle_error):.1f}°")
        print(f"  线速度 vx: {vx:.1f} mm/s")
        print(f"  角速度 vw: {vw:.2f} rad/s ({np.degrees(vw):.1f}°/s)")


if __name__ == '__main__':
    try:
        # 测试1: B样条平滑
        test_bspline_smoothing()
        
        # 测试2: 速度规划
        test_velocity_profile()
        
        # 测试3: 控制模型
        test_control_model()
        
        print("\n" + "=" * 60)
        print("所有测试完成！")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
