"""
测试可视化功能 - 不需要真实机器人
"""
import numpy as np
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs, Debug_Msg
from scipy.interpolate import splprep, splev

class MockRRTPlanner:
    """模拟RRT*规划器"""
    class Node:
        def __init__(self, x, y, parent=None):
            self.x = x
            self.y = y
            self.parent = parent
    
    def __init__(self):
        # 创建模拟的搜索树
        self.node_list = []
        
        # 根节点
        root = self.Node(0, 0)
        self.node_list.append(root)
        
        # 生成随机树
        np.random.seed(42)
        for i in range(100):
            # 随机新节点
            x = np.random.uniform(-2000, 4000)
            y = np.random.uniform(-1000, 1000)
            
            # 随机选择父节点
            parent_idx = np.random.randint(0, len(self.node_list))
            parent = self.node_list[parent_idx]
            
            # 创建新节点
            node = self.Node(x, y, parent)
            self.node_list.append(node)

def test_visualization():
    """测试所有可视化功能"""
    debugger = Debugger()
    package = Debug_Msgs()
    
    print("=" * 60)
    print("可视化功能测试")
    print("=" * 60)
    
    # 1. 测试RRT*树绘制(灰色)
    print("\n1. 测试RRT*树绘制...")
    mock_planner = MockRRTPlanner()
    debugger.draw_rrt_tree(package, mock_planner)
    print(f"   ✓ 绘制了{len(mock_planner.node_list)}个节点的树结构(灰色)")
    
    # 2. 测试路径绘制(红色)
    print("\n2. 测试最终路径绘制...")
    path = [
        [0, 0],
        [500, 200],
        [1000, 500],
        [1500, 800],
        [2000, 900],
        [2500, 950],
        [3000, 1000]
    ]
    debugger.draw_path(package, path, color=Debug_Msg.RED)
    print(f"   ✓ 绘制了{len(path)}个点的路径(红色)")
    
    # 3. 测试B样条轨迹绘制(蓝色)
    print("\n3. 测试B样条轨迹绘制...")
    path_array = np.array(path)
    tck, u = splprep([path_array[:, 0], path_array[:, 1]], s=0, k=3)
    debugger.draw_trajectory(package, tck, num_points=100, color=Debug_Msg.BLUE)
    print("   ✓ 绘制了B样条参考轨迹(蓝色)")
    
    # 4. 测试MPC预测轨迹(黄色)
    print("\n4. 测试MPC预测轨迹...")
    predicted_states = []
    for i in range(10):
        x = 1000 + i * 100
        y = 500 + 50 * np.sin(i * 0.3)
        theta = 0.3 * i
        predicted_states.append([x, y, theta])
    debugger.draw_prediction(package, predicted_states, color=Debug_Msg.YELLOW)
    print(f"   ✓ 绘制了{len(predicted_states)}步预测轨迹(黄色)")
    
    # 5. 测试机器人位置(绿色)
    print("\n5. 测试机器人位置...")
    debugger.draw_circle(package, 1500, 600, 100, color=Debug_Msg.GREEN)
    print("   ✓ 绘制了机器人位置圆(绿色)")
    
    # 6. 测试障碍物(紫色)
    print("\n6. 测试障碍物...")
    obstacles = [
        (800, 300, 150),
        (1800, 700, 150),
        (2500, 400, 150)
    ]
    for ox, oy, radius in obstacles:
        debugger.draw_circle(package, ox, oy, radius, color=Debug_Msg.PURPLE)
    print(f"   ✓ 绘制了{len(obstacles)}个障碍物圆(紫色)")
    
    # 7. 测试Pure Pursuit跟踪点
    print("\n7. 测试Pure Pursuit跟踪点...")
    debugger.draw_point(package, 1400, 580, color=Debug_Msg.CYAN)  # 投影点
    debugger.draw_point(package, 1700, 650, color=Debug_Msg.ORANGE)  # 前视点
    print("   ✓ 绘制了投影点(青色)和前视点(橙色)")
    
    # 8. 发送所有可视化
    print("\n8. 发送可视化包...")
    debugger.send(package)
    print("   ✓ 已发送到调试端口(localhost:20001)")
    
    # 统计信息
    print("\n" + "=" * 60)
    print("统计信息:")
    print(f"  - 总消息数: {len(package.msgs)}")
    print(f"  - RRT*节点: {len(mock_planner.node_list)}")
    print(f"  - 路径点数: {len(path)}")
    print(f"  - 预测步数: {len(predicted_states)}")
    print(f"  - 障碍物数: {len(obstacles)}")
    print("=" * 60)
    
    # 颜色说明
    print("\n颜色说明:")
    colors = [
        ("灰色 (GRAY)", "RRT*搜索树"),
        ("红色 (RED)", "RRT*最终路径"),
        ("蓝色 (BLUE)", "B样条参考轨迹"),
        ("黄色 (YELLOW)", "MPC预测轨迹"),
        ("绿色 (GREEN)", "机器人位置"),
        ("紫色 (PURPLE)", "动态障碍物"),
        ("青色 (CYAN)", "Pure Pursuit投影点"),
        ("橙色 (ORANGE)", "Pure Pursuit前视点")
    ]
    for color, desc in colors:
        print(f"  {color:20s} - {desc}")
    
    print("\n✅ 所有可视化测试完成!")
    print("   请在仿真界面查看效果")
    print("   如果看不到,请检查:")
    print("   1. 仿真器是否运行在localhost:20001")
    print("   2. 防火墙是否允许UDP通信")
    print("   3. 视角是否正确(坐标范围-5000~5000)")

if __name__ == '__main__':
    test_visualization()
