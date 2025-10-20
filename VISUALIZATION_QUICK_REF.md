# 可视化快速参考

## 🚀 快速启动

### 运行完整可视化
```bash
python main-dynamic.py
```

### 测试可视化功能
```bash
python test_visualization.py
```

---

## 🎨 颜色速查表

| 颜色 | 英文 | 元素 | 时机 |
|------|------|------|------|
| 🩶 灰色 | GRAY | RRT*树 | 规划时 |
| 🔴 红色 | RED | 最优路径 | 规划完成 |
| 🔵 蓝色 | BLUE | B样条轨迹 | 平滑完成 |
| 🟡 黄色 | YELLOW | MPC预测 | 每个控制周期 |
| 🟢 绿色 | GREEN | 机器人 | 实时 |
| 🟣 紫色 | PURPLE | 障碍物 | 实时 |
| 🩵 青色 | CYAN | 投影点(PP) | Pure Pursuit模式 |
| 🟠 橙色 | ORANGE | 前视点(PP) | Pure Pursuit模式 |

---

## 🔧 开关控制

### main-dynamic.py 配置

```python
# 控制器选择
USE_MPC = True  # True=MPC, False=Pure Pursuit

# 可视化开关(可自定义添加)
SHOW_RRT_TREE = True
SHOW_FINAL_PATH = True
SHOW_BSPLINE = True
SHOW_PREDICTION = True  # MPC only
SHOW_OBSTACLES = True
```

---

## 📊 可视化层次

### 静态层(规划完成后不变)
```
灰色 RRT*树 → 红色最优路径 → 蓝色B样条
```

### 动态层(实时更新)
```
绿色机器人 + 黄色MPC预测 + 紫色障碍物
```

---

## 🐛 故障排查

### 看不到任何可视化
```bash
# 1. 检查调试器端口
netstat -an | findstr 20001

# 2. 测试调试器
python test_visualization.py

# 3. 检查防火墙
# 允许 UDP 20001
```

### 只看到部分颜色
```python
# 检查 Debug_Msg 颜色常量
from zss_debug_pb2 import Debug_Msg
print(Debug_Msg.RED)    # 应该是1
print(Debug_Msg.BLUE)   # 应该是6
print(Debug_Msg.YELLOW) # 应该是3
```

### MPC黄色轨迹不显示
```python
# 确保USE_MPC = True
# 检查debug_info中是否有predicted_states
if 'predicted_states' in debug_info:
    print(f"预测{len(debug_info['predicted_states'])}步")
else:
    print("MPC未返回预测轨迹!")
```

### Pure Pursuit跟踪点不显示
```python
# 确保USE_MPC = False
# 检查debug_info
if 'current_point' in debug_info:
    print("投影点:", debug_info['current_point'])
if 'target_point' in debug_info:
    print("前视点:", debug_info['target_point'])
```

---

## 📐 坐标系

```
          +Y (场地上方)
           |
           |
-X --------+-------- +X
(左)       |       (右)
           |
          -Y (场地下方)

单位: mm
范围: X[-5000, 5000], Y[-3500, 3500]
```

---

## 🎯 最佳实践

### 1. 分阶段调试
```python
# 第1阶段: 只看规划
SHOW_RRT_TREE = True
SHOW_FINAL_PATH = True
SHOW_BSPLINE = False
SHOW_PREDICTION = False

# 第2阶段: 检查平滑
SHOW_RRT_TREE = False  # 关闭树减少干扰
SHOW_FINAL_PATH = True
SHOW_BSPLINE = True

# 第3阶段: 观察控制
SHOW_RRT_TREE = False
SHOW_FINAL_PATH = False
SHOW_BSPLINE = True
SHOW_PREDICTION = True  # MPC预测
```

### 2. 性能优化
```python
# 降低可视化频率
frame_counter = 0
while True:
    # ... 控制逻辑 ...
    
    frame_counter += 1
    if frame_counter % 5 == 0:  # 每5帧可视化一次
        package = Debug_Msgs()
        # ... 绘制 ...
        debugger.send(package)
```

### 3. 录制分析
```python
# 保存可视化数据
vis_history = []

while running:
    # ... 控制 ...
    
    vis_data = {
        'time': time.time(),
        'robot_pos': current_pos.tolist(),
        'prediction': debug_info.get('predicted_states', [])
    }
    vis_history.append(vis_data)

# 保存到文件
import json
with open('visualization_log.json', 'w') as f:
    json.dump(vis_history, f)
```

---

## 📸 典型画面说明

### 初始规划完成
```
🩶🩶🩶 灰色树充满整个搜索空间
🔴🔴🔴 红色路径从树中穿过
🔵🔵🔵 蓝色曲线覆盖在红色路径上
```

### MPC动态避障
```
🔵 蓝色参考轨迹(目标)
🟡 黄色预测轨迹(实际规划)
🟣 紫色障碍物靠近
→ 黄色偏离蓝色避开紫色
→ 障碍物远离后黄色回归蓝色
```

### Pure Pursuit跟踪
```
🔵 蓝色轨迹
🩵 青色点在轨迹上(投影)
🟠 橙色点在青色前方300mm(前视)
🟢 绿色机器人朝橙色移动
```

---

## 🎓 进阶技巧

### 多路径对比
```python
# 绘制多条候选路径
for i, path in enumerate(candidate_paths):
    colors = [Debug_Msg.RED, Debug_Msg.ORANGE, Debug_Msg.YELLOW]
    debugger.draw_path(package, path, color=colors[i])
```

### 速度场可视化
```python
# 绘制速度矢量
for state in trajectory:
    x, y, theta = state
    vx = 500  # mm/s
    # 绘制箭头
    end_x = x + vx * np.cos(theta) * 0.1
    end_y = y + vx * np.sin(theta) * 0.1
    debugger.draw_line(package, x, y, end_x, end_y, color=Debug_Msg.GREEN)
```

### 代价场可视化
```python
# 用颜色表示代价
for x in range(-5000, 5000, 200):
    for y in range(-3500, 3500, 200):
        cost = compute_cost(x, y)
        if cost < 100:
            color = Debug_Msg.GREEN
        elif cost < 500:
            color = Debug_Msg.YELLOW
        else:
            color = Debug_Msg.RED
        debugger.draw_circle(package, x, y, 50, color=color)
```

---

## 📚 相关文档

- **详细说明**: `VISUALIZATION_GUIDE.md`
- **算法原理**: `MPC_GUIDE.md`
- **参数配置**: `CONTROLLER_CONFIG.md`
- **性能对比**: `CONTROLLER_COMPARISON.md`

---

## 💡 提示

1. **第一次运行**: 先运行`test_visualization.py`确认调试器工作
2. **调试时**: 关闭不需要的可视化元素减少干扰
3. **性能问题**: 降低可视化频率或减少RRT*节点显示
4. **录制演示**: 使用屏幕录制软件捕捉算法运行过程
5. **学习理解**: 对照`VISUALIZATION_GUIDE.md`理解每个颜色的含义

---

## ✅ 检查清单

运行前确认:
- [ ] 仿真器已启动
- [ ] 调试端口20001可访问
- [ ] 已选择控制器类型(USE_MPC)
- [ ] Python环境已激活
- [ ] 所有依赖已安装(scipy, numpy)

调试时检查:
- [ ] 灰色树是否显示(规划是否成功)
- [ ] 红色路径是否合理(是否避障)
- [ ] 蓝色曲线是否平滑(平滑是否过度)
- [ ] 黄色预测是否跟随蓝色(MPC跟踪性能)
- [ ] 绿色机器人是否在轨迹附近(控制效果)

---

**快速记忆口诀:**
```
灰树红路蓝样条,
黄预绿车紫障碍,
青投橙视纯跟踪。
```
