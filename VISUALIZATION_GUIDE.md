# 算法可视化指南

## 可视化总览

`main-dynamic.py` 现在支持完整的实时算法可视化,通过调试器(Debugger)绘制到仿真界面。

---

## 🎨 颜色方案

| 颜色 | 元素 | 含义 |
|------|------|------|
| **灰色 (GRAY)** | RRT*树 | 所有探索的节点和连接 |
| **红色 (RED)** | RRT*最终路径 | 从RRT*选出的最优路径 |
| **蓝色 (BLUE)** | B样条参考轨迹 | 平滑后的目标轨迹 |
| **黄色 (YELLOW)** | MPC预测轨迹 | MPC预测的未来N步路径 |
| **绿色 (GREEN)** | 机器人位置 | 当前机器人所在位置 |
| **紫色 (PURPLE)** | 动态障碍物 | 其他机器人的位置和半径 |
| **青色 (CYAN)** | 当前投影点 | Pure Pursuit的轨迹投影点 |
| **橙色 (ORANGE)** | 前视目标点 | Pure Pursuit的lookahead点 |

---

## 📊 可视化层次

### 第1层: 规划层(静态)

#### 1.1 RRT*探索树 (灰色)
```python
debugger.draw_rrt_tree(package, planning)
```

**显示内容:**
- 所有探索的节点
- 节点间的父子连接关系
- 完整的搜索树结构

**特点:**
- 灰色细线,低亮度
- 展示算法的探索过程
- 通常包含数百到数千个节点

**分析要点:**
- 树的密度反映搜索强度
- 分支方向显示探索偏好
- 可判断障碍物周围的搜索困难程度

---

#### 1.2 RRT*最优路径 (红色)
```python
debugger.draw_path(package, original_path, color=Debug_Msg.RED)
```

**显示内容:**
- RRT*回溯得到的最优路径
- 从起点到终点的完整序列

**特点:**
- 红色粗线,高亮显示
- 是从灰色树中提取的最优解
- 通常呈现折线状(未平滑)

**分析要点:**
- 路径是否避开障碍物
- 路径长度是否合理
- 是否有不必要的绕行

---

#### 1.3 B样条参考轨迹 (蓝色)
```python
debugger.draw_trajectory(package, tck, num_points=200, color=Debug_Msg.BLUE)
```

**显示内容:**
- B样条平滑后的连续曲线
- 通过200个采样点绘制

**特点:**
- 蓝色平滑曲线
- 消除了红色路径的折线转角
- 保持了原路径的大致走向

**分析要点:**
- 曲线是否平滑连续
- 是否偏离原路径太远
- 是否仍然避开障碍物

---

### 第2层: 控制层(动态)

#### 2.1 MPC预测轨迹 (黄色)
```python
debugger.draw_prediction(package, predicted_states, color=Debug_Msg.YELLOW)
```

**显示内容:**
- MPC预测的未来N步(通常10步)
- 每步0.1秒,共预测1秒

**特点:**
- 黄色短曲线
- 实时更新,跟随机器人移动
- 显示MPC的"预见性"

**分析要点:**
- 预测轨迹是否跟随蓝色参考轨迹
- 遇到障碍物时是否提前避让
- 曲线平滑度(反映控制平滑性)

**与蓝色轨迹对比:**
```
蓝色(参考): 理想的全局轨迹,不考虑动态障碍
黄色(预测): 实时优化的局部轨迹,考虑动态避障
```

---

#### 2.2 Pure Pursuit跟踪点
**当前投影点 (青色):**
```python
debugger.draw_point(package, cp[0], cp[1], color=Debug_Msg.CYAN)
```
- 机器人在轨迹上的最近投影点
- 表示"我在轨迹的哪个位置"

**前视目标点 (橙色):**
```python
debugger.draw_point(package, tp[0], tp[1], color=Debug_Msg.ORANGE)
```
- 从投影点向前lookahead_distance的点
- 表示"我要朝哪里走"

**分析要点:**
- 青色点应该在蓝色轨迹上或附近
- 橙色点应该在青色点前方
- lookahead距离决定转弯的提前量

---

### 第3层: 环境层(实时)

#### 3.1 机器人位置 (绿色)
```python
debugger.draw_circle(package, robot.x, robot.y, 100, color=Debug_Msg.GREEN)
```

**显示内容:**
- 自己机器人的实时位置
- 半径100mm的圆圈

**分析要点:**
- 是否跟随蓝色/黄色轨迹移动
- 偏离距离是否在合理范围(<200mm)

---

#### 3.2 动态障碍物 (紫色)
```python
for obs_id, (ox, oy, radius) in obstacles_dict.items():
    debugger.draw_circle(package, ox, oy, radius, color=Debug_Msg.PURPLE)
```

**显示内容:**
- 其他蓝色机器人
- 对方黄色机器人
- 半径150mm(含安全裕度)

**分析要点:**
- MPC黄色轨迹是否主动绕开紫色圆
- 安全距离是否足够

---

## 🔍 算法运行过程分析

### MPC模式完整可视化

**初始阶段(路径规划):**
```
1. 灰色树展开 → 显示RRT*探索
2. 红色路径出现 → RRT*找到解
3. 蓝色曲线覆盖 → B样条平滑完成
```

**运行阶段(轨迹跟踪):**
```
每个控制周期(~10ms):
1. 绿色圆移动 → 机器人位置更新
2. 黄色预测更新 → MPC重新优化
3. 紫色圆变化 → 障碍物移动
4. 黄色避开紫色 → 动态避障生效
```

---

## 📐 可视化数学解释

### RRT*树的密度
```
节点密度 = len(node_list) / 搜索区域面积

密度高 → 探索充分,路径更优
密度低 → 快速求解,可能次优
```

### 路径平滑度量
```python
# 红色路径(RRT*)
转折角度: 通常 30-90度
路径段数: 10-30段

# 蓝色轨迹(B样条)
曲率: 连续可导
路径段数: 50-200段(视觉上平滑)
```

### MPC预测误差
```
蓝色与黄色的距离:
- <100mm: 跟踪良好
- 100-300mm: 避障导致偏离
- >300mm: 参数需调优或障碍太密
```

---

## 🎬 动画效果说明

### 静态元素(不变)
- 灰色RRT*树
- 红色最优路径

### 半动态元素(每控制周期重绘)
- 蓝色B样条轨迹(固定形状,重复绘制)

### 动态元素(实时变化)
- 绿色机器人(跟随真实位置)
- 黄色MPC预测(每次优化结果不同)
- 紫色障碍物(随对方机器人移动)

---

## 🐛 调试技巧

### 1. RRT*没找到路径
**现象:** 只有灰色树,没有红色路径
**检查:**
```python
# 增加迭代次数
max_iter=2000 → 5000

# 减小扩展步长
expand_dis=800 → 400

# 增大机器人半径裕度
robot_radius=100 → 150
```

---

### 2. B样条偏离太多
**现象:** 蓝色曲线远离红色路径
**检查:**
```python
# 减小平滑参数s
s=0  # 插值模式,强制通过所有点
s=100 → s=10

# 减小采样点数
num_points=200 → 100
```

---

### 3. MPC抖动
**现象:** 黄色预测轨迹剧烈震荡
**检查权重:**
```python
# 增加平滑权重
R_dvx=1.0 → 3.0
R_dvw=1.0 → 3.0

# 减小跟踪权重
Q_position=100 → 50
```

---

### 4. MPC撞障碍物
**现象:** 黄色轨迹穿过紫色圆
**检查:**
```python
# 增大避障权重
Q_obstacle=1000 → 2000

# 增大安全距离
safety_distance=100 → 200

# 检查约束是否生效
if len(constraints) > 0:
    print("Constraints active")
```

---

### 5. Pure Pursuit走偏
**现象:** 绿色圆偏离蓝色轨迹
**检查:**
```python
# 减小前视距离(增强跟踪)
lookahead_distance=300 → 200

# 增大角度增益
kp_angular=4.0 → 6.0

# 检查投影点
if dist(cyan_point, blue_trajectory) > 200:
    print("Projection error!")
```

---

## 📈 性能监控

### 可视化频率
```python
# 当前实现
控制频率: ~100Hz (0.01s)
可视化频率: 100Hz (每个控制周期)

# 优化建议(如果卡顿)
可视化频率: 20Hz (每5个控制周期)
if frame_counter % 5 == 0:
    debugger.send(package)
```

### 绘制开销
```
RRT*树: ~O(N), N=节点数, 通常1000-3000
最终路径: O(P), P=路径点数, 通常10-30
B样条: O(S), S=采样点数, 200
MPC预测: O(10), 固定10步
障碍物: O(M), M=机器人数, 通常<10

总复杂度: O(N + P + S + 10 + M) ≈ O(N)
```

---

## 🎓 教学用途

### 演示1: RRT*探索过程
1. 慢速运行,观察灰色树的生长
2. 看树如何"试探"各个方向
3. 最终红色路径从树中"浮现"

### 演示2: 路径平滑的必要性
1. 对比红色折线 vs 蓝色曲线
2. 想象机器人走折线会频繁急转
3. 平滑后转向更自然

### 演示3: MPC预测性避障
1. 让障碍物快速移动
2. 观察黄色轨迹提前偏离蓝色轨迹
3. 障碍物通过后,黄色回归蓝色

### 演示4: Pure Pursuit vs MPC
```
Pure Pursuit (青/橙):
- 只看前方固定距离
- 无预测,反应式
- 简单快速

MPC (黄色):
- 预测未来1秒
- 主动规划
- 复杂但智能
```

---

## 🔧 自定义可视化

### 添加新元素
```python
# 在 debug.py 添加方法
def draw_velocity_vector(self, package, x, y, vx, vy, scale=10):
    """绘制速度矢量箭头"""
    end_x = x + vx * scale
    end_y = y + vy * scale
    self.draw_line(package, x, y, end_x, end_y, color=Debug_Msg.GREEN)
    # 添加箭头头部
    # ...

# 在 main-dynamic.py 使用
debugger.draw_velocity_vector(package, 
                              robot.x, robot.y,
                              vx, 0,  # vy=0 for differential drive
                              scale=5)
```

### 改变颜色方案
```python
# 修改 main-dynamic.py
# 例如:使用品红色表示RRT*路径
debugger.draw_path(package, original_path, color=Debug_Msg.PURPLE)
```

### 条件可视化
```python
# 只在特定条件下显示某些元素
SHOW_RRT_TREE = True  # 开关
SHOW_PREDICTION = True

if SHOW_RRT_TREE:
    debugger.draw_rrt_tree(package, planning)

if SHOW_PREDICTION and USE_MPC:
    debugger.draw_prediction(package, predicted_states)
```

---

## 📝 总结

| 阶段 | 可视化元素 | 关键颜色 | 作用 |
|------|----------|---------|------|
| **规划** | RRT*树 | 灰色 | 展示探索 |
| **规划** | 最优路径 | 红色 | 展示初解 |
| **平滑** | B样条轨迹 | 蓝色 | 展示目标 |
| **控制** | MPC预测 | 黄色 | 展示优化 |
| **控制** | PP跟踪点 | 青/橙 | 展示跟踪 |
| **状态** | 机器人 | 绿色 | 展示位置 |
| **环境** | 障碍物 | 紫色 | 展示约束 |

**核心思想:**
- **灰→红→蓝:** 从探索到优化到平滑
- **绿→黄→紫:** 当前状态、未来预测、环境约束
- **层次分明:** 规划层(静态) + 控制层(动态) + 环境层(实时)

通过这套可视化系统,你可以**直观地看到算法的每个步骤**,极大地帮助理解、调试和优化!
