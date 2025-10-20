# 轨迹控制器升级说明

## 问题分析

### 1. B样条采样点数的澄清

**之前的误解:**
- Demo说"采样点越少越平滑"

**正确理解:**
```
B样条曲线本身是连续光滑的数学曲线
splev(u_new, tck) 是从这条光滑曲线上"采样"离散点
```

- **采样多(100+点)** → 折线段多 → 视觉上**更接近**光滑曲线
- **采样少(10-20点)** → 折线段少 → 看起来像多边形

**结论:** 采样点数越多,视觉上确实越平滑!

但是:对于机器人控制,**不是采样点数的问题,而是控制方式的问题**。

---

### 2. "一顿一顿"的根本原因

**旧实现 - 点对点控制器:**
```python
# 等到达一个点才切换到下一个点
if dist_to_target < 150:
    ckptidx += 1
    
vx, vw = compute_control_command(current_pos, orientation, target_pos)
```

**问题:**
1. 机器人走到点A → 停顿/减速
2. 切换到点B → 重新加速
3. 走到点B → 又停顿
4. ...循环

就像**公交车站站停**,每站都要刹车再起步。

---

## 解决方案:轨迹跟踪控制器

### 核心思想

**不跟踪离散点,而是跟踪连续轨迹**

```python
# 旧方式
target = path[idx]  # 跟踪第i个点

# 新方式
target = trajectory(u)  # 跟踪B样条参数u处的点,u连续变化
```

### 技术特点

#### 1. Pure Pursuit 算法变体
```python
# 找到机器人在轨迹上的投影点
closest_u = find_closest_point_on_trajectory(robot_pos)

# 从投影点向前看lookahead_distance,找到前视点
lookahead_u = find_lookahead_point(closest_u, lookahead_distance=300mm)

# 朝前视点行驶
target_angle = atan2(lookahead_point - robot_pos)
```

#### 2. 曲率自适应速度
```python
# 计算轨迹曲率
κ = compute_curvature(u)  # κ = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)

# 转弯半径小 → 速度慢
if κ > 0:
    v_max = min(v_max, sqrt(a_lateral_max / κ))
```

#### 3. 前馈+反馈控制
```python
# 前馈:根据轨迹曲率
ω_ff = κ * v  # 沿曲线转动需要的角速度

# 反馈:纠正角度误差
ω_fb = Kp * (target_angle - robot_angle)

# 总角速度
ω = ω_ff + ω_fb
```

---

## 代码对比

### 旧方式:点对点

```python
ckptidx = 0
while ckptidx < len(path):
    target_pos = path[ckptidx]
    dist = norm(target_pos - robot_pos)
    
    if dist < 150:  # 到达了才切换
        ckptidx += 1
        continue
    
    vx, vw = compute_control_command(robot_pos, orientation, target_pos)
    action.sendCommand(vx, vy=0, vw)
```

**结果:** 一顿一顿,每个点都有减速

---

### 新方式:轨迹跟踪

```python
# 设置轨迹(一次性)
trajectory_controller.set_trajectory(path, k=3, s=0)

while True:
    # 连续计算控制量
    vx, vw, finished = trajectory_controller.compute_control(
        robot_pos, 
        robot_orientation
    )
    
    if finished:
        break
    
    action.sendCommand(vx, vy=0, vw)
```

**结果:** 连续平滑,沿轨迹稳定跟踪

---

## 技术细节

### 1. 参数化表示
```python
# B样条参数 u ∈ [0, 1]
u = 0.0      # 起点
u = 0.5      # 中点
u = 1.0      # 终点

# 任意u处的位置
x, y = splev(u, tck)
```

### 2. 最近点查找
```python
def find_closest_point_on_trajectory(robot_pos):
    # 在u附近密集采样
    u_samples = linspace(current_u - 0.2, current_u + 0.2, 50)
    points = splev(u_samples, tck)
    
    # 找最近的
    distances = norm(points - robot_pos)
    closest_u = u_samples[argmin(distances)]
    
    return closest_u
```

### 3. 前视点查找
```python
def find_lookahead_point(start_u, lookahead_dist):
    u = start_u
    accumulated = 0
    
    # 沿轨迹前进
    while u < 1.0 and accumulated < lookahead_dist:
        p1 = splev(u, tck)
        p2 = splev(u + du, tck)
        accumulated += norm(p2 - p1)
        u += du
    
    return u, splev(u, tck)
```

### 4. 曲率计算
```python
def compute_curvature(u):
    # 一阶导数
    dx, dy = splev(u, tck, der=1)
    
    # 二阶导数
    ddx, ddy = splev(u, tck, der=2)
    
    # 曲率公式
    κ = |dx*ddy - dy*ddx| / (dx² + dy²)^(3/2)
    
    return κ
```

---

## 优势对比

| 特性 | 点对点控制 | 轨迹跟踪控制 |
|------|-----------|-------------|
| **平滑性** | ❌ 一顿一顿 | ✅ 连续平滑 |
| **速度连续性** | ❌ 频繁加减速 | ✅ 速度连续变化 |
| **转弯处理** | ❌ 固定增益 | ✅ 根据曲率自适应 |
| **跟踪精度** | ⚠️ 只到达点 | ✅ 沿整条轨迹 |
| **计算效率** | ⚠️ 简单但效果差 | ✅ 稍复杂但效果好 |
| **适合场景** | 直线为主 | 曲线轨迹 |

---

## 参数调优指南

### 1. 前视距离 `lookahead_distance`
```python
lookahead_distance = 300  # mm
```

- **太小(50-100mm):** 轨迹跟踪精确,但可能震荡
- **太大(500-1000mm):** 平滑但可能切角
- **推荐:** 机器人长度的2-3倍

### 2. 横向误差增益 `kp_lateral`
```python
kp_lateral = 2.0
```

- **太小:** 跟踪不紧,可能偏离轨迹
- **太大:** 可能震荡
- **推荐:** 从2.0开始调

### 3. 角度控制增益 `kp_angular`
```python
kp_angular = 4.0
```

- **太小:** 转向慢,跟不上曲线
- **太大:** 转向过激,震荡
- **推荐:** 4.0-6.0

### 4. B样条采样点数
```python
# 对于轨迹控制器,采样点数不再关键!
# 因为控制器直接使用B样条参数u,不依赖采样点

# 但如果需要可视化,可以多采样
num_points_for_visualization = 200  # 视觉平滑
```

---

## 运行效果预期

### 旧方式输出
```
Reached waypoint 1/50
Reached waypoint 2/50
...
Waypoint 10/50, dist=342.5, vx=684.9, vw=-0.03
Waypoint 10/50, dist=332.8, vx=665.6, vw=-0.03
...
Reached waypoint 11/50  ← 停顿!
Reached waypoint 12/50  ← 停顿!
```

### 新方式输出
```
Trajectory set: 50 control points, 8456.3mm total length
Progress: 5.0%, pos=(123.4,567.8), vx=856.2, vw=0.23
Progress: 10.0%, pos=(234.5,678.9), vx=892.1, vw=0.31
Progress: 15.0%, pos=(345.6,789.0), vx=901.5, vw=0.28
...
Progress: 95.0%, pos=(4321.0,2987.6), vx=756.3, vw=-0.15
Reached goal 1
```

**连续进度更新,无停顿!**

---

## 使用步骤

### 1. 安装依赖
```bash
# 已有 scipy, numpy
```

### 2. 运行测试
```bash
python trajectory_controller.py
```

会生成 `trajectory_controller_demo.png` 显示:
- 轨迹可视化
- 曲率分布图

### 3. 运行主程序
```bash
python main-dynamic.py
```

观察:
- 是否还有"一顿一顿"
- 速度是否连续变化
- 转弯是否平滑

---

## 进一步优化方向

### 1. 速度规划器集成
```python
# 当前:恒速 + 曲率减速
# 未来:梯形速度曲线

velocity_profiler = VelocityProfiler(...)
v_planned = velocity_profiler.compute_velocity_at_u(u)
```

### 2. 动态重规划
```python
# 检测障碍物变化
if obstacle_moved:
    # 重新规划后半段
    new_path = replan_from(current_u)
    trajectory_controller.update_trajectory(new_path, start_u=current_u)
```

### 3. MPC集成
```python
# Model Predictive Control
# 预测未来N步,优化控制序列
mpc_controller = MPCController(trajectory_controller)
vx, vw = mpc_controller.compute_control(...)
```

---

## FAQ

### Q1: 为什么不直接用SMC(滑模控制)?

A: 
- SMC适合**轨迹跟踪**,但需要轨迹的**连续导数**
- 当前实现已经是一种轨迹跟踪,类似Pure Pursuit
- SMC可以作为下一步改进,替换角速度控制部分

### Q2: 采样点数到底该设多少?

A:
- **对于轨迹控制器:** 不重要!控制器直接用B样条参数
- **对于可视化调试:** 100-200点足够
- **对于碰撞检测:** 50-100点(在`bspline_smoother`中)

### Q3: 为什么还会有碰撞警告?

A:
- B样条平滑会让路径靠近障碍物
- 可以:
  1. 增大 `robot_radius` 参数(增加安全裕度)
  2. 减小 `s` 平滑参数(更忠实原路径)
  3. 增加碰撞检测容差

### Q4: 轨迹控制器适合动态场景吗?

A:
- **半适合:** 如果障碍物慢速移动,可以重规划
- **不适合:** 如果障碍物快速移动,需要DWA/MPC
- **建议:** 结合使用:
  - 静态部分用B样条轨迹跟踪
  - 动态部分用DWA局部避障

---

## 总结

| 改进点 | 旧方案 | 新方案 |
|--------|--------|--------|
| **控制方式** | 点对点 | 连续轨迹跟踪 |
| **运动特征** | 一顿一顿 | 平滑连续 |
| **速度规划** | 简单衰减 | 曲率自适应 |
| **角速度控制** | 纯反馈 | 前馈+反馈 |
| **B样条使用** | 仅采样点 | 参数化表示 |

**核心提升:** 从"走到点"变成"跟踪轨迹"!
