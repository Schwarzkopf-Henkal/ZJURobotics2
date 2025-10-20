# 动态场景优化方案

## 当前系统的问题

当前的 `main-dynamic.py` 使用 **静态规划** 策略：
1. **一次性规划**：在起点规划完整路径到终点
2. **不响应变化**：障碍物（其他机器人）移动后，路径不会更新
3. **容易失败**：动态障碍物可能阻塞原本安全的路径

## 动态场景优化方案

### 方案1: 滚动窗口重规划 (Receding Horizon) ⭐推荐

**核心思想**：不规划到最终目标，只规划短期局部路径，并周期性重规划

**实现步骤**：

```python
# 伪代码
PLANNING_HORIZON = 2000  # mm，只规划2米范围内
REPLAN_INTERVAL = 0.5    # 秒，每0.5秒重规划一次

while not reached_final_goal:
    # 1. 计算局部目标点（在规划视野内朝向最终目标）
    local_goal = compute_local_goal(current_pos, final_goal, PLANNING_HORIZON)
    
    # 2. 获取最新的障碍物信息
    obstacles = get_current_obstacles()
    
    # 3. 规划局部路径
    local_path = plan_path(current_pos, local_goal, obstacles)
    
    # 4. 执行路径（只执行一小段）
    execute_path_segment(local_path, duration=REPLAN_INTERVAL)
    
    # 5. 循环：重新规划
```

**优点**：
- ✅ 简单易实现
- ✅ 自动适应动态环境
- ✅ 计算量可控（只规划局部）

**缺点**：
- ⚠️ 可能陷入局部最优（死胡同）
- ⚠️ 需要调参（视野大小、重规划频率）

**实现难度**：★★☆☆☆

---

### 方案2: 动态窗口法 (DWA - Dynamic Window Approach) ⭐适合实时性要求高

**核心思想**：在速度空间中搜索，考虑动力学约束，实时避障

**原理**：
1. 在当前速度附近采样多个候选速度 `(vx, vw)`
2. 对每个候选速度，模拟未来短时间（1-2秒）的轨迹
3. 评分：`score = α·heading + β·clearance + γ·velocity`
   - heading: 朝向目标的程度
   - clearance: 与障碍物的距离
   - velocity: 速度大小（越快越好）
4. 选择得分最高的速度执行

**优点**：
- ✅ 天然考虑动力学约束
- ✅ 实时性好（10-50ms）
- ✅ 平滑控制
- ✅ 适合差速驱动机器人

**缺点**：
- ⚠️ 只考虑局部，可能陷入死角
- ⚠️ 需要结合全局规划器

**实现难度**：★★★☆☆

**Python库支持**：可以使用 `python-motion-planning` 库

---

### 方案3: 障碍物运动预测 + 时空A*

**核心思想**：预测对手机器人的未来轨迹，在时空中规划避开

**实现步骤**：

```python
# 1. 预测障碍物运动
def predict_obstacle_trajectory(obstacle, time_horizon=3.0):
    """
    简单预测：假设障碍物匀速直线运动
    复杂预测：卡尔曼滤波、神经网络预测
    """
    predicted_positions = []
    dt = 0.1
    for t in range(0, int(time_horizon / dt)):
        future_pos = obstacle.pos + obstacle.vel * t * dt
        predicted_positions.append((future_pos, t * dt))
    return predicted_positions

# 2. 时空A*规划
# 状态：(x, y, t)
# 障碍物在不同时刻占据不同位置
# 规划时需要确保在时刻t，机器人不与该时刻的障碍物重叠
```

**优点**：
- ✅ 主动预测，更安全
- ✅ 可以找到等待时机（让对手先过）

**缺点**：
- ⚠️ 预测不准确时可能失败
- ⚠️ 计算复杂度高

**实现难度**：★★★★☆

---

### 方案4: 模型预测控制 (MPC)

**核心思想**：滚动优化控制序列，考虑未来N步的状态

**原理**：
```
min  Σ (state_cost + control_cost)
s.t. 
    dynamics: x(t+1) = f(x(t), u(t))
    constraints: obstacle avoidance, vel/acc limits
```

**优点**：
- ✅ 最优控制
- ✅ 显式处理约束
- ✅ 平滑轨迹

**缺点**：
- ⚠️ 计算量大
- ⚠️ 实现复杂

**实现难度**：★★★★★

---

### 方案5: 分层规划 - 全局+局部

**核心思想**：结合全局路径规划和局部反应式避障

```
┌─────────────────────────────────────┐
│  全局规划器 (慢，粗糙)              │
│  - RRT*/A* 规划参考路径             │
│  - 每1-2秒更新一次                  │
│  - 忽略动态障碍物                   │
└──────────┬──────────────────────────┘
           │ 提供参考路径
           ▼
┌─────────────────────────────────────┐
│  局部规划器 (快，精细)              │
│  - DWA / 速度障碍法                 │
│  - 每0.05秒更新                     │
│  - 跟随参考路径 + 避开动态障碍      │
└─────────────────────────────────────┘
```

**优点**：
- ✅ 兼顾效率和安全
- ✅ 工业界常用方案
- ✅ 鲁棒性好

**缺点**：
- ⚠️ 两层协调需要调参

**实现难度**：★★★☆☆

---

## 推荐实施路线

### 阶段1：滚动窗口重规划（本周）
最简单，立即见效

### 阶段2：添加DWA局部避障（下周）
提升实时性和平滑度

### 阶段3：障碍物速度跟踪（可选）
如果对手移动很快才需要

---

## 快速实现：滚动窗口版本

以下是最小改动方案（在当前代码基础上）：

```python
# main-dynamic.py 改动点

PLANNING_HORIZON = 2000  # 只规划2米内
REPLAN_THRESHOLD = 500   # 执行500mm后重新规划

while not reached_goal:
    # 当前位置
    current_pos = [vision.my_robot.x, vision.my_robot.y]
    
    # 计算局部目标
    direction = goal - current_pos
    distance = norm(direction)
    if distance > PLANNING_HORIZON:
        local_goal = current_pos + direction / distance * PLANNING_HORIZON
    else:
        local_goal = goal
    
    # 规划到局部目标
    path = rrt_star.plan(current_pos, local_goal, obstacles)
    path = bspline_smooth(path)
    
    # 执行路径，但不要走完，走一部分就重新规划
    executed_distance = 0
    for waypoint in path:
        execute_to(waypoint)
        executed_distance += dist(last_waypoint, waypoint)
        
        # 走了一定距离，或者检测到障碍物变化，触发重规划
        if executed_distance > REPLAN_THRESHOLD:
            break  # 跳出，触发外层while重新规划
```

---

## 障碍物变化检测

添加一个简单的障碍物跟踪器：

```python
class ObstacleTracker:
    def __init__(self):
        self.last_obstacles = []
    
    def has_significant_change(self, current_obstacles, threshold=200):
        """检测障碍物是否有显著移动"""
        if len(self.last_obstacles) != len(current_obstacles):
            return True
        
        for last_obs, curr_obs in zip(self.last_obstacles, current_obstacles):
            dist = np.linalg.norm(
                np.array(last_obs[:2]) - np.array(curr_obs[:2])
            )
            if dist > threshold:
                return True
        
        self.last_obstacles = current_obstacles
        return False

# 使用
tracker = ObstacleTracker()

while planning:
    obstacles = get_obstacles()
    if tracker.has_significant_change(obstacles):
        print("Obstacle moved! Replanning...")
        replan()
```

---

## 性能对比（预期）

| 方案 | 成功率 | 路径长度 | 计算时间 | 实现难度 |
|------|--------|----------|----------|----------|
| 当前静态规划 | 60% | 优 | 快 | 简单 |
| 滚动窗口 | 85% | 良 | 快 | 简单 |
| DWA | 90% | 中 | 很快 | 中等 |
| 时空A* | 95% | 优 | 慢 | 困难 |
| MPC | 98% | 最优 | 很慢 | 很困难 |
| 分层规划 | 95% | 优 | 快 | 中等 |

---

## 总结

**立即实施**：滚动窗口重规划
**中期目标**：分层规划（RRT* + DWA）
**长期优化**：如果需要更高性能，考虑MPC或学习方法

需要我帮你实现滚动窗口版本吗？
