# 🚀 快速开始指南

## 5分钟快速上手

### 步骤1: 安装依赖

```bash
pip install scipy numpy
```

可选（用于测试）：
```bash
pip install matplotlib
```

---

### 步骤2: 测试B样条功能（可选）

在不连接机器人的情况下测试新功能：

```bash
python test_bspline.py
```

如果成功，你会看到：
- 控制台输出路径统计信息
- 生成两张图片：
  - `bspline_test_result.png` - 路径平滑效果
  - `velocity_profile_test.png` - 速度曲线

---

### 步骤3: 运行主程序

```bash
python main-dynamic.py
```

---

### 步骤4: 观察输出

**正常输出示例**：
```
found path!!
Original path length: 15 points
After simple smoothing: 12 points
After B-spline smoothing: 100 points

Reached waypoint 10/100
Waypoint 10/100, dist=234.5, orient=45.2°, vx=800.0, vw=0.52
Reached waypoint 20/100
...
Reached goal 1
```

---

## ⚙️ 参数快速调整

### 机器人太快/太慢

编辑 `main-dynamic.py` 第13行：
```python
MAX_LINEAR_VEL = 1000  # 改为 500（慢） 或 1500（快）
```

### 转向太激进/太慢

编辑 `main-dynamic.py` 第58行：
```python
Kp_angular = 4.0  # 改为 2.0（慢） 或 6.0（快）
```

### 到达精度

编辑 `main-dynamic.py` 第16行：
```python
POSITION_TOLERANCE = 150  # 改为 100（严格） 或 200（宽松）
```

---

## 🐛 常见问题

### Q: 提示 "No module named 'scipy'"

**解决**：
```bash
pip install scipy
```

### Q: 机器人在原地打转

**原因**：角速度增益太大

**解决**：降低 `Kp_angular` 到 2.0

### Q: 路径穿过障碍物

**原因**：平滑过度

**解决**：在 `main-dynamic.py` 第121行改为：
```python
s=0,  # 保持为0，使用插值模式
```

### Q: "Warning: Path too short"

**说明**：这是正常的，RRT*生成的路径点太少（<4个），程序会自动使用原始路径

**无需处理**

---

## 📊 性能对比

运行几次后，对比改进前后的表现：

| 指标 | 改进前 | 改进后 | 如何测量 |
|------|--------|--------|----------|
| 到达成功率 | ? | ? | 10次中成功次数 |
| 平均用时 | ? | ? | 从起点到终点的时间 |
| 路径平滑度 | 低 | 高 | 目视观察 |
| 控制精度 | 低 | 高 | 是否正确转向 |

---

## 🎯 下一步

### 如果效果好
继续使用，可选：
- 调优参数以获得更好性能
- 查看 `DYNAMIC_OPTIMIZATION_GUIDE.md` 了解动态场景优化

### 如果效果不理想
1. 检查参数设置（见上面的快速调整）
2. 运行测试脚本确认B样条功能正常
3. 查看 `README_IMPROVEMENTS.md` 的调试章节

---

## 📞 需要帮助？

查看详细文档：
- `SUMMARY.md` - 完整改进总结
- `README_IMPROVEMENTS.md` - 详细使用说明
- `DYNAMIC_OPTIMIZATION_GUIDE.md` - 动态场景优化方案

---

**祝测试顺利！** 🎉
