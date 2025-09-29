from vision import Vision
from action import Action
from debug import Debugger
from prm import PRM
import time
import math

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    planner = PRM()
    # 1. 路径规划
    start_x, start_y = -4500, -3000
    goal_x, goal_y = 4500, 3000
    path_x, path_y, road_map, sample_x, sample_y = planner.plan(
        vision=vision, start_x=start_x, start_y=start_y, goal_x=goal_x, goal_y=goal_y)

    # 3. 沿路径点依次移动
    idx = 0
    target_eps = 100  # 允许误差，单位mm
    angle_eps = 0.1   # 朝向允许误差，单位弧度
    path_x = path_x[::-1]
    path_y = path_y[::-1]
    while idx < len(path_x):
        # 获取当前目标点
        tx, ty = path_x[idx], path_y[idx]
        # 获取黄色0号当前位置和朝向
        robot = vision.yellow_robot[0]
        rx, ry = robot.x, robot.y
        theta = robot.orientation  # 单位：弧度
        # 计算与目标点距离
        dist = ((tx - rx) ** 2 + (ty - ry) ** 2) ** 0.5
        # 计算目标方向
        target_angle = math.atan2(ty - ry, tx - rx)
        # 计算角度误差
        angle_diff = target_angle - theta
        # 归一化到[-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        # 差速控制逻辑
        if dist < target_eps:
            idx += 1
            continue
        # 先转向，后前进
        K_v = 0.5  # 线速度比例系数
        K_w = 2.0  # 角速度比例系数
        max_v = 1000
        max_w = 3.0
        if abs(angle_diff) > angle_eps:
            vx = 0
            vw = K_w * angle_diff
        else:
            vx = K_v * dist
            vw = K_w * angle_diff
        # 限制最大速度
        vx = max(-max_v, min(max_v, vx))
        vw = max(-max_w, min(max_w, vw))
        # 发送指令给黄色0号
        action.sendCommand(vx=vx, vw=vw)
        # 画路径和采样点
        debugger.draw_all(sample_x, sample_y, road_map, path_x, path_y)
        time.sleep(0.02)
    # 到达终点后停止
    for _ in range(10):
        action.sendCommand(vx=0, vw=0)
        time.sleep(0.02)
    print("黄色0号已到达目标点！")
