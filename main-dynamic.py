import numpy as np
from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
import time
from time import sleep
from rrt_star import RRTStar
from rrt_with_pathsmoothing import path_smoothing
from bspline_smoother import BSplineSmoother, VelocityProfiler
from trajectory_controller import TrajectoryController
# import matplotlib.pyplot as plt

# 目标点序列
goals=[[-4500,-3000],[4500,3000]]
curgoal=0
rndcnt=0

# 机器人参数
MAX_LINEAR_VEL = 1000  # mm/s 最大线速度
MAX_ANGULAR_VEL = 3.0   # rad/s 最大角速度
ROBOT_RADIUS = 100      # mm 机器人半径
POSITION_TOLERANCE = 150  # mm 到达目标点的容差
ANGLE_TOLERANCE = 0.1   # rad 角度容差
def normalize_angle(angle):
	"""将角度归一化到[-pi, pi]"""
	while angle > np.pi:
		angle -= 2 * np.pi
	while angle < -np.pi:
		angle += 2 * np.pi
	return angle


def compute_control_command(current_pos, current_orientation, target_pos, target_vel=MAX_LINEAR_VEL):
	"""
	计算控制指令（差速驱动模型）
	
	Args:
		current_pos: 当前位置 [x, y]
		current_orientation: 当前朝向 (rad)
		target_pos: 目标位置 [x, y]
		target_vel: 目标线速度
		
	Returns:
		(vx, vw): 前向速度和角速度
	"""
	# 计算目标点相对于机器人的角度
	dx = target_pos[0] - current_pos[0]
	dy = target_pos[1] - current_pos[1]
	target_angle = np.arctan2(dy, dx)
	
	# 计算角度误差
	angle_error = normalize_angle(target_angle - current_orientation)
	
	# 计算距离
	distance = np.sqrt(dx**2 + dy**2)
	
	# 角速度控制 - 比例控制器
	Kp_angular = 4.0  # 角速度增益
	vw = Kp_angular * angle_error
	vw = np.clip(vw, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)
	
	# 线速度控制 - 根据角度误差调整
	# 如果角度误差大，降低线速度；角度小时全速前进
	angle_factor = np.cos(angle_error)  # 角度对齐时为1，垂直时为0
	angle_factor = max(0, angle_factor)  # 确保非负
	
	# 距离越近，速度越慢
	if distance < 500:
		distance_factor = distance / 500
	else:
		distance_factor = 1.0
	
	vx = target_vel * angle_factor * distance_factor
	vx = np.clip(vx, 0, MAX_LINEAR_VEL)
	
	# 如果角度误差太大（>60度），先原地转向
	if abs(angle_error) > np.pi / 3:
		vx = 0
	
	return vx, vw


if __name__ == '__main__':
	vision = Vision()
	action = Action()
	debugger = Debugger()
	
	# 初始化B样条平滑器和轨迹控制器
	bspline_smoother = BSplineSmoother(robot_radius=ROBOT_RADIUS)
	trajectory_controller = TrajectoryController(
		max_linear_vel=MAX_LINEAR_VEL,
		max_angular_vel=MAX_ANGULAR_VEL,
		lookahead_distance=300,  # 300mm前视距离
		kp_angular=4.0
	)
	
	sleep(1)  # wait for vision start
	
	while rndcnt<=1:
		# 1. path planning & velocity planning
		# Do something
		# 枚举除蓝色robot0外的所有机器人位置
		print(vision.my_robot.orientation)
		while curgoal<=1:
			robot_obstacles = []
			for robot in vision.blue_robot[1:]:
				robot_obstacles.append((robot.x, robot.y, 200))
			for robot in vision.yellow_robot:
				robot_obstacles.append((robot.x, robot.y, 200))
			planning = RRTStar(
				start=goals[curgoal],
				goal=[vision.my_robot.x, vision.my_robot.y],
				rand_area=[[-5000, +5000],[-3500, 3500]],
				obstacle_list=robot_obstacles,
				expand_dis=800,
				robot_radius=100,
				max_iter=2000
			)
			original_path = planning.planning(animation=False)
			if original_path is None:
				print("No Path Found")
				break
			else:
				print("found path!!")
				print(f"Original path length: {len(original_path)} points")
				
				# B样条平滑路径
				# 先用简单平滑，然后用B样条
				smoothed_path = path_smoothing(original_path, 100, robot_obstacles, robot_radius=ROBOT_RADIUS)
				print(f"After simple smoothing: {len(smoothed_path)} points")
				
				# 使用B样条进一步平滑
				# 关键：采样点数不要太多，否则跟原路径差不多
				bspline_path = bspline_smoother.smooth_path(
					smoothed_path, 
					robot_obstacles, 
					s=0,  # 插值模式，通过所有点
					k=3,  # 三次样条
					num_points=max(50, len(smoothed_path) * 2)  # 适度增加点数
				)
				print(f"After B-spline smoothing: {len(bspline_path)} points")
				
				path = bspline_path
				
				# 设置轨迹控制器
				trajectory_controller.set_trajectory(path, k=3, s=0)
				
				# # 绘制路径对比图
				# plt.figure(figsize=(12, 8))
				
				# # 绘制障碍物
				# for (ox, oy, radius) in robot_obstacles:
				# 	circle = plt.Circle((ox, oy), radius, color='red', alpha=0.5)
				# 	plt.gca().add_patch(circle)
				
				# # 绘制原始路径 (蓝色虚线)
				# if len(original_path) > 0:
				# 	original_x = [point[0] for point in original_path]
				# 	original_y = [point[1] for point in original_path]
				# 	plt.plot(original_x, original_y, 'b--', linewidth=2, label='原始RRT路径', alpha=0.7)
				# 	plt.scatter(original_x, original_y, c='blue', s=30, alpha=0.7)
				
				# # 绘制平滑后路径 (红色实线)
				# if len(smoothed_path) > 0:
				# 	smoothed_x = [point[0] for point in smoothed_path]
				# 	smoothed_y = [point[1] for point in smoothed_path]
				# 	plt.plot(smoothed_x, smoothed_y, 'r-', linewidth=3, label='平滑后路径')
				# 	plt.scatter(smoothed_x, smoothed_y, c='red', s=50)
				
				# # 标记起点和终点
				# plt.scatter(goals[curgoal][0], goals[curgoal][1], c='green', s=200, marker='s', label='起点', zorder=5)
				# plt.scatter(vision.my_robot.x, vision.my_robot.y, c='orange', s=200, marker='*', label='机器人位置', zorder=5)
				
				# # 设置图形属性
				# plt.xlabel('X坐标 (mm)')
				# plt.ylabel('Y坐标 (mm)')
				# plt.title(f'路径规划对比 - 目标点{curgoal+1}')
				# plt.legend()
				# plt.grid(True, alpha=0.3)
				# plt.axis('equal')
				
				# # 设置坐标轴范围
				# plt.xlim(-5500, 5500)
				# plt.ylim(-4000, 4000)
				
				# # 显示图形
				# plt.tight_layout()
				# plt.show(block=False)
				# plt.pause(5)  # 显示2秒
				# plt.close()
				
				# # 使用平滑后的路径进行导航
				# path = smoothed_path
			
			# 使用轨迹控制器连续跟踪
			while True:
				# 获取当前机器人状态
				current_pos = np.array([vision.my_robot.x, vision.my_robot.y])
				current_orientation = vision.my_robot.orientation
				
				# 计算控制指令
				vx, vw, finished = trajectory_controller.compute_control(
					current_pos, 
					current_orientation
				)
				
				if finished:
					print(f"Reached goal {curgoal+1}")
					break
				
				# 发送控制指令
				action.sendCommand(vx=vx, vy=0, vw=vw)
				
				# 调试可视化
				package = Debug_Msgs()
				# 绘制机器人位置
				debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y, 100)
				
				# 绘制当前跟踪点和前视点
				debug_info = trajectory_controller.get_debug_info()
				if 'current_point' in debug_info:
					cp = debug_info['current_point']
					tp = debug_info['target_point']
					debugger.draw_point(package, int(cp[0]), int(cp[1]))  # 最近点
					debugger.draw_point(package, int(tp[0]), int(tp[1]))  # 前视点
				
				debugger.send(package)
				
				# 每0.5秒打印一次调试信息
				progress = trajectory_controller.get_progress()
				if int(progress * 100) % 5 == 0:
					print(f"Progress: {progress*100:.1f}%, "
					      f"pos=({current_pos[0]:.1f},{current_pos[1]:.1f}), "
					      f"orient={np.degrees(current_orientation):.1f}°, "
					      f"vx={vx:.1f}, vw={vw:.2f}")
				
				time.sleep(0.01)
			
			# 到达目标点,停止
			action.sendCommand(vx=0, vy=0, vw=0)
			curgoal += 1
			# 2. send command
			# action.sendCommand(vx=100, vy=0, vw=0)

			# 3. draw debug msg
			
		# time.sleep(0.01)
		rndcnt+=1
		curgoal=0
