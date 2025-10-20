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
from mpc_controller import MPCController, DynamicObstaclePredictor
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
	
	# 初始化控制器
	bspline_smoother = BSplineSmoother(robot_radius=ROBOT_RADIUS)
	
	# 选择控制器类型
	USE_MPC = True  # True: MPC控制器, False: Pure Pursuit控制器
	
	if USE_MPC:
		# MPC控制器 - 支持动态避障
		controller = MPCController(
			prediction_horizon=10,     # 预测10步
			control_horizon=5,         # 优化5步控制量
			dt=0.1,                    # 100ms步长
			max_linear_vel=MAX_LINEAR_VEL,
			max_angular_vel=MAX_ANGULAR_VEL,
			robot_radius=ROBOT_RADIUS + 50,  # 额外安全裕度
			safety_distance=100
		)
		# 障碍物预测器
		obstacle_predictor = DynamicObstaclePredictor(history_length=5)
		print("Using MPC Controller with dynamic obstacle avoidance")
	else:
		# Pure Pursuit轨迹跟踪控制器
		controller = TrajectoryController(
			max_linear_vel=MAX_LINEAR_VEL,
			max_angular_vel=MAX_ANGULAR_VEL,
			lookahead_distance=300,
			kp_angular=4.0
		)
		print("Using Pure Pursuit Controller")
	
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
				
				# 设置控制器轨迹
				controller.set_trajectory(path, k=3, s=0)
				
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
			
			# 主控制循环
			current_u = 0.0  # 轨迹参数
			
			while True:
				# 获取当前机器人状态
				current_pos = np.array([vision.my_robot.x, vision.my_robot.y])
				current_orientation = vision.my_robot.orientation
				
				# 如果使用MPC,更新动态障碍物
				if USE_MPC:
					# 构建障碍物字典 {id: (x, y, radius)}
					obstacles_dict = {}
					
					# 添加其他蓝色机器人
					for i, robot in enumerate(vision.blue_robot[1:], start=1):
						obs_id = f'blue_{i}'
						obstacles_dict[obs_id] = (robot.x, robot.y, 150)
					
					# 添加黄色机器人
					for i, robot in enumerate(vision.yellow_robot):
						obs_id = f'yellow_{i}'
						obstacles_dict[obs_id] = (robot.x, robot.y, 150)
					
					# 更新障碍物历史并估计速度
					obstacle_predictor.update(obstacles_dict)
					
					# 获取带速度预测的障碍物列表
					dynamic_obstacles = obstacle_predictor.get_predicted_obstacles(obstacles_dict)
					
					# 更新MPC控制器
					controller.update_dynamic_obstacles(dynamic_obstacles)
					
					# 计算MPC控制指令
					vx, vw, finished, debug_info = controller.compute_control(
						current_pos,
						current_orientation,
						current_u
					)
					
					# 更新轨迹参数
					if 'closest_u' in debug_info:
						current_u = debug_info['closest_u']
					
				else:
					# 使用Pure Pursuit控制器
					vx, vw, finished = controller.compute_control(
						current_pos,
						current_orientation
					)
					debug_info = controller.get_debug_info()
				
				if finished:
					print(f"Reached goal {curgoal+1}")
					break
				
				# 发送控制指令
				action.sendCommand(vx=vx, vy=0, vw=vw)
				
				# 调试可视化
				package = Debug_Msgs()
				# 绘制机器人位置
				debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y, 100)
				
				# 绘制跟踪点
				if not USE_MPC and 'current_point' in debug_info:
					cp = debug_info['current_point']
					tp = debug_info['target_point']
					debugger.draw_point(package, int(cp[0]), int(cp[1]))
					debugger.draw_point(package, int(tp[0]), int(tp[1]))
				
				debugger.send(package)
				
				# 打印调试信息
				if USE_MPC:
					if 'progress' in debug_info and int(debug_info['progress']) % 10 == 0:
						print(f"MPC - Progress: {debug_info['progress']:.1f}%, "
						      f"pos=({current_pos[0]:.0f},{current_pos[1]:.0f}), "
						      f"vx={vx:.0f}, vw={vw:.2f}, "
						      f"cost={debug_info.get('cost', 0):.1f}, "
						      f"obs={debug_info.get('n_obstacles', 0)}, "
						      f"opt_time={debug_info.get('optimization_time', 0):.1f}ms")
				else:
					progress = controller.get_progress()
					if int(progress * 100) % 10 == 0:
						print(f"PurePursuit - Progress: {progress*100:.1f}%, "
						      f"pos=({current_pos[0]:.0f},{current_pos[1]:.0f}), "
						      f"vx={vx:.0f}, vw={vw:.2f}")
				
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
