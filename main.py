import numpy as np
from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
import time
from time import sleep
from rrt_star import RRTStar
from rrt_with_pathsmoothing import path_smoothing
# import matplotlib.pyplot as plt
goals=[[-4500,-3000],[4500,3000]]
curgoal=0
rndcnt=0
robotvel=1000
if __name__ == '__main__':
	vision = Vision()
	action = Action()
	debugger = Debugger()
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
				print("Original path:", original_path)
				
				# 路径平滑优化
				smoothed_path = path_smoothing(original_path, 100, robot_obstacles, robot_radius=100)
				print("Smoothed path:", smoothed_path)
				
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
				path = smoothed_path
			ckptidx=0
			while ckptidx<len(path):
				theta=np.arctan2(path[ckptidx][1]-vision.my_robot.y,path[ckptidx][0]-vision.my_robot.x)
				print(ckptidx, theta, path[ckptidx], vision.my_robot.x, vision.my_robot.y, np.sqrt(np.square(path[ckptidx][0]-vision.my_robot.x)+np.square(path[ckptidx][1]-vision.my_robot.y)))
				print(np.cos(-(vision.my_robot.orientation-theta))*robotvel,np.sin(-(vision.my_robot.orientation-theta))*robotvel)
				action.sendCommand(vx=np.cos(-(vision.my_robot.orientation-theta))*robotvel, vy=np.sin(-(vision.my_robot.orientation-theta))*robotvel, vw=0)
				time.sleep(0.01)
				package = Debug_Msgs()
				debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y)
				debugger.send(package)
				if(np.sqrt(np.square(path[ckptidx][0]-vision.my_robot.x)+np.square(path[ckptidx][1]-vision.my_robot.y))<100):
					ckptidx+=1
			curgoal+=1
			action.sendCommand(vx=0, vy=0, vw=0)
			# 2. send command
			# action.sendCommand(vx=100, vy=0, vw=0)

			# 3. draw debug msg
			
		# time.sleep(0.01)
		rndcnt+=1
		curgoal=0
