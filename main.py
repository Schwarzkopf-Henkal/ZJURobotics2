from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
import time
from time import sleep
from rrt_star import RRTStar
goals=[[-4500,-3000],[4500,3000]]
curgoal=0
rndcnt=0
if __name__ == '__main__':
	vision = Vision()
	action = Action()
	debugger = Debugger()
	sleep(1)  # wait for vision start
	while rndcnt<=1:
		# 1. path planning & velocity planning
		# Do something
		# 枚举除蓝色robot0外的所有机器人位置
		while curgoal<=1:
			robot_obstacles = []
			for robot in vision.blue_robot[1:]:
				robot_obstacles.append((robot.x, robot.y, 200))
			for robot in vision.yellow_robot:
				robot_obstacles.append((robot.x, robot.y, 200))
			planning = RRTStar(
				start=goals[curgoal],
				goal=[vision.my_robot.x, vision.my_robot.y],
				rand_area=[-5000, +5000],
				obstacle_list=robot_obstacles,
				expand_dis=300,
				robot_radius=100,
				max_iter=2000
			)
			path = planning.planning(animation=True)
			if path is None:
				print("No Path Found")
				break
			else:
				print("found path!!")
				print(path)
			ckptidx=0
			while ckptidx<len(path)-1:
			# 2. send command
			action.sendCommand(vx=100, vy=0, vw=0)

			# 3. draw debug msg
			package = Debug_Msgs()
			debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y)
			debugger.send(package)

		time.sleep(0.01)
		rndcnt+=1
		curgoal=0
