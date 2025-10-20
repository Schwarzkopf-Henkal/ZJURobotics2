import math
import numpy as np
from scipy.optimize import minimize
from vision import Vision
from action import Action
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs
import time

if __name__ == '__main__':
    vision = Vision()
    action = Action()
    debugger = Debugger()
    time.sleep(0.1)

    # Start and goal positions
    point_a = [-4500, -3000]
    point_b = [4500, 3000]
    current_target = point_b  # Start by going to point_b
    
    # Trip counter
    trip_count = 0  # Count number of one-way trips
    max_trips = 4   # 2 round trips = 4 one-way trips
    
    # Robot parameters
    radius = 200  # Robot radius in mm
    safe_margin = 100  # Increased safety margin for dynamic obstacles
    r_safe = 2 * radius + safe_margin  # Safe radius for CBF
    
    # Stuck detection
    stuck_counter = 0
    max_stuck_count = 300  # Increased from 30 to 100 (1 second at 0.01s per loop)
    last_position = [0, 0]
    stuck_threshold = 300  # mm movement threshold

    # Velocity smoothing filter (low-pass filter)
    alpha = 0.7  # Smoothing factor
    prev_vx = 0.0
    prev_vw = 0.0

    while trip_count < max_trips:
        # Blue robot 0 current state
        robot = vision.blue_robot[0]
        rx, ry = robot.x, robot.y
        theta = robot.orientation
        
        # Stuck detection
        movement = math.hypot(rx - last_position[0], ry - last_position[1])
        if movement < stuck_threshold:
            stuck_counter += 1
        else:
            stuck_counter = 0
        last_position = [rx, ry]

        # Current target point
        goal_x, goal_y = current_target[0], current_target[1]

        # Check if stuck and apply escape maneuver
        if stuck_counter >= max_stuck_count:
            print(f"Robot stuck! Attempting escape maneuver...")
            import random
            escape_angle = random.uniform(-math.pi, math.pi)
            action.sendCommand(vx=300, vw=escape_angle * 0.5)
            time.sleep(0.5)
            stuck_counter = 0
            prev_vx = 0.0
            prev_vw = 0.0
            continue

        # Desired velocity towards the goal
        dx = goal_x - rx
        dy = goal_y - ry
        dist = math.hypot(dx, dy)
        
        # Check if reached current target, switch to the other point
        if dist < 150:
            trip_count += 1
            if current_target[0] == point_b[0] and current_target[1] == point_b[1]:
                current_target = point_a
                print(f"Reached point B ({goal_x}, {goal_y}) (Trip {trip_count}/{max_trips}), now heading to point A")
            else:
                current_target = point_b
                print(f"Reached point A ({goal_x}, {goal_y}) (Trip {trip_count}/{max_trips}), now heading to point B")
            
            if trip_count >= max_trips:
                print("Completed 2 round trips! Stopping...")
                action.sendCommand(vx=0, vw=0)
                break
            
            time.sleep(0.2)
            prev_vx = 0.0
            prev_vw = 0.0
            continue

        # Constant velocity control with smoother speed decay near target
        max_v = 1000  # Maximum linear velocity
        max_w = 2.0  # Maximum angular velocity
        
        base_speed = 500
        if dist < 1000:
            decay_factor = math.exp(-(1000 - dist) / 500.0)
            target_speed = 200 + (base_speed - 200) * decay_factor
        else:
            target_speed = base_speed
        
        vx_des = target_speed * dx / dist if dist > 0 else 0
        vy_des = target_speed * dy / dist if dist > 0 else 0
        
        # Convert to robot frame
        v_body = math.cos(theta) * vx_des + math.sin(theta) * vy_des
        # Enhanced turning: higher gain + normalization + min threshold
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = angle_to_goal - theta
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        w_body = 2.0 * angle_diff  # Higher gain for sensitive turning
        if abs(angle_diff) < 0.05:  # Min threshold to avoid zero-lock
            w_body = 0.0
        else:
            w_body = max(-max_w, min(max_w, w_body))
        
        print(f"angle_diff: {angle_diff:.3f} rad, w_body: {w_body:.3f} rad/s")  # Debug print
        
        # Limit velocities
        v_body = max(-max_v, min(max_v, v_body))

        # SciPy QP for CBF (relative degree 1: h = dist - r_safe)
        u_ref = np.array([v_body, w_body])
        constraints = []
        
        # Collect close obstacles (limit to 3 for stability)
        obs_list = []
        all_robots = vision.blue_robot + vision.yellow_robot
        for rob in all_robots:
            if not rob.visible:
                continue
            if hasattr(rob, 'id') and rob.id == 0:
                continue
            ox, oy = rob.x, rob.y
            ovx, ovy = rob.vel_x, rob.vel_y  # For dynamic
            dx_obs = rx - ox
            dy_obs = ry - oy
            dist_obs = math.hypot(dx_obs, dy_obs)
            if dist_obs > 1200 or dist_obs < 50:
                continue
            obs_list.append((ox, oy, ovx, ovy))
            if len(obs_list) >= 3:  # Limit to 3
                break

        num_obs = len(obs_list)
        if num_obs > 0:
            for ox, oy, ovx, ovy in obs_list:
                dist_obs = math.hypot(rx - ox, ry - oy)
                h = float(dist_obs - r_safe)  # FIXED: Ensure float
                
                # Gradient: grad_h = [(rx-ox)/dist, (ry-oy)/dist]
                grad_h_x = float((rx - ox) / (dist_obs + 1e-6))
                grad_h_y = float((ry - oy) / (dist_obs + 1e-6))
                
                # L_f h contrib
                A_v = float(grad_h_x * math.cos(theta) + grad_h_y * math.sin(theta))
                h_dot_obs = float(- (grad_h_x * ovx + grad_h_y * ovy))
                
                # Stronger A_w for turning
                angle_to_obs = math.atan2(oy - ry, ox - rx)  # To obs (note: from robot to obs)
                angle_diff_obs = angle_to_obs - theta
                angle_diff_obs = (angle_diff_obs + math.pi) % (2 * math.pi) - math.pi
                turn_strength = 4.0 if dist_obs < 600 else 2.5
                A_w = float(- dist_obs * math.sin(angle_diff_obs) * turn_strength)
                
                A_cbf = np.array([A_v, A_w], dtype=float)
                
                # Adaptive gamma
                gamma = float(3.0 if dist_obs < 600 else 1.5)

                def constraint_func(u, A=A_cbf, h_val=h, g=gamma, h_obs=h_dot_obs):
                    return np.dot(A, u) + g * h_val + h_obs
                
                constraints.append({'type': 'ineq', 'fun': constraint_func})
        
        # Objective: min ||u - u_ref||^2
        def objective(u):
            return np.sum((u - u_ref)**2)

        # Bounds
        bounds = [(-max_v, max_v), (-max_w, max_w)]
        
        # Solve QP
        vx_cmd, vw_cmd = v_body, w_body  # Default
        if len(constraints) > 0:
            result = minimize(objective, u_ref, method='SLSQP', bounds=bounds, constraints=constraints, 
                              options={'maxiter': 200, 'ftol': 1e-4})
            if result.success:
                u_safe = result.x
                vx_cmd, vw_cmd = u_safe[0], u_safe[1]
                print(f"QP success: {num_obs} obs, vw_cmd={vw_cmd:.2f}")
            else:
                print(f"QP failed: {result.message}, scaled ref")
                vx_cmd = v_body * 0.8
                vw_cmd = w_body * 0.8
        else:
            print("No obstacles, nominal input")

        # Limit velocities again
        vx_cmd = max(-max_v, min(max_v, vx_cmd))
        vw_cmd = max(-max_w, min(max_w, vw_cmd))

        # Apply low-pass filter
        vx_cmd = alpha * prev_vx + (1 - alpha) * vx_cmd
        vw_cmd = alpha * prev_vw + (1 - alpha) * vw_cmd
        prev_vx = vx_cmd
        prev_vw = vw_cmd

        # Send command
        action.sendCommand(vx=vx_cmd, vw=vw_cmd)

        time.sleep(0.01)