import numpy as np, cv2
from enum import Enum
from math import cos, sin, pi
import time

from .Globals import *
from .Vision import VisionModule

STATE = Enum('STATE', [
	'LOST',
	'WANDER',
	'AISLE_DOWN',
	'FACE_BAY',
	'COLLECT_ITEM',
	'FACE_OUT',
	'WANDER_OUT',
	'FACE_PACKING',
	'APPROACH_PACKING',
	'ASCEND_PACKING',
	'DROP_ITEM',
	'DESCEND_PACKING',
	])

PHASE = Enum('PHASE', [
	'COLLECT',
	'DROPOFF'
	])


class NavigationModule:

	last_fwd = 0
	last_rot = 0


	
	MAX_ROBOT_VEL = 0.2 # m/s
	ROTATIONAL_BIAS = 0.9 #tweak this parameter to be more or less aggressive with turns vs straight
	Kp = 2.4 # proportional term. beware if its too high we will try to go backwards for sharp turns
	MAX_ROBOT_ROT = pi/3 * Kp # rad/s
	RADIUS = 0.15 # how far to stay away from wall

	@classmethod
	def set_velocity(cls, fwd, rot):
		cls.last_fwd = fwd
		cls.last_rot = rot
		Specific.set_velocity(fwd, rot)


	#region state machine
	# For each state define a start and update function stored in this dict
	state_callbacks = {}
	current_state = STATE.WANDER
	

	@classmethod
	def init(cls, initial_state,instruction):
		
		cls.target_aisle = int(instruction[0])
		cls.target_bay = int(instruction[1])
		cls.target_side = instruction[2]
		cls.target_height = int(instruction[3])
		cls.target_object = instruction[4]


		shelf_length = 112 #cm
		bay_width = shelf_length / 4
		cls.target_bay_distance = shelf_length - bay_width/2 - cls.target_bay*bay_width
		print(f"We want to be {cls.target_bay_distance} cm from aisle marker {cls.target_aisle}")

		cls.state_callbacks[STATE.LOST] = (cls.lost_start, cls.lost_update)
		cls.state_callbacks[STATE.WANDER] = (cls.wander_start, cls.wander_update)
		cls.state_callbacks[STATE.AISLE_DOWN] = (cls.aisle_down_start, cls.aisle_down_update)
		cls.state_callbacks[STATE.FACE_BAY] = (cls.face_bay_start, cls.face_bay_update)
		cls.state_callbacks[STATE.COLLECT_ITEM] = (cls.collect_item_start, cls.collect_item_update)

		cls.current_state = initial_state
		cls.t_now = time.time()
		cls.call_current_start()
		cls.forced_avoidance_start()
	
	

	# Call the start function for the current state
	@classmethod
	def call_current_start(cls):
		print(f"State is now {cls.current_state.name}")
		return cls.state_callbacks[cls.current_state][0]()
	
	# Call the update function for the current state
	@classmethod
	def call_current_update(cls, delta,*args):
		return cls.state_callbacks[cls.current_state][1](delta,*args)

	# Call current update function then update the current state
	@classmethod
	def update(cls,*args):
		t_last = cls.t_now
		cls.t_now = time.time()
		delta = cls.t_now - t_last

		new_state, debug_img = cls.call_current_update(delta,*args)
		if new_state != cls.current_state:
			cls.current_state = new_state
			cls.call_current_start()
		return debug_img
	#endregion

	#region Utility Functions
	
	
	
	@classmethod
	def expand_safety(cls, dist_map):
		d_theta = FOV_HORIZONTAL / SCREEN_WIDTH # angle per pixel horizontally

		# left to right
		
		should_expand = dist_map[:, 1]
		dist_map = dist_map[:,0].copy()

		effect = []
		for i, curr_dist in enumerate(dist_map):
			# update points in effect. Decrease the angle and discard expired points
			effect = [(dist, angle_left-d_theta) for dist,angle_left in effect if angle_left > d_theta]
			
			# How long would this point be effective for if we stored it
			new_angle_left = np.arctan(cls.RADIUS/curr_dist)

			# The maximum distance allowed at this point is the minimum dist of points still in effect
			# The minimum effective point.
			if len(effect)>0:
				dists = [dist for dist,angle_left in effect]
				min_dist_effect = min(dists)
				min_dist_index = dists.index(min_dist_effect)
			else:
				# if effect is empty include this point and continue
				dist_map[i] = curr_dist
				if should_expand[i]:
					effect.append((curr_dist, new_angle_left))
				continue

			if curr_dist < min_dist_effect: # If this point is a new minimum, store it
				# update the current point
				dist_map[i] = curr_dist
				# add the current point
				if should_expand[i]:
					effect.append((curr_dist, new_angle_left))
			else: # If this point is not a new minimum then take the minimum effective dist
				# update the current point
				dist_map[i] = min_dist_effect
				if new_angle_left > effect[min_dist_index][1] and should_expand[i]: # Still make this point effective if it might expire later than the current minimum effective point
					# add the current point
					effect.append((curr_dist, new_angle_left))

			
		# right to left
		effect = []
		for i, curr_dist in enumerate(reversed(dist_map)):
			# update points in effect. Decrease the angle and discard expired points
			effect = [(dist, angle_left-d_theta) for dist,angle_left in effect if angle_left > d_theta]
			
			# How long would this point be effective for if we stored it
			new_angle_left = np.arctan(cls.RADIUS/curr_dist)

			# The maximum distance allowed at this point is the minimum dist of points still in effect
			# The minimum effective point.
			if len(effect)>0:
				dists = [dist for dist,angle_left in effect]
				min_dist_effect = min(dists)
				min_dist_index = dists.index(min_dist_effect)
			else:
				# if effect is empty include this point and continue
				dist_map[-i-1] = curr_dist
				if should_expand[-1-i]:
					effect.append((curr_dist, new_angle_left))
				continue

			if curr_dist < min_dist_effect: # If this point is a new minimum, store it
				# update the current point
				dist_map[-1-i] = curr_dist
				# add the current point
				if should_expand[-1-i]:
					effect.append((curr_dist, new_angle_left))
			else: # If this point is not a new minimum then take the minimum effective dist
				# update the current point
				dist_map[-1-i] = min_dist_effect
				if new_angle_left > effect[min_dist_index][1] and should_expand[-1-i]: # Still make this point effective if it might expire later than the current minimum effective point
					# add the current point
					effect.append((curr_dist, new_angle_left))
		return dist_map

	@classmethod
	def forced_avoidance_start(cls):
		cls.last_left_dist = 2.0
		cls.last_right_dist = 2.0
		cls.avoid_right = 0
		cls.avoid_left = 0
		cls.left_obstacle_dist = None
		cls.right_obstacle_dist = None

	@classmethod
	def forced_avoidance(cls, dist_map):
		if dist_map is not None:
			# Forced Avoidance
			left_dist = dist_map[dist_map[:,1]>0][0,0]
			right_dist = dist_map[dist_map[:,1]>0][-1,0]
			print(f"{left_dist:.2f}, {right_dist:.2f}")
			
			change_in_left = left_dist - cls.last_left_dist
			change_in_right = right_dist - cls.last_right_dist
			
			# if we were within 0.3m of an obstacle and cant see it anymore
			# (edge distance increased by 0.2 or more,  *this only works for shelves, if there are narrow obstacles it will be weird* )
			if change_in_left > 0.2 and cls.last_left_dist < 0.4 and cls.last_fwd > 0:# and cls.last_rot > 0:
				print("AVOID LEFT!")
				cls.avoid_left = cls.last_left_dist * sin(pi/3)+0.2
				cls.left_obstacle_dist = cls.last_left_dist
			if change_in_right > 0.2 and cls.last_right_dist < 0.4 and cls.last_fwd > 0:# and cls.last_rot < 0:
				print("AVOID RIGHT!")
				cls.avoid_right = cls.last_right_dist * sin(pi/3)+0.2
				cls.right_obstacle_dist = cls.last_right_dist

			# if we are avoiding an obstacle just act like it is still there on the edge
			if cls.avoid_left > 0:
				dist_map[0,0] = cls.left_obstacle_dist
				dist_map[0,1] = 1 # treat as real point (expand in safety)
			if cls.avoid_right > 0:
				dist_map[-1,0] = cls.right_obstacle_dist
				dist_map[-1,1] = 1 # treat as real point (expand in safety)
			
			cls.last_left_dist = left_dist
			cls.last_right_dist = right_dist
			return dist_map
		else:
			cls.last_left_dist = 0.3
			cls.last_right_dist = 0.3
	
	@classmethod
	def forced_avoidance_timer_update(cls, fwd, delta):
		if cls.avoid_right>0:
			cls.avoid_right -= fwd * delta
		if cls.avoid_left>0:
			cls.avoid_left -= fwd * delta
	#endregion
	
	#region State Callbacks
	@classmethod
	def lost_start(cls):
		print("Lost!")

	@classmethod
	def lost_update(cls,delta, debug_img, *args):
		return STATE.WANDER, debug_img

	@classmethod
	def wander_start(cls):
		pass
		

	@classmethod
	def wander_update(cls, delta, debug_img, aisle, marker_distance, marker_bearing, points, projected_floor, dist_map):

		if marker_distance is not None and abs(marker_bearing) < 0.8:
			return STATE.AISLE_DOWN, debug_img

		if points is not None and points.size > 0:

			dist_map = cls.forced_avoidance(dist_map)
			if cls.avoid_left > 0:
				debug_img = cv2.putText(debug_img, "A", (20,20), 0, 1, (0,0,255), 2)
			if cls.avoid_right > 0:
				debug_img = cv2.putText(debug_img, "A", (SCREEN_WIDTH-20,20), 0, 1, (0,0,255), 2)

			safety_map = cls.expand_safety(dist_map)
			
			debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw


			if abs(safety_map.max() - safety_map.min()) < 0.01:
				# no safe path- turn and dont move forward
				
				goal_error = (dist_map.argmax() - SCREEN_WIDTH/2) / (SCREEN_WIDTH) * pi/3
				rotational_vel = goal_error*cls.Kp

				#forward_vel = MAX_ROBOT_VEL * (1.0 - ROTATIONAL_BIAS*abs(rotational_vel)/MAX_ROBOT_ROT)
				forward_vel = 0

				debug_img = cv2.drawMarker(debug_img, (dist_map.argmax(), int(SCREEN_HEIGHT - dist_map.max()/2 * SCREEN_HEIGHT)), (0,0,255), cv2.MARKER_STAR, 10)

			else:
				# move into the longest safe path
				goal_error = (safety_map.argmax() - SCREEN_WIDTH/2) / (SCREEN_WIDTH) * pi/3
				rotational_vel = goal_error*cls.Kp
				forward_vel = cls.MAX_ROBOT_VEL * (1.0 - cls.ROTATIONAL_BIAS*abs(rotational_vel)/cls.MAX_ROBOT_ROT)

				debug_img = cv2.drawMarker(debug_img, (safety_map.argmax(), int(SCREEN_HEIGHT - safety_map.max()/2 * SCREEN_HEIGHT)), (0,255,255), cv2.MARKER_STAR, 10)
			
			
		else:
			cls.forced_avoidance(None)
			rotational_vel = pi/2
			forward_vel = 0



		cls.forced_avoidance_timer_update(forward_vel/5, delta*8)
		cls.set_velocity(forward_vel/5,rotational_vel/5)

		return STATE.WANDER, debug_img
	
	
	@classmethod
	def aisle_down_start(cls):
		pass
		

	@classmethod
	def aisle_down_update(cls, delta, debug_img, aisle, marker_distance, marker_bearing, points, projected_floor, dist_map):
		if marker_distance is None:
			return STATE.LOST, debug_img
		
		if marker_distance < cls.target_bay_distance:
			return STATE.FACE_BAY, debug_img

		if points is not None and points.size > 0:

			dist_map = cls.forced_avoidance(dist_map)
			if cls.avoid_left > 0:
				debug_img = cv2.putText(debug_img, "A", (20,20), 0, 1, (0,0,255), 2)
			if cls.avoid_right > 0:
				debug_img = cv2.putText(debug_img, "A", (SCREEN_WIDTH-20,20), 0, 1, (0,0,255), 2)

			safety_map = cls.expand_safety(dist_map)
			# repulsive_map = (safety_map < 0.3)*1

			attractive_map = np.zeros(safety_map.size)
			for i in range(safety_map.size):
				bearing = (i/SCREEN_WIDTH - 1/2) * FOV_HORIZONTAL
				attractive_map[i] = 1.0 - 0.2 * abs(bearing - marker_bearing)
			
			debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (255, 255, 0), 1) # draw

			if abs(safety_map.min() - safety_map.max()) < 0.01:
				forward_vel = -cls.MAX_ROBOT_VEL/2
				rotational_vel = 0
			else:
				potential_map = attractive_map + safety_map

				debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - potential_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw
				debug_img = cv2.drawMarker(debug_img, (potential_map.argmax(), int(SCREEN_HEIGHT - potential_map.max()/2 * SCREEN_HEIGHT)), (0,255,255), cv2.MARKER_STAR, 10)
				

				goal_error = (potential_map.argmax() - SCREEN_WIDTH/2) / (SCREEN_WIDTH) * pi/3
				rotational_vel = goal_error*cls.Kp
				forward_vel = cls.MAX_ROBOT_VEL * (1.0 - 4.0 * cls.ROTATIONAL_BIAS*abs(rotational_vel)/cls.MAX_ROBOT_ROT)

			
			cls.forced_avoidance_timer_update(forward_vel/5, delta)
			cls.set_velocity(forward_vel/5,rotational_vel/5)
		else:
			return STATE.LOST, debug_img
		

		return STATE.AISLE_DOWN, debug_img
	
	@classmethod
	def face_bay_start(cls):
		cls.turn_to_bay_remaining = pi/2
		cls.set_velocity(0,0)
	
	@classmethod
	def face_bay_update(cls, delta, debug_img, *args):
		
		rot = 0.35
		
		cls.turn_to_bay_remaining -= rot*delta
		if cls.turn_to_bay_remaining <= 0:
			return STATE.COLLECT_ITEM, debug_img

		if cls.target_side == "Left":
			rot = -rot
		cls.set_velocity(0, rot)

		return STATE.FACE_BAY, debug_img


	@classmethod
	def collect_item_start(cls):
		cls.set_velocity(0,0)
	
	@classmethod
	def collect_item_update(cls, delta, debug_img, *args):
		return STATE.COLLECT_ITEM, debug_img
	
	#endregion