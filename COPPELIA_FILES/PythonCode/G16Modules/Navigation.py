import numpy as np, cv2
from enum import Enum
from math import cos, sin, pi
import time

from .Globals import *
from .SimVision import Vision

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


class Navigation:

	last_fwd = 0
	last_rot = 0


	MAX_ROBOT_VEL = 0.2 # m/s
	ROTATIONAL_BIAS = 0.9 #tweak this parameter to be more or less aggressive with turns vs straight
	Kp = 1.0 # proportional term. beware if its too high we will try to go backwards for sharp turns
	MAX_ROBOT_ROT = pi/3 * Kp # rad/s
	RADIUS = 0.2 # how far to stay away from wall

	
	# For each state define a start and update function stored in this dict
	state_callbacks = {}

	# set_velocity (fwd (m/s), rot (rad/s, right is positive))
	set_velocity_callback = None

	@classmethod
	def initialise_callbacks(cls):
		cls.state_callbacks[STATE.WANDER] = (cls.wander_start, cls.wander_update)
	
	#region Utility Functions
	@classmethod
	def calculate_view_transforms(cls):
		#ANGLE_H = packerBotSim.horizontalViewAngle
		#ANGLE_V = packerBotSim.verticalViewAngle

		DIST_X = 0#packerBotSim.robotParameters.cameraDistanceFromRobotCenter
		DIST_Z = 0.0752#packerBotSim.robotParameters.cameraHeightFromFloor
		# tilt = packerBotSim.robotParameters.cameraTilt
		TILT = 1.5 * 3.1415926535 / 180
		# tilt is meant to be 0 but it is slightly off in simulator

		# Precalculate Transformation Matrices and ground plane
		camera_to_robot_rotate = np.array([
				[cos(TILT), 0, -sin(TILT), 0],
				[        0, 1,          0, 0],
				[sin(TILT), 0,  cos(TILT), 0],
				[        0, 0,          0, 1]])
		camera_to_robot_translate = np.array([
				[1, 0, 0, DIST_X],
				[0, 1, 0,      0],
				[0, 0, 1, DIST_Z],
				[0, 0, 0,      1]])
		
		cls.camera_to_robot = np.matmul(camera_to_robot_translate,camera_to_robot_rotate)
		robot_to_camera = np.linalg.inv(cls.camera_to_robot)
		#robot_to_camera_translate = np.linalg.inv(camera_to_robot_translate)
		robot_to_camera_rotate = np.linalg.inv(camera_to_robot_rotate)

		# Normal and point of the ground plane, relative to camera
		cls.normal_camera = np.matmul(robot_to_camera_rotate , np.array([[0,0,1,1]]).T)[0:3, 0]
		cls.r_camera = np.matmul(robot_to_camera , np.array([[0,0,0,1]]).T)[0:3, 0]

	@classmethod
	def set_velocity(cls, fwd, rot):
		cls.last_fwd = fwd
		cls.last_rot = rot
		if cls.set_velocity_callback is not None:
			cls.set_velocity_callback(fwd, rot)
		else:
			print("Warning: No set_velocity_callback on Navigation module!")
	
	
	@classmethod
	def expand_safety(cls,dist_map):
		d_theta = FOV_HORIZONTAL / SCREEN_WIDTH # angle per pixel horizontally

		# left to right
		dist_map = dist_map.copy()

		effect = []
		for i, curr_dist in enumerate(dist_map):
			# update points in effect. Decrease the angle and discard expired points
			effect = [(dist, angle_left-d_theta) for dist,angle_left in effect if angle_left > d_theta]
			# add the current point
			effect.append((curr_dist, np.arctan(cls.RADIUS/curr_dist)))
			# update the current point
			dist_map[i] = min([dist for dist,angle_left in effect])
		
		# right to left
		effect = []
		for i, curr_dist in enumerate(reversed(dist_map)):
			# update points in effect. Decrease the angle and discard expired points
			effect = [(dist, angle_left-d_theta) for dist,angle_left in effect if angle_left > d_theta]
			# add the current point
			effect.append((curr_dist, np.arctan(cls.RADIUS/curr_dist)))
			# update the current point
			dist_map[-1-i] = min([dist for dist,angle_left in effect])
		return dist_map

	
	@classmethod
	def project_point_to_ground(cls,screen_coords):
		x = screen_coords[:, 0]
		y = screen_coords[:, 1]

		# Coordinates on a plane in front of the camera, relative to camera
		cx = -(x-SCREEN_WIDTH/2) / SCREEN_WIDTH
		cy = -(y-SCREEN_HEIGHT/2) / SCREEN_WIDTH
		cz = 1 / FOV_HORIZONTAL * np.ones(cx.shape)
		cpi = np.array([cz,-cx,cy])


		# coordinates on the ground, relative to camera
		cpo = np.dot(cls.normal_camera, cls.r_camera) / np.dot(cls.normal_camera, cpi) * cpi
		cpo = np.array([cpo[0,:], cpo[1,:], cpo[2,:], np.ones(cpo[0,:].shape)])

		# coodinates on the ground, relative to robot
		rpo = np.matmul(cls.camera_to_robot, cpo)
		return rpo[0:2,:].T
	
	@classmethod
	def timer_init(cls):
		cls.t_now = time.time()

	@classmethod
	def timer_update(cls):
		t_last = cls.t_now
		cls.t_now = time.time()
		return cls.t_now - t_last

	@staticmethod
	def combine_contour_points(contours):
		# Are must be greater then 200. Then, sort the remaining ones by their area
		contours = [cont for cont in contours if cv2.contourArea(cont) > 200]
		contours = sorted(contours, key=lambda cont: -cv2.contourArea(cont))
		
		combined_contour = None
		xmax = -1
		xmin = SCREEN_WIDTH+1
		for cont in contours:

			bounds = cv2.boundingRect(cont) # left, top, width, height

			# only consider new contours strictly outside of what we have already seen
			# * this means that a contour between two already seen will be excluded! this should be pretty rare though *
			if bounds[0]+bounds[2] <= xmin:
				xmin = bounds[0]
				xmax = max(xmax, bounds[0]+bounds[2])
			elif bounds[0] >= xmax:
				xmax = bounds[0] + bounds[2]
				xmin = min(xmin, bounds[0])
			else:
				continue
			

			# npc is array of contour points along the edge (and around the outside) of the object
			npc = np.array(cont)
			npc = npc[:,0,:]

			
			
			# discard points on the edge of the screen and sort left to right (if two points are at same x prefer larger y (lower on the screen))
			# this is so later when we get unique elements the lower one is kept
			npc = npc[(npc[:,1] > 0) & (npc[:,1] < SCREEN_HEIGHT-1) & (npc[:,0] > 0) & (npc[:,0] < SCREEN_WIDTH-1), :]
			npc = npc[np.argsort(npc[:, 0] + 1.0-npc[:, 1]/SCREEN_HEIGHT), :]

			if npc.size < 3:
				continue

			# this is too annoying to do in the case of multiple contours
			# add a 0 point before and after so that the dist map counts blank spaces correctly
			# if npc[0,0] > 1: # any with 0 were removed so first being =1 is acceptable
			# 	npc = np.r_[[[npc[0,0]-1, SCREEN_HEIGHT-1]], npc]
			# if npc[-1,0] < SCREEN_WIDTH-2:# any with SCREEN_WIDTH-1 were removed so first being =SCREEN_WIDTH-2 is acceptable
			# 	npc = np.r_[npc, [[npc[-1,0]+1, SCREEN_HEIGHT-1]]]	

			
			# # append to comvined_contour
			if combined_contour is None:
				combined_contour = npc
			else:
				combined_contour = np.r_[combined_contour, npc]
		return combined_contour

	@classmethod
	def project_and_filter_contour(cls,contour_points):
		if contour_points is not None:
			# make sure it is sorted from left to right and only keep one point per column. (lower points preferred)	
			contour_points = contour_points[np.unique(contour_points[:, 0], return_index=True)[1]]


			# project contour onto the ground
			projection = cls.project_point_to_ground(contour_points)

			# discard points above the horizon
			mask = projection[:, 0] >= 0
			projection = projection[mask]
			contour_points = contour_points[mask]
			return contour_points, projection
		else:
			return None, None

	@staticmethod
	def get_dist_map(contour_points, projection):
		# distances of each point. However each point does not match 1 to 1 with pixels
		dist_real = np.sqrt(projection[:,0]**2 + projection[:,1]**2)
		
		# find which point matches to which pixel considering duplicates and skips
		dist_map = np.zeros(SCREEN_WIDTH, np.float32)
		j = 0
		for i in range(len(dist_map)):
			while j < len(contour_points)-1 and contour_points[j, 0] < i:
				j += 1
			dist_map[i] = dist_real[j]
		return dist_map

	@classmethod
	def forced_avoidance(cls, dist_map):
		if dist_map is not None:
			# Forced Avoidance
			left_dist = dist_map[0]
			right_dist = dist_map[-1]
			
			change_in_left = left_dist - cls.last_left_dist
			change_in_right = right_dist - cls.last_right_dist
			
			# if we were within 0.3m of an obstacle and cant see it anymore
			# (edge distance increased by 0.2 or more,  *this only works for shelves, if there are narrow obstacles it will be weird* )
			if change_in_left > 0.2 and cls.last_left_dist < 0.3 and cls.last_fwd > 0 and cls.last_rot > 0:
				print("AVOID LEFT!")
				cls.avoid_left = cls.last_left_dist * sin(pi/3)+0.2
				cls.left_obstacle_dist = cls.last_left_dist
			if change_in_right > 0.2 and cls.last_right_dist < 0.3 and cls.last_fwd > 0 and cls.last_rot < 0:
				print("AVOID RIGHT!")
				cls.avoid_right = cls.last_right_dist * sin(pi/3)+0.2
				cls.right_obstacle_dist = cls.last_right_dist

			# if we are avoiding an obstacle just act like it is still there on the edge
			if cls.avoid_left > 0:
				dist_map[0] = cls.left_obstacle_dist
			if cls.avoid_right > 0:
				dist_map[-1] = cls.right_obstacle_dist
			
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
	def wander_start(cls):
		cls.last_left_dist = 2.0
		cls.last_right_dist = 2.0
		cls.avoid_right = 0
		cls.avoid_left = 0
		cls.left_obstacle_dist = None
		cls.right_obstacle_dist = None
		cls.timer_init()
		

	@classmethod
	def wander_update(cls):
		delta = cls.timer_update()

		img = Vision.get_image()
		debug_img = img.copy()

		# exclude small areas and consider in order of size
		contour, mask = Vision.findFloor(img)
		floor_contour = cls.combine_contour_points(contour)
		floor_contour, projected_floor = cls.project_and_filter_contour(floor_contour)

		if floor_contour is not None and floor_contour.size > 0:
			# draw
			debug_img = cv2.polylines(debug_img, [floor_contour], False, (0, 0, 255), 1) # draw

			dist_map = cls.get_dist_map(floor_contour, projected_floor)

			dist_map = cls.forced_avoidance(dist_map)
			if cls.avoid_left > 0:
				debug_img = cv2.putText(debug_img, "A", (20,20), 0, 1, (0,0,255), 2)
			if cls.avoid_right > 0:
				debug_img = cv2.putText(debug_img, "A", (SCREEN_WIDTH-20,20), 0, 1, (0,0,255), 2)

			safety_map = cls.expand_safety(dist_map)
			
			debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - dist_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 0), 1) # draw
			debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw


			if safety_map.max() == safety_map.min():
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
			

		cv2.imshow("res", debug_img)
		cv2.waitKey(1)

		forward_vel = forward_vel/5
		rotational_vel = rotational_vel/5

		cls.forced_avoidance_timer_update(forward_vel, delta*8)
		cls.set_velocity(forward_vel,rotational_vel)

		return STATE.WANDER
	#endregion