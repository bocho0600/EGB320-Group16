#!/usr/bin/python

# import the  bot module - this will include math, time, numpy (as np) and vrep python modules
from warehousebot_lib import *
import numpy as np, cv2
import time
from math import cos, sin

STATE_LOST = 0
STATE_WANDER = 1
STATE_AISLE_DOWN = 2
STATE_FACE_BAY = 3
STATE_COLLECT_ITEM = 4
STATE_FACE_OUT = 5
STATE_WANDER_OUT = 6
STATE_FACE_PACKING = 7
STATE_APPROACH_PACKING = 8
STATE_ASCEND_PACKING = 9
STATE_DROP_ITEM = 10
STATE_DESCEND_PACKING = 11

PHASE_COLLECT = 0
PHASE_DROPOFF = 1

class State:
	def __init__(self, packerBotSim):
		self.pbs = packerBotSim
		#self.start()
	
	def set_velocity(self, fwd, rot):
		self.pbs.SetTargetVelocities(fwd, -rot)
	
	def get_image(self):
		_, img = self.pbs.GetCameraImage()

		img = convert_image(img)
		img = cv2.resize(img, (SCREEN_WIDTH, SCREEN_HEIGHT))
		return img

	def expand_safety(self, dist_map):
		d_theta = np.pi/3 / SCREEN_WIDTH # angle per pixel horizontally
		radius = 0.15 # how far to stay away from wall

		# left to right
		dist_map = dist_map.copy()

		effect = []
		for i, curr_dist in enumerate(dist_map):
			# update points in effect. Decrease the angle and discard expired points
			effect = [(dist, angle_left-d_theta) for dist,angle_left in effect if angle_left > d_theta]
			# add the current point
			effect.append((curr_dist, np.arctan(radius/curr_dist)))
			# update the current point
			dist_map[i] = min([dist for dist,angle_left in effect])
		
		# right to left
		effect = []
		for i, curr_dist in enumerate(reversed(dist_map)):
			# update points in effect. Decrease the angle and discard expired points
			effect = [(dist, angle_left-d_theta) for dist,angle_left in effect if angle_left > d_theta]
			# add the current point
			effect.append((curr_dist, np.arctan(radius/curr_dist)))
			# update the current point
			dist_map[-1-i] = min([dist for dist,angle_left in effect])
		return dist_map



	def start(self):
		# override
		raise NotImplementedError("start should be overidden")

	def update(self):
		# override
		raise NotImplementedError("update should be overidden")

class State_Wander(State):
	
	def start(self):
		pass

	def update(self):
		# t_last = t_now
		# t_now = time.time()
		# delta = t_now-t_last

		img = self.get_image()
		
		contour, mask = findFloor(img)
		
		res = img.copy()
		#res = cv2.bitwise_and(img, img, mask=mask)
		
		floor_contour = None
		xmax = -1
		for cont in contour:
			if cv2.contourArea(cont) > 200:
				bounds = cv2.boundingRect(cont) # left, top, width, height
				if bounds[0] < xmax:
					# only consider new contours to the right of what we have already seen
					continue
				xmax = bounds[0] + bounds[2]

				# npc is array of contour points along the edge (and around the outside) of the object
				npc = np.array(cont)
				npc = npc[:,0,:]

				
				
				# discard points on the edge of the screen and sort left to right
				npc = npc[(npc[:,1] > 0) & (npc[:,1] < SCREEN_HEIGHT-1) & (npc[:,0] > 0) & (npc[:,0] < SCREEN_WIDTH-1), :]
				npc = npc[np.argsort(npc[:, 0]), :]

				if npc.size < 3:
					continue

				# add a 0 point before and after so that the dist map counts blank spaces correctly
				if npc[0,0] > 1: # any with 0 were removed so first being =1 is acceptable
					npc = np.r_[[[npc[0,0]-1, SCREEN_HEIGHT-1]], npc]
				if npc[-1,0] < SCREEN_WIDTH-2:# any with SCREEN_WIDTH-1 were removed so first being =SCREEN_WIDTH-2 is acceptable
					npc = np.r_[npc, [[npc[-1,0]+1, SCREEN_HEIGHT-1]]]	

				# draw
				res = cv2.polylines(res, [npc], False, (0, 0, 255), 1) # draw
				
				# append to floor_contour
				if floor_contour is None:
					floor_contour = npc
				else:
					floor_contour = np.r_[floor_contour, npc]
				

		
	
		if floor_contour is not None:
			# project contour onto the ground
			projected_floor = project_point_to_ground(floor_contour)
			
			# distances of each point. However each point does not match 1 to 1 with pixels
			dist_real = np.sqrt(projected_floor[:,0]**2 + projected_floor[:,1]**2)
			
			# find which point matches to which pixel considering duplicates and skips
			dist_map = np.zeros(SCREEN_WIDTH, np.float32)
			j = 0
			for i in range(len(dist_map)):
				while j < len(floor_contour)-1 and floor_contour[j, 0] < i:
					j += 1
				dist_map[i] = dist_real[j]

			safety_map = self.expand_safety(dist_map)
			
			res = cv2.polylines(res, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - dist_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 0), 1) # draw
			res = cv2.polylines(res, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw

			MAX_ROBOT_VEL = 0.2 # m/s
			MAX_ROBOT_ROT = np.pi/3 # rad/s
			ROTATIONAL_BIAS = 0.95 #tweak this parameter to be more or less aggressive with turns vs straight
			Kp = 0.8 # proportional term. beware if its too high we will try to go backwards for sharp turns

			if safety_map.max() == safety_map.min():
				# no safe path- turn and dont move forward
				forward_vel = 0
				goal_error = (dist_map.argmax() - SCREEN_WIDTH/2) / (SCREEN_WIDTH) * np.pi/3
				rotational_vel = goal_error*Kp

				res = cv2.drawMarker(res, (dist_map.argmax(), int(SCREEN_HEIGHT - dist_map.max()/2 * SCREEN_HEIGHT)), (0,0,255), cv2.MARKER_STAR, 10)

			else:
				# move into the longest safe path
				goal_error = (safety_map.argmax() - SCREEN_WIDTH/2) / (SCREEN_WIDTH) * np.pi/3
				rotational_vel = goal_error*Kp
				forward_vel = MAX_ROBOT_VEL * (1.0 - ROTATIONAL_BIAS*abs(rotational_vel)/MAX_ROBOT_ROT)

				res = cv2.drawMarker(res, (safety_map.argmax(), int(SCREEN_HEIGHT - safety_map.max()/2 * SCREEN_HEIGHT)), (0,255,255), cv2.MARKER_STAR, 10)

		else:
			rotational_vel = np.pi/2
			forward_vel = 0
		
		
		

		cv2.imshow("res", res)
		cv2.waitKey(1)

		print(forward_vel, rotational_vel)
		self.set_velocity(forward_vel/5,rotational_vel/5)
		return STATE_WANDER


class RobotStateMachine:
	def __init__(self, packerBotSim):
		self.current_state = STATE_WANDER
		self.current_phase = PHASE_COLLECT
		self.all_states = [None, State_Wander]
		self.pbs = packerBotSim
		
		# Initialise all states in state_o
		self.state_o = []
		for state in self.all_states:
			if state is not None:
				self.state_o.append(state(self.pbs))
			else:
				self.state_o.append(None)
		
		self.get_current_state().start()

	def get_current_state(self):
		return self.state_o[self.current_state]

	def update(self, itemRangeBearing, packingBayRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing):
		state_c = self.get_current_state()
		new_state = state_c.update()
		if new_state != self.current_state:
			self.current_state = new_state
			self.get_current_state().start()
		


	# def update(self, packerBotSim, itemRangeBearing, packingBayRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing):
	# 	match self.current_state:
	# 		case 0: # STATE_LOST
	# 			# Print error message
	# 			# -> STATE_WANDER
	# 			pass
	# 		case 1: # STATE_WANDER
	# 			# Just wander around until a marker is found
	# 			# Avoid obstacles
	# 			# -> STATE_AISLE_DOWN
	# 			pass
	# 		case 2: # STATE_AISLE_DOWN
	# 			# Go down the aisle to the right bay
	# 			# Avoid obstacles
	# 			# -> STATE_FACE_BAY
	# 			pass
	# 		case 3: # STATE_FACE_BAY
	# 			# Turn to the bay and position in front of it
	# 			# -> STATE_COLLECT_ITEM
	# 			pass
	# 		case 4: # STATE_COLLECT_ITEM
	# 			# Reach up ad grab the item
	# 			# -> STATE_FACE_OUT
	# 			pass
	# 		case 5: # STATE_FACE_OUT
	# 			# Turn to face out of the aisle (need to remember which side we're on)
	# 			# -> STATE_WANDER_OUT
	# 			pass
	# 		case 6: # STATE_WANDER_OUT
	# 			# Go forward out of the aisle until we see the pakcing station.
	# 			# If we hit a wall turn right
	# 			# Don't enter an aisle
	# 			# Avoid obstacles
	# 			# -> STATE_FACE_PACKING
	# 			pass
	# 		case 7: # STATE_FACE_PACKING
	# 			# Turn to face packing station
	# 			# -> STATE_APPROACH_PACKING
	# 			pass
	# 		case 8: # STATE_APPROACH_PACKING
	# 			# Go to the packing station but don't go up it
	# 			# Avoid obstacles
	# 			# STATE_ASCEND_PACKING
	# 			pass
	# 		case 9: # STATE_ASCEND_PACKING
	# 			# Go slowly up the packing station
	# 			# -> STATE_DROP_ITEM
	# 			pass
	# 		case 10: # STATE_DROP_ITEM
	# 			# (Lower the item)
	# 			# Drop the item
	# 			# -> STATE_DESCEND_PACKING
	# 			pass
	# 		case 11: # STATE_DESCEND_PACKING
	# 			# Go backwards slowly down the packing station
	# 			# Then turn 180 degrees
	# 			# -> STATE_WANDER
	# 			pass

	# 	packerBotSim.SetTargetVelocities(0,0)

			
	
def convert_image(img):
	img = np.reshape((np.array(img).astype(np.uint8)), (480,640,3))
	return cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)

color_ranges = {
            'floor': (np.array([0, 0, 80]), np.array([0, 0, 103])),
            'wall': (np.array([0, 0, 146]), np.array([30, 1, 255])),
            'blue': (np.array([3, 171, 54]), np.array([3, 175, 112])),
            'black': (np.array([0, 0, 0]), np.array([0, 0, 0])),
            'yellow': (np.array([99, 216, 130]), np.array([99, 217, 187])),
            'green': (np.array([40, 90, 0]), np.array([70, 255, 180])),
            'orange1': (np.array([5, 150, 150]), np.array([20, 255, 255])),
            'orange2': (np.array([165, 150, 150]), np.array([180, 255, 255])),
      }

def findFloor(img):
	imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
	FloorMask = cv2.inRange(imgHSV, color_ranges['floor'][0], color_ranges['floor'][1])
	FloorMask = cv2.morphologyEx(FloorMask, cv2.MORPH_CLOSE, np.ones((3,3)))
	contoursFloor, _ = cv2.findContours(FloorMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
	return contoursFloor, FloorMask

def project_point_to_ground(screen_coords):
	x = screen_coords[:, 0]
	y = screen_coords[:, 1]

	
	# Coordinates on a plane in front of the camera, relative to camera
	cx = -(x-SCREEN_WIDTH/2) / SCREEN_WIDTH
	cy = -(y-SCREEN_HEIGHT/2) / SCREEN_WIDTH
	cz = 1 / ANGLE_H * np.ones(cx.shape)
	cpi = np.array([cz,-cx,cy])


	# coordinates on the ground, relative to camera
	cpo = np.dot(normal_camera, r_camera) / np.dot(normal_camera, cpi) * cpi
	cpo = np.array([cpo[0,:], cpo[1,:], cpo[2,:], np.ones(cpo[0,:].shape)])

	# coodinates on the ground, relative to robot
	rpo = np.matmul(camera_to_robot, cpo)
	return rpo[0:2,:].T

# SET SCENE PARAMETERS
sceneParameters = SceneParameters()
sceneParameters.bayContents= np.random.random_integers(0,5,(6,4,3)) # Random item in each bay
sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl # specify a bowl in the bay in shelf 0 
# with x,y coords (3,1) (zero-indexed). Items are {bowl,mug,bottle,soccer,rubiks,cereal}.

# SET ROBOT PARAMETERS
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'	# specify if using differential (currently omni is not supported)

SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240


# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the Coppelia Simulator so don't have to Stop it manually when pressing CTRL+C

	try:
		packerBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
		packerBotSim.StartSimulator()
		packerBotSim.SetCameraPose(0.1, 0.1, 0)

		botStateMachine = RobotStateMachine(packerBotSim)		

		ANGLE_H = packerBotSim.horizontalViewAngle
		ANGLE_V = packerBotSim.verticalViewAngle
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
		
		camera_to_robot = np.matmul(camera_to_robot_translate,camera_to_robot_rotate)
		robot_to_camera = np.linalg.inv(camera_to_robot)
		robot_to_camera_translate = np.linalg.inv(camera_to_robot_translate)
		robot_to_camera_rotate = np.linalg.inv(camera_to_robot_rotate)

		# Normal and point of the ground plane, relative to camera
		normal_camera = np.matmul(robot_to_camera_rotate , np.array([[0,0,1,1]]).T)[0:3, 0]
		r_camera = np.matmul(robot_to_camera , np.array([[0,0,0,1]]).T)[0:3, 0]

		while True:

			itemRangeBearing, packingBayRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing = packerBotSim.GetDetectedObjects()
			
			# print(itemRangeBearing)
			# print(packingBayRangeBearing)
			# print(obstaclesRangeBearing)
			# print(rowMarkerRangeBearing)
			# print(shelfRangeBearing)
			# print()

			botStateMachine.update(itemRangeBearing, packingBayRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing)

			#packerBotSim.SetTargetVelocities(0, 0)  # forward velocity, rotation
			packerBotSim.UpdateObjectPositions() # needs to be called once at the end of the main code loop

	except KeyboardInterrupt as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
		packerBotSim.StopSimulator()