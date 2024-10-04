import numpy as np, cv2
from enum import Enum
from math import cos, sin, pi
import time

from .Globals import *
from .Vision import VisionModule
# from .ItemCollection import ItemCollectionModule
from .PathProcess import PathProcess

STATE = Enum('STATE', [
	'LOST',
	'WANDER',
	'FIND_AISLE_FROM_OUTSIDE',
	'AISLE_DOWN',
	'AISLE_DOWN_BAY3',
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


	
	MAX_ROBOT_VEL = 0.13 # m/s
	ROTATIONAL_BIAS = 0.3 #tweak this parameter to be more or less aggressive with turns vs straight
	Kp = 2.4 # proportional term. beware if its too high we will try to go backwards for sharp turns
	MAX_ROBOT_ROT = pi/6 # rad/s
	RADIUS = 0.15 # how far to stay away from wall

	@classmethod
	def set_velocity(cls, fwd, rot, delta):

		# Specific.set_velocity(fwd, rot)
		PathProcess.set_velocity(fwd, rot)


	#region state machine
	# For each state define a start and update function stored in this dict
	current_state = STATE.WANDER
	

	@classmethod
	def init(cls, initial_state,instruction):
		
		cls.target_aisle = int(instruction[0])
		cls.target_bay = int(instruction[1])
		cls.target_side = instruction[2]
		cls.target_height = int(instruction[3])
		cls.target_object = instruction[4]
		print(f"Aisle: {cls.target_aisle}, Bay: {cls.target_bay}, Side: {cls.target_side}")

		cls.shelf_length = 112 #cm
		cls.bay_width = cls.shelf_length / 4
		cls.target_bay_distance = cls.shelf_length - cls.bay_width/2 - cls.target_bay*cls.bay_width
		print(f"We want to be {cls.target_bay_distance} cm from aisle marker {cls.target_aisle}")

		cls.current_state = initial_state
		cls.t_now = time.time()

		PathProcess.Start()
	
	@classmethod
	def end(cls):
		PathProcess.End()
	


	# Call current update function then update the current state
	@classmethod
	def update(cls,*args):
		t_last = cls.t_now
		cls.t_now = time.time()
		delta = cls.t_now - t_last

		debug_img = cls.try_localize(*args)

		return debug_img
	

	has_pose_estimate = False
	time_since_localization = 0
	


	@classmethod
	def try_localize(cls, delta, debug_img, visout):
		# Check if we have enough information to localize absolutely (i.e., marker or packing station unobstructed)
		# vis pipeline visout should give us processed shelf contours with lines and facing/fake/away points, sorted by x,
		# marker and obstacle info etc. We want to project shelf points to floor and be able to find normals!

		# 	If not, use dead reckoning (from pathprocess, it has more accurate integration of the "odometry")
		#	just update our position, or if we don't have an estimate, update our position in the temp map (new made up term :)

		# 	Check if we have enough information to correct a dead-reckonning
		#	based on shelf points or obstructed marker/packing station
		#	the only situations where we cant localize or correct are too close to a wall or shelf, or facing a wall

		# If we know where we are
		# 	and vision matches, locate obstacles, plan path and execute.
		# 		Don't even react just follow the path
		# 	but it doesn't match up
		# 		use local navigation to avoid obstacles.
		#		if time since localization gets too high, has_pose_estimate = false and we go back to using temp map
		# If we don't know where we are
		# 	wander (try and find the packing station)

		# Temp map is observed points, track important points using dead reckoning only (cant be bothered to correct within temp map)
		# More recently observed points have higher priority
		# Points expire after some time
		# points include shelf and obstacles
		# to include an obstacle on the map we need to have a decently good estimate of our own position
		# Time since localization or distance travelled since localization could be a good indicator of how confident we are in our current position esitimate.
		# (Assume all successful localizations are exactly correct!)
		# When we start we ought to be able to localize straight away off the packing station.
		# If its obstructed just use the temp map to go towards it until its not obstructed.
		# When navigating temp map, 2d potential field is used (?). We have points and normals, just avoid the points and ignore normals

		# How can we 'control' to the centre of the aisle or to face an item using PID or state space?
		# For centre of aisle we can just set the trajectory to the center and (somehow) state-space ourselves along the trajectory
		# In this case we should only recalculate the trajectory if we stray too far from it,
		# rather than every loop recalculating a new trajectory that starts from our current position. 
		# Doing that could work, I think I will find out because it is much more feasible to implement
		# For facing an item, there can be a special state that continues setting exponentially decaying rotating paths until we are facing it
		# In this case we can ignore most of the vision processing and definitely ignore localization (we won't have enough info)
		# The repeated paths set could be exponential or just a single segment that expires when we should be facing it
		
		# Just need to consider that the motors have a minimum speed and we could get stuck not quite facing it and not generating enough torque to face it
		# Maybe a minimum speed offset should be introduced in the mobility code, then assume linear between that and max.
		# (i.e. what is the minimum pwm value to get any actual movement (max with no movement, +1)).
		# Even then there could be a minimum real speed which we can expect to move
		# in that case we just have to make sure that whenever we really want to move, we make sure we move faster than that.
		# Whenever we want to move really slowly for some reason, just move minimum speed for less time. It will work
		
	
		pass
	