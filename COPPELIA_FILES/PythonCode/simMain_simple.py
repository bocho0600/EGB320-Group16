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
	
	def start(self):
		pass

	def update(self):
		pass

def State_Wander(State):
	def __init__(self, packerBotSim):
		self.pbs = packerBotSim
		self.found_aisle = False
		self.wander_bias = 0
	
	def start(self):
		self.found_aisle = False
		self.wander_bias = 0

	def update(self):
		self.pbs.SetTargetVelocities()
		return STATE_WANDER


class RobotStates:
	def __init__(self):
		self.current_state = STATE_LOST
		self.current_phase = PHASE_COLLECT
		self.state_o = State_Wander(packerBotSim)

		

	def update(self, packerBotSim, itemRangeBearing, packingBayRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing):
		new_state = self.state_o.update()
		if new_state != self.current_state:
			self.current_state = new_state
			self.state_o = State_Wander(packerBotSim)
			self.state_o.start()
		


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
	# 			# Go to the packing station but don't do up it
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

			
	


# SET SCENE PARAMETERS
sceneParameters = SceneParameters()
sceneParameters.bayContents= np.random.random_integers(0,5,(6,4,3)) # Random item in each bay
sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl # specify a bowl in the bay in shelf 0 
# with x,y coords (3,1) (zero-indexed). Items are {bowl,mug,bottle,soccer,rubiks,cereal}.

# SET ROBOT PARAMETERS
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'	# specify if using differential (currently omni is not supported)

botState = RobotStates()

# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the Coppelia Simulator so don't have to Stop it manually when pressing CTRL+C

	try:
		packerBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
		packerBotSim.StartSimulator()
		packerBotSim.SetCameraPose(0.1, 0.1, 0)

		

		while True:

			itemRangeBearing, packingBayRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing = packerBotSim.GetDetectedObjects()
			
			# print(itemRangeBearing)
			# print(packingBayRangeBearing)
			# print(obstaclesRangeBearing)
			# print(rowMarkerRangeBearing)
			# print(shelfRangeBearing)
			# print()

			botState.update(packerBotSim, itemRangeBearing, packingBayRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing)

			#packerBotSim.SetTargetVelocities(0, 0)  # forward velocity, rotation
			packerBotSim.UpdateObjectPositions() # needs to be called once at the end of the main code loop

	except KeyboardInterrupt as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
		packerBotSim.StopSimulator()