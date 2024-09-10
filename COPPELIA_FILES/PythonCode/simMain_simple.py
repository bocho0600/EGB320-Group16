#!/usr/bin/python

# import the  bot module - this will include math, time, numpy (as np) and vrep python modules
from warehousebot_lib import *
import numpy as np, cv2
from math import cos, sin, pi

from G16Modules.Globals import *
from G16Modules.Navigation import Navigation, STATE, PHASE
from G16Modules.SimVision import Vision
from G16Modules.RobotStateMachine import RobotStateMachine



#region Main

# SET SCENE PARAMETERS
sceneParameters = SceneParameters()
sceneParameters.bayContents= np.random.random_integers(0,5,(6,4,3)) # Random item in each bay
sceneParameters.bayContents[0,3,1] = warehouseObjects.bowl # specify a bowl in the bay in shelf 0 
# with x,y coords (3,1) (zero-indexed). Items are {bowl,mug,bottle,soccer,rubiks,cereal}.

# SET ROBOT PARAMETERS
robotParameters = RobotParameters()
robotParameters.driveType = 'differential'	# specify if using differential (currently omni is not supported)


# MAIN SCRIPT
if __name__ == '__main__':

	# Wrap everything in a try except case that catches KeyboardInterrupts. 
	# In the exception catch code attempt to Stop the Coppelia Simulator so don't have to Stop it manually when pressing CTRL+C

	try:
		packerBotSim = COPPELIA_WarehouseRobot('127.0.0.1', robotParameters, sceneParameters)
		packerBotSim.StartSimulator()
		packerBotSim.SetCameraPose(0.1, 0.1, 0)

		# function to get image in simulator
		def get_image():
			_, img = packerBotSim.GetCameraImage()

			
			img = np.reshape((np.array(img).astype(np.uint8)), (480,640,3))
			img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
			img = cv2.resize(img, (SCREEN_WIDTH, SCREEN_HEIGHT))
			return img

		# we need to tell RobotStateMachine how to get the image and how to set target velocities
		RobotStateMachine.init(STATE.AISLE_DOWN, get_image, lambda fwd, rot: packerBotSim.SetTargetVelocities(fwd, -rot))		
		

		while True:
			RobotStateMachine.update()
			packerBotSim.UpdateObjectPositions() # needs to be called once at the end of the main code loop

	except KeyboardInterrupt as e:
		# attempt to stop simulator so it restarts and don't have to manually press the Stop button in VREP 
		packerBotSim.StopSimulator()
#endregion