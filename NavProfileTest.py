import cv2
#import numpy as np
import time
from G16Modules.Globals import *
from G16Modules.Vision import VisionModule
from G16Modules.Navigation import NavigationModule, STATE
#import RP2040 as I2C
import VisionSub.csvread as csvread


if __name__ == "__main__": # Run the main function
	CSV = csvread.CSVReader('Order_1.csv')
	CSV.read_csv()
	instructions = CSV.RobotInstruction() # Generating robot instructions and print instructions

	#i2c = I2C.I2C()

	# Possibly override color ranges for simulator
	if hasattr(Specific, 'color_ranges'):
		VisionModule.color_ranges = Specific.color_ranges


	try:
		
		Specific.start()
		instruction = instructions[3] # 1 2 4
		
		NavigationModule.init(STATE.WANDER, instruction)

		
		robotview, visout = VisionModule.Pipeline(False)
		
		points = VisionModule.combine_contour_points(visout.contours, False)
		points = VisionModule.handle_outer_contour(points)
		points, projected_floor = VisionModule.project_and_filter_contour(points)
		if points is not None and points.shape[0] > 3:
	
			dist_map = VisionModule.get_dist_map(points, projected_floor) # dist map column 0 is dist, column 1 is real point
			safety_map = NavigationModule.expand_safety(dist_map)

		Specific.update()


	finally:
		Specific.end()
