import cv2
#import numpy as np
#import time
from G16Modules.Globals import *
from G16Modules.Vision import VisionModule
from G16Modules.Navigation import NavigationModule, STATE
#import RP2040 as I2C
import csvread as csvread


def main(): # Main function
	#i2c = I2C.I2C()

	# Possibly override color ranges for simulator
	if hasattr(Specific, 'color_ranges'):
		VisionModule.color_ranges = Specific.color_ranges
	if hasattr(Specific, 'focal_length'):
		VisionModule.focal_length = Specific.focal_length


	try:
		
		Specific.start()
		NavigationModule.init(STATE.AISLE_DOWN, instruction[2]) # instruction 2 is aisle 2 bay 2

		while True:
			
			robotview, aisle, marker_distance, marker_bearing, points, projected_floor, dist_map = VisionModule.Pipeline()
			# print(marker_distance, marker_bearing)
			robotview = NavigationModule.update(robotview, aisle, marker_distance, marker_bearing, points, projected_floor, dist_map)

			VisionModule.ExportImage("RobotView", robotview, FPS = True)
			Specific.update()

			if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
				break

	finally:
		Specific.end()

if __name__ == "__main__": # Run the main function
	CSV = csvread.CSVReader('Order_1.csv')
	CSV.read_csv()
	instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions

	main()
