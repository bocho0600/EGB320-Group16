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


	try:
		
		Specific.start()
		instruction = instructions[1]
		
		NavigationModule.init(STATE.FIND_AISLE_FROM_OUTSIDE, instruction) # instruction 2 is aisle 2 bay 2

		while True:
			
			pipeline = 'nav'

			if pipeline == 'vision':
				robotview = VisionModule.DebugPipeline(True)
				VisionModule.ExportImage("RobotView", robotview, FPS = True)
			else:
				
				robotview, visout = VisionModule.Pipeline()
				# print(marker_distance, marker_bearing)
				robotview = NavigationModule.update(robotview, visout)

				VisionModule.ExportImage("RobotView", robotview, FPS = True)

			Specific.update()

			if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
				break

	finally:
		Specific.end()

if __name__ == "__main__": # Run the main function
	CSV = csvread.CSVReader('Order_1.csv')
	CSV.read_csv()
	instructions = CSV.RobotInstruction() # Generating robot instructions and print instructions

	main()
