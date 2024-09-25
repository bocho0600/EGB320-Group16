import cv2
#import numpy as np
import time
from G16Modules.Globals import *
from G16Modules.Vision import VisionModule
from G16Modules.Navigation import NavigationModule, STATE
#import RP2040 as I2C
import VisionSub.csvread as csvread


def main(): # Main function
	#i2c = I2C.I2C()

	# Possibly override color ranges for simulator
	if hasattr(Specific, 'color_ranges'):
		VisionModule.color_ranges = Specific.color_ranges


	try:
		
		Specific.start()
		instruction = instructions[3] # 1 2 4
		
		NavigationModule.init(STATE.WANDER, instruction)
		t1 = time.time()

		while True:
			
			pipeline = 'nav'
			draw = True

			if pipeline == 'vision':
				robotview = VisionModule.DebugPipeline(True)
				VisionModule.ExportImage("RobotView", robotview, FPS = True)
			else:
				
				robotview, visout = VisionModule.Pipeline(False)
				# print(marker_distance, marker_bearing)
				
				if draw:
					robotview = NavigationModule.update(robotview, visout)
					VisionModule.ExportImage("RobotView", robotview, FPS = True)
				else:
					robotview = NavigationModule.update(None, visout)

					t2 = time.time()
					print(f"FPS: {1.0/(t2-t1):.1f}")
					t1 = t2


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
