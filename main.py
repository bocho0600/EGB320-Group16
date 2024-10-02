import cv2
import numpy as np
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
		
		NavigationModule.init(STATE.FIND_AISLE_FROM_OUTSIDE, instruction) # Temp start = wander for debugging. normally it should be find_aisle_from_outside 
		t1 = time.time()

		pipeline = 'debug'
		draw = True

		while True:
		
			if pipeline == 'debug_distmap':
				# Run vision and most CPU-intensive nav code but don't move

				robotview, visout = VisionModule.Pipeline(False)
				points = VisionModule.combine_contour_points(visout.contours, exclude_horizontal_overlap=False)
				points = VisionModule.handle_outer_contour(points)
				points, projected_floor = VisionModule.project_and_filter_contour(points)
				if points is not None and points.shape[0] > 3:
					dist_map = VisionModule.get_dist_map(points, projected_floor) # dist map column 0 is dist, column 1 is real point
					safety_map = NavigationModule.expand_safety(dist_map)

					if draw:
						robotview = cv2.polylines(robotview, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - dist_map[:,0]/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 0), 1) # draw
						robotview = cv2.polylines(robotview, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw
				else:
					print("No shelf points found")
				
				
				if draw:
					VisionModule.ExportImage("RobotView", robotview, FPS = True)
				else:
					t2 = time.time()
					print(f"FPS: {1.0/(t2-t1):.1f}")
					t1 = t2
			elif pipeline == 'debug':
				# Run vision and most CPU-intensive nav code but don't move
				robotview = VisionModule.DebugPipeline(draw)

				if draw:
					VisionModule.ExportImage("RobotView", robotview, FPS = True)
				else:
					t2 = time.time()
					print(f"FPS: {1.0/(t2-t1):.1f}")
					t1 = t2
			elif pipeline == 'nav':
				# Full navigation move to the desired shelf
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
