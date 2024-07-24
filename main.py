import cv2
#import numpy as np
#import time
from G16Modules.Globals import *
from G16Modules.Vision import VisionModule
#from G16Modules.Navigation import NavigationModule, STATE
#import RP2040 as I2C
import csvread as csvread


def main(): # Main function
	#i2c = I2C.I2C()
	Specific.start(grabber = True)
	
	try:
		while True:
			img, imgHSV, robotview = Specific.get_image()
			#robotview = img
			
			DebugDraw = True
			
			contoursShelf, ShelfMask = VisionModule.findShelf(imgHSV)
			ShelfCenter = VisionModule.GetContoursShelf(contoursShelf, robotview, (0, 0, 255), "She", Draw = DebugDraw)
			if ShelfCenter != None:
				ShelfAngle = VisionModule.GetBearing(ShelfCenter[1])
				cv2.putText(robotview, f"Angle: {int(ShelfAngle)} cm", (int(ShelfCenter[0]), int(ShelfCenter[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)
			
			# Detect obstacles in the HSV image
			contoursObstacle, ObstacleMask = VisionModule.findObstacle(imgHSV)

			# Get the list of detected obstacles' centers and dimensions
			detected_obstacles = VisionModule.GetContoursObject(contoursObstacle, robotview, (0, 255, 255), "Obs", Draw=DebugDraw)

			# Check if any obstacles were detected
			if detected_obstacles is not None:
			# Loop through each detected obstacle and process it
				for obstacle in detected_obstacles:
					x_ObstacleCenter, y_ObstacleCenter, ObHeight, ObWidth = obstacle
					
					# Calculate the obstacle's angle and distance
					ObstacleAngle = VisionModule.GetBearing(x_ObstacleCenter)
					ObstacleDistance = VisionModule.GetDistance(ObHeight, 150)

					# Add the angle and distance information to the image
					cv2.putText(robotview, f"A: {int(ObstacleAngle)} deg", (int(x_ObstacleCenter), int(y_ObstacleCenter + ObHeight / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
					cv2.putText(robotview, f"D: {int(ObstacleDistance)} cm", (int(x_ObstacleCenter), int(y_ObstacleCenter)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)

			
			# # Assuming contoursMarkers is a list of contours found using cv2.findContours
			# contoursMarkers, MarkerMask = VisionModule.findMarkers(imgHSV)

			# # Get the list of detected markers' center and dimensions
			# detected_markers = VisionModule.GetContoursObject(contoursMarkers, robotview, (0, 255, 255), "Circ", Draw=DebugDraw)

			# if detected_markers is not None:
			# 	for marker in detected_markers:
			# 		x_MarkerCenter, y_MarkerCenter, MaHeight, MaWidth = marker
			# 		MarkerAngle = VisionModule.GetBearing(x_MarkerCenter)
			# 		MarkerDistance = VisionModule.GetDistance(MaHeight, 70)
			# 		cv2.putText(robotview, f"A: {int(MarkerAngle)} deg", (int(x_MarkerCenter), int(y_MarkerCenter + MaHeight / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
			# 		cv2.putText(robotview, f"D: {int(MarkerDistance)} cm", (int(x_MarkerCenter), int(y_MarkerCenter)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100),1)
			# 		# You can now process each marker as needed


			VisionModule.ExportImage("RobotView", robotview, FPS = True)

			if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
				break

	finally:
		Specific.end()

if __name__ == "__main__": # Run the main function
	CSV = csvread.CSVReader('Order_1.csv')
	CSV.read_csv()
	instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions

	main()
