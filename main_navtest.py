import cv2
import numpy as np
import time
#import RP2040 as I2C
import csvread as csvread


from COPPELIA_FILES.PythonCode.G16Modules.Globals import *
from COPPELIA_FILES.PythonCode.G16Modules.Navigation import Navigation, STATE, PHASE
from COPPELIA_FILES.PythonCode.G16Modules.StaticVision import VisionModule
from COPPELIA_FILES.PythonCode.G16Modules.RobotStateMachine import RobotStateMachine

color_ranges = {
		'wall': (np.array([39, 0, 0]), np.array([162, 255, 255])),
		'floor': (np.array([0, 0, 0]), np.array([179, 255, 255])),
		'yellow': (np.array([25, 90, 0]), np.array([36, 233, 255])),
		'blue': (np.array([104, 85, 0]), np.array([123, 255, 255])),
		'green': (np.array([30, 36, 0]), np.array([96, 255, 226])),
		'orange1': (np.array([0, 100, 0]), np.array([20, 255, 255])),
		'orange2': (np.array([165, 100, 0]), np.array([180, 255, 255])),
		'black': (np.array([0, 0, 43]), np.array([179, 55, 109]))
}



def main(): # Main function
		#i2c = I2C.I2C()
		cap = VisionModule.initialize_camera()

		RobotStateMachine.init(lambda:None, lambda fwd, rot: None)	

		while(1):
				img,imgHSV,robotview = VisionModule.Capturing()
				print(img.shape)
				#imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

				#contoursItem, ItemMask = VisionModule.findItems(imgHSV)
				#xi1, yi1, xi2, yi2 = VisionModule.GetContoursObject(contoursItem, robotview, (0, 255, 0), "Item", Draw = True)

				contoursShelf, ShelfMask = VisionModule.findShelf(imgHSV)
				xs1, ys1, xs2, ys2 = VisionModule.GetContoursShelf(contoursShelf, robotview, (0, 0, 255), "Shelf", Draw = True)

				contoursObstacle, ObstacleMask = VisionModule.findObstacle(imgHSV)
				xo1, yo1, xo2, yo2 = VisionModule.GetContoursObject(contoursObstacle, robotview, (0, 255, 255), "Obstacle", Draw = True)

				contoursLoading, LoadingMask = VisionModule.findLoadingArea(imgHSV)
				xl1, yl1, xl2, yl2 = VisionModule.GetContoursShelf(contoursLoading, robotview, (120, 120, 255), "LoadingArea", Draw = True)

				VisionModule.ExportImage("RobotView", robotview, FPS = True)
				#VisionModule.ExportImage("WallMask", BlackMask, FPS = True)

				if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
					break
		cap.close()

if __name__ == "__main__": # Run the main function
		CSV = csvread.CSVReader('Order_1.csv')
		CSV.read_csv()
		instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions

		main()
