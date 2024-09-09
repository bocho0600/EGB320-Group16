import numpy as np, cv2
from .Globals import *

class Vision:

	get_image_callback = None
	
	@classmethod
	def get_image(cls):
		if cls.get_image_callback is not None:
			return cls.get_image_callback()
		else:
			print("Warning: set get_image_callback in Vision module")

	color_ranges = {
				'floor': (np.array([0, 0, 80]), np.array([0, 0, 135])),
				'wall': (np.array([0, 0, 146]), np.array([30, 1, 255])),
				'blue': (np.array([3, 171, 54]), np.array([3, 175, 112])),
				'black': (np.array([0, 0, 0]), np.array([0, 0, 0])),
				'yellow': (np.array([99, 216, 130]), np.array([99, 217, 187])),
				'green': (np.array([40, 90, 0]), np.array([70, 255, 180])),
				'orange1': (np.array([5, 150, 150]), np.array([20, 255, 255])),
				'orange2': (np.array([165, 150, 150]), np.array([180, 255, 255])),
		}

	
	@staticmethod
	def findFloor(img):
		imgHSV = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
		FloorMask = cv2.inRange(imgHSV, Vision.color_ranges['floor'][0], Vision.color_ranges['floor'][1])
		FloorMask = cv2.morphologyEx(FloorMask, cv2.MORPH_CLOSE, np.ones((3,3)))
		contoursFloor, _ = cv2.findContours(FloorMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		return contoursFloor, FloorMask