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
	
	@classmethod
	def findShelf(cls, imgHSV, area_threshold=10000):
		# Create a mask for the blue color range
		ShelfMask = cv2.inRange(imgHSV, cls.color_ranges['blue'][0], cls.color_ranges['blue'][1])
		
		# Find contours on the mask
		contoursShelf, _ = cv2.findContours(ShelfMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
		
		# Filter out small contours by area
		filtered_contours = [cnt for cnt in contoursShelf if cv2.contourArea(cnt) > area_threshold]
		
		return filtered_contours, ShelfMask
	
	def MarkersDetection(cls, contoursMarkers, output, colour, Draw=True):
		shape_count = 0  # Initialize the shape counter
		detected_shapes = []  # List to store detected shape information
		shape = None
		for contour in contoursMarkers:
			if cv2.contourArea(contour)>300 and cv2.contourArea(contour) < 1000:  # Skip small contours
				epsilon = 0.04 * cv2.arcLength(contour, True)  # Calculate the perimeter of the contour
				# Approximate the contour
				ShapeContours = cv2.approxPolyDP(contour, epsilon, True)
				# Determine the number of vertices
				num_vertices = len(ShapeContours)
				# Identify the shape based on the number of vertices
				if num_vertices == 4:
					shape = "Square"
				elif num_vertices > 4 and num_vertices < 8:
					shape = "Circle"  # Assuming more than 4 sides is a circle for simplicity
				else:
					shape = "Unknown"

				# Only recognize squares and circles
				if shape in ["Square", "Circle"]:
					shape_count += 1  # Increment the counter
					detected_shapes.append((ShapeContours, shape))  # Store shape details
					if Draw:
						cv2.drawContours(output, [ShapeContours], -1, colour, 3)
						x, y = ShapeContours.ravel()[0], ShapeContours.ravel()[1]
						cv2.putText(output, shape, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)

		# Return all detected shapes and the shape count
		return detected_shapes, shape_count, shape