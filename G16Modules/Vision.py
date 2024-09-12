import cv2
import numpy as np
from math import cos,sin
import time
from threading import Thread, Lock, Condition

from .Globals import *
#from threading import Thread
# MADE BY KELVIN LE, QUT EGB320 GROUP16 
# StudentID: n11429984
# Email: minhnguyen.le@qut.edu.au

class VisionModule:
	color_ranges = {
		'wall': (np.array([39, 0, 0]), np.array([162, 255, 255])),
		'floor': (np.array([0, 0, 0]), np.array([179, 255, 255])),
		'yellow': (np.array([25, 90, 0]), np.array([36, 233, 255])),
		'blue': (np.array([81, 0, 0]), np.array([116, 255, 255])),
		'green': (np.array([55, 79, 0]), np.array([70, 255, 255])),
		'orange1': (np.array([0, 100, 0]), np.array([20, 255, 255])),
		'orange2': (np.array([165, 100, 0]), np.array([180, 255, 255])),
		'black': (np.array([29,53, 0]), np.array([51, 103, 81]))
	}



	
	#region Contour Processing 
	# ANGLE_H = packerBotSim.horizontalViewAngle
	# ANGLE_V = packerBotSim.verticalViewAngle

	DIST_X = 0 #packerBotSim.robotParameters.cameraDistanceFromRobotCenter
	DIST_Z = 0.0752 #packerBotSim.robotParameters.cameraHeightFromFloor
	# tilt = packerBotSim.robotParameters.cameraTilt
	TILT = 1.5 * 3.1415926535 / 180
	# tilt is meant to be 0 but it is slightly off in simulator

	# Precalculate Transformation Matrices and ground plane
	camera_to_robot_rotate = np.array([
			[cos(TILT), 0, -sin(TILT), 0],
			[        0, 1,          0, 0],
			[sin(TILT), 0,  cos(TILT), 0],
			[        0, 0,          0, 1]])
	camera_to_robot_translate = np.array([
			[1, 0, 0, DIST_X],
			[0, 1, 0,      0],
			[0, 0, 1, DIST_Z],
			[0, 0, 0,      1]])
	
	camera_to_robot = np.matmul(camera_to_robot_translate,camera_to_robot_rotate)
	robot_to_camera = np.linalg.inv(camera_to_robot)
	#robot_to_camera_translate = np.linalg.inv(camera_to_robot_translate)
	robot_to_camera_rotate = np.linalg.inv(camera_to_robot_rotate)

	# Normal and point of the ground plane, relative to camera
	normal_camera = np.matmul(robot_to_camera_rotate , np.array([[0,0,1,1]]).T)[0:3, 0]
	r_camera = np.matmul(robot_to_camera , np.array([[0,0,0,1]]).T)[0:3, 0]


	@classmethod
	def project_point_to_ground(cls,screen_coords):
		x = screen_coords[:, 0]
		y = screen_coords[:, 1]

		# Coordinates on a plane in front of the camera, relative to camera
		cx = -(x-SCREEN_WIDTH/2) / SCREEN_WIDTH
		cy = -(y-SCREEN_HEIGHT/2) / SCREEN_WIDTH
		cz = 1 / FOV_HORIZONTAL * np.ones(cx.shape)
		cpi = np.array([cz,-cx,cy])


		# coordinates on the ground, relative to camera
		cpo = np.dot(cls.normal_camera, cls.r_camera) / np.dot(cls.normal_camera, cpi) * cpi
		cpo = np.array([cpo[0,:], cpo[1,:], cpo[2,:], np.ones(cpo[0,:].shape)])

		# coodinates on the ground, relative to robot
		rpo = np.matmul(cls.camera_to_robot, cpo)
		return rpo[0:2,:].T


	@staticmethod
	def combine_contour_points(contours, exclude_horizontal_overlap=True):

		
		combined_contour = None
		xmax = -1
		xmin = SCREEN_WIDTH+1
		for cont in contours:

			
			if exclude_horizontal_overlap:
				bounds = cv2.boundingRect(cont) # left, top, width, height

				# only consider new contours strictly outside of what we have already seen
				# * this means that a contour between two already seen will be excluded! this should be pretty rare though *
				if bounds[0]+bounds[2] <= xmin:
					xmin = bounds[0]
					xmax = max(xmax, bounds[0]+bounds[2])
				elif bounds[0] >= xmax:
					xmax = bounds[0] + bounds[2]
					xmin = min(xmin, bounds[0])
				else:
					continue
			

			# npc is array of contour points along the edge (and around the outside) of the object
			npc = np.array(cont)
			npc = npc[:,0,:]

			
			
			# discard points on the edge of the screen and sort left to right (if two points are at same x prefer larger y (lower on the screen))
			# this is so later when we get unique elements the lower one is kept
			npc = npc[(npc[:,1] > 0) & (npc[:,1] < SCREEN_HEIGHT-1) & (npc[:,0] > 0) & (npc[:,0] < SCREEN_WIDTH-1), :]
			npc = npc[np.argsort(npc[:, 0]), :]

			if npc.size < 3:
				continue

			# this is too annoying to do in the case of multiple contours
			# add a 0 point before and after so that the dist map counts blank spaces correctly
			# if npc[0,0] > 1: # any with 0 were removed so first being =1 is acceptable
			# 	npc = np.r_[[[npc[0,0]-1, SCREEN_HEIGHT-1]], npc]
			# if npc[-1,0] < SCREEN_WIDTH-2:# any with SCREEN_WIDTH-1 were removed so first being =SCREEN_WIDTH-2 is acceptable
			# 	npc = np.r_[npc, [[npc[-1,0]+1, SCREEN_HEIGHT-1]]]	

			
			# # append to comvined_contour
			if combined_contour is None:
				combined_contour = npc
			else:
				combined_contour = np.r_[combined_contour, npc]
		return combined_contour

	@classmethod
	def project_and_filter_contour(cls,contour_points):
		if contour_points is not None:
			# make sure it is sorted from left to right and only keep one point per column. (lower points preferred)	
			contour_points = contour_points[np.unique(contour_points[:, 0]+ 1.0-contour_points[:, 1]/SCREEN_HEIGHT, return_index=True)[1]]


			# project contour onto the ground
			projection = cls.project_point_to_ground(contour_points)

			# discard points above the horizon
			mask = projection[:, 0] >= 0
			projection = projection[mask]
			contour_points = contour_points[mask]
			return contour_points, projection
		else:
			return None, None

	@staticmethod
	def get_dist_map(contour_points, projection):
		# distances of each point. However each point does not match 1 to 1 with pixels
		dist_real = np.sqrt(projection[:,0]**2 + projection[:,1]**2)
		
		# find which point matches to which pixel considering duplicates and skips
		dist_map = np.zeros(SCREEN_WIDTH, np.float32)
		j = 0
		for i in range(len(dist_map)):
			while j < len(contour_points)-1 and contour_points[j, 0] < i:
				j += 1
			dist_map[i] = dist_real[j]
		return dist_map
	#endregion

	focal_length = 70 #cm
	real_circle_diameter = 70 #cm

 
	t1 = time.time()
	fps = 0


	@classmethod
	def Capturing(cls):

		img, imgHSV = Specific.get_image()  # Capture a single image frame

		t2 = time.time()
		cls.fps = 1.0 / (t2-cls.t1)
		cls.t1 = t2

		return img, imgHSV

	@classmethod
	def ExportImage(cls, WindowName, view, FPS=False):
		if FPS:
			cv2.putText(view, f'{int(cls.fps)}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 2)  # Display the FPS on the screen
		
		cv2.imshow(WindowName, view)

	@classmethod
	def findBlack(cls, imgHSV):
		# Create masks for the orange color
		#HSV90 = cv2.rotate(imgHSV, cv2.ROTATE_180)
		BlackMask = cv2.inRange(imgHSV, cls.color_ranges['black'][0], cls.color_ranges['black'][1])
		#contoursBlack, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return BlackMask

	@classmethod
	def findItems(cls, imgHSV):
		# Create masks for the orange color
		#HSV90 = cv2.rotate(imgHSV, cv2.ROTATE_180)
		ItemMask1 = cv2.inRange(imgHSV, cls.color_ranges['orange1'][0], cls.color_ranges['orange1'][1])
		ItemMask2 = cv2.inRange(imgHSV, cls.color_ranges['orange2'][0], cls.color_ranges['orange2'][1])
		ItemMask = ItemMask1 | ItemMask2  # Combine masks
		contoursItem, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return contoursItem, ItemMask

	@classmethod
	def findShelf(cls, imgHSV, area_threshold=10000, chain = cv2.CHAIN_APPROX_SIMPLE):
		# Create a mask for the blue color range
		ShelfMask = cv2.inRange(imgHSV, cls.color_ranges['blue'][0], cls.color_ranges['blue'][1])
		
		# Find contours on the mask
		contoursShelf, _ = cv2.findContours(ShelfMask, cv2.RETR_EXTERNAL, chain)
		
		# Filter out small contours by area
		filtered_contours = [cnt for cnt in contoursShelf if cv2.contourArea(cnt) > area_threshold]
		
		return contoursShelf, ShelfMask

	@classmethod
	def findLoadingArea(cls, imgHSV):
		LoadingAreaMask = cv2.inRange(imgHSV, cls.color_ranges['yellow'][0], cls.color_ranges['yellow'][1])
		contoursLoadingArea, _ = cv2.findContours(LoadingAreaMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return contoursLoadingArea, LoadingAreaMask

	@classmethod
	def findObstacle(cls, imgHSV, chain = cv2.CHAIN_APPROX_SIMPLE):
		ObstacleMask = cv2.inRange(imgHSV, cls.color_ranges['green'][0], cls.color_ranges['green'][1])
		contoursObstacle, _ = cv2.findContours(ObstacleMask, cv2.RETR_EXTERNAL, chain)
		return contoursObstacle, ObstacleMask
	
	@classmethod
	def findWall(cls, imgGray, imgRGB):
		_, WhiteMask = cv2.threshold(imgGray, 165, 255, cv2.THRESH_BINARY) # Apply thresholding to get white colour filter
		WallMask = np.zeros_like(imgGray) # Create an empty mask for the wall
		contours, _ = cv2.findContours(WhiteMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for contour in contours:
			if cv2.contourArea(contour) > 300 :
				cv2.drawContours(WallMask, [contour], -1, 255, thickness=cv2.FILLED)
		# Extract the wall region (WILL NEED TO BE ADJUSTED)
		WallImage = cv2.bitwise_and(imgRGB, imgRGB, mask=WallMask)
		WallImage = cv2.cvtColor(WallImage, cv2.COLOR_BGR2GRAY)
		_, MarkerMask = cv2.threshold(imgGray, 120, 255, cv2.THRESH_BINARY_INV)
		# Find contours in the MarkerMask
		contoursMarkers, _ = cv2.findContours(WallImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		return WallMask, contoursMarkers, WallImage

	@classmethod
	def findMarkers(cls,imgHSV):
		BlackMask = cv2.inRange(imgHSV, cls.color_ranges['black'][0], cls.color_ranges['black'][1])
		contoursMarkers, MarkerMask = cv2.findContours(BlackMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		return contoursMarkers, MarkerMask




	@classmethod
	def MarkerShapeDetection(cls, contoursMarkers, output, Draw = True):
		detected = False
		shapeCount = 0
		distances = []  # List to store distances for all detected circles
		bearings = []
		xs = []
		ys = []
		for contour in contoursMarkers:
			if cv2.contourArea(contour) > 300 and cv2.contourArea(contour) < 50000:
				epsilon = 0.03 * cv2.arcLength(contour, True)
				ShapeContours = cv2.approxPolyDP(contour, epsilon, True)
				num_vertices = len(ShapeContours)
				if num_vertices == 4:
					shape = "Square"
				elif num_vertices in [4, 12]:
					shape = "Circle"
					shapeCount += 1
					(x_center, y_center), radius = cv2.minEnclosingCircle(contour)
					diameter = 2 * radius
					distance = cls.GetDistance(diameter, 70)
					bearing = cls.GetBearing(x_center)

					distances.append(distance)  # Store each distance
					bearings.append(bearing)
					xs.append(x_center)
					ys.append(y_center)
					
					if Draw:
						cv2.putText(output, f"Distance: {distance} cm", (x_center, y_center), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (123, 200, 100), 2)
						cv2.putText(output, f"Circle: {shapeCount}", (int(x_center), int(y_center)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)
						cv2.circle(output, (int(x_center), int(y_center)), int(radius), (0, 255, 0), 2)
					detected = True

		if detected:
			return shapeCount, distances, bearings, xs, ys  # Return list of distances
		else:
			return None, [], [], [], []  # Return empty list if no circles detected

	@classmethod
	def ProcessAisleMarkers(cls, shapeCount, distances, bearings, xs, ys):
		# return aisle, distance, bearing
		# detected_shapes if a list of tuples (ShapeContours, shape), shape in ["Square", "Circle"]
		if shapeCount == 0:
			return 0, None, None, None, None
				
		distance = np.array(distances).mean()
		bearing = np.array(bearings).mean()
		x_center = np.array(xs).mean()
		y_center = np.array(ys).mean()

		return shapeCount, distance, bearing, x_center, y_center
  

	@classmethod
	def GetContoursShelf(cls, contours, output, colour, text, Draw=True):
		detected = False
		center = (0, 0)
		radius = 0
		for contour in contours:
			if cv2.contourArea(contour) > 1000:
				# Calculate the contour's center using moments
				M = cv2.moments(contour)
				if M['m00'] != 0:  # Avoid division by zero
					center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
				else:
					center = (0, 0)
				
				if Draw:
					# Draw the contour
					cv2.drawContours(output, [contour], -1, colour, 2)
					
					# Draw the text at the center of the contour
					cv2.putText(output, text, center, 
								cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
					
				detected = True
				# If you want to calculate a bounding box
				x, y, w, h = cv2.boundingRect(contour)
				x1, y1, x2, y2 = x, y, x + w, y + h
			else:
				x1, y1, x2, y2 = None, None, None, None

		if detected:
			return x1, y1, x2, y2
		else:
			return None, None, None, None
		
	@classmethod
	def GetContoursObject(cls, contours, output, colour, text, Draw = True):
		detected = False
		for contour in contours:
			if cv2.contourArea(contour) > 500:
				x, y, width, height = cv2.boundingRect(contour)
				if Draw:
					cv2.rectangle(output, (x, y), (x + width, y + height), colour, 2)
					cv2.putText(output, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				detected = True
		if detected:
			x_center, y_center = x + width // 2, y + height // 2
			return x_center, y_center
		else:
			return  None, None
	
	@classmethod
	def GetDistance(cls, width, real_width):
		return (cls.focal_length * real_width) / width + 4
	
	@classmethod
	def GetBearing(cls, x_center):
		offset_pixels = x_center - SCREEN_WIDTH/ 2
		return (offset_pixels / SCREEN_WIDTH) * 22.3

