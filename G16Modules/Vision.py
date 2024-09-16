import cv2
import numpy as np
from math import cos,sin, pi
import time
from threading import Thread, Lock, Condition

from .Globals import *
#from threading import Thread
# MADE BY KELVIN LE, QUT EGB320 GROUP16 
# StudentID: n11429984
# Email: minhnguyen.le@qut.edu.au

class VisionOutput:
	# Basically a class where we can put whatever data we want
	def __init__(self, **kwargs):
		for key, value in kwargs.items():
			self.__setattr__(key, value)

class VisionModule:
	color_ranges = {
		'wall': (np.array([34, 0, 211]), np.array([45, 74, 255])),
		'floor': (np.array([0, 0, 0]), np.array([179, 255, 255])),
		'yellow': (np.array([29, 177, 244]), np.array([30, 251, 255])),
		'blue': (np.array([81, 0, 0]), np.array([116, 255, 255])),
		'green': (np.array([55, 79, 0]), np.array([70, 255, 255])),
		'orange1': (np.array([0, 100, 0]), np.array([20, 255, 255])),
		'orange2': (np.array([165, 100, 0]), np.array([180, 255, 255])),
		'black': (np.array([38,31, 45]), np.array([66, 121 , 88]))
	}


	
	#region Contour Processing 
	# DIST_X, DIST_Z and TILT are defined in Globals.py

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

			npc = np.c_[npc, np.ones((npc.shape[0], 1), np.int32)] # new column : is real observed point
			
			# discard points on the edge of the screen and sort left to right (if two points are at same x prefer larger y (lower on the screen))
			# this is so later when we get unique elements the lower one is kept
			npc = npc[(npc[:,0] > 0) & (npc[:,0] < SCREEN_WIDTH-1), :]
			#npc = npc[(npc[:,1] > 0) & (npc[:,1] < SCREEN_HEIGHT-1) & (npc[:,0] > 0) & (npc[:,0] < SCREEN_WIDTH-1), :]
			npc = npc[np.argsort(npc[:, 0]), :]

			if npc.shape[0] < 3:
				continue

			# # append to comvined_contour
			if combined_contour is None:
				combined_contour = npc
			else:
				combined_contour = np.r_[combined_contour, npc]

		if combined_contour is not None:
			combined_contour = combined_contour[np.argsort(combined_contour[:,0])]
		return combined_contour

	@classmethod
	def handle_outer_contour(cls, combined_contour):
		if combined_contour is not None:
			# add a 0 point before and after so that the dist map counts blank spaces as close walls and will turn away from it and towards a shelf.
			# This will cause problems when trying to leave an aisle so don't call it in that case.
			if combined_contour[0,0] > 1: # any with 0 were removed so first being =1 is acceptable
				combined_contour = np.r_[[[combined_contour[0,0]-1, SCREEN_HEIGHT-1, 0]], combined_contour]
			if combined_contour[-1,0] < SCREEN_WIDTH-2:# any with SCREEN_WIDTH-1 were removed so first being =SCREEN_WIDTH-2 is acceptable
				combined_contour = np.r_[combined_contour, [[combined_contour[-1,0]+1, SCREEN_HEIGHT-1, 0]]]
		return combined_contour


	@classmethod
	def project_and_filter_contour(cls,contour_points):
		if contour_points is not None:
			# make sure it is sorted from left to right and only keep one point per column. (lower points preferred)	
			contour_points = contour_points[np.unique(contour_points[:, 0]+ 1.0-contour_points[:, 1]/SCREEN_HEIGHT, return_index=True)[1], :]

			# project contour onto the ground
			projection = cls.project_point_to_ground(contour_points)

			# discard points above the horizon
			mask = projection[:, 0] >= 0
			projection = projection[mask]
			contour_points = contour_points[mask, :]
			return contour_points, projection
		else:
			return None, None

	@staticmethod
	def get_dist_map(contour_points, projection):
		# distances of each point. However each point does not match 1 to 1 with pixels
		dist_real = np.sqrt(projection[:,0]**2 + projection[:,1]**2)
		
		# find which point matches to which pixel considering duplicates and skips
		dist_map = np.zeros(SCREEN_WIDTH, np.float32)
		real_points = np.zeros(SCREEN_WIDTH, np.float32)
		j = 0 # jth point
		for i in range(len(dist_map)): # ith pixel
			while j < len(contour_points)-1 and contour_points[j, 0] < i:
				j += 1

			# if we are not on the first point we can consider the previous point
			# and if out current point is past the current pixel we should look at the previous point
			if j > 0 and contour_points[j, 0] > i: 
				# we would take nearest, minimum, interpolate etc
				# take the one with max distance
				if dist_real[j-1] > dist_real[j]:
					dist_map[i] = dist_real[j-1]
					real_points[i] = contour_points[j-1, 2]
				else:
					dist_map[i] = dist_real[j]
					real_points[i] = contour_points[j, 2]
			else:
				dist_map[i] = dist_real[j]
				real_points[i] = contour_points[j, 2]
		return np.c_[dist_map, real_points]
	#endregion

	focal_length = 35 #mm
	real_circle_diameter = 70 #mm

 
	t1 = time.time()
	fps = 0


	@classmethod
	def DebugPipeline(cls, DebugDraw):
		img, imgHSV, robotview = Specific.get_image()
		#robotview = img
		
		DebugDraw = True
		
		contoursShelf, ShelfMask = VisionModule.findShelf(imgHSV)
		ShelfCenter = VisionModule.GetContoursShelf(contoursShelf, robotview, (0, 0, 255), "She", Draw = DebugDraw)
		if ShelfCenter != None:
			ShelfAngle = VisionModule.GetBearing(ShelfCenter[1]) * 180 / pi
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
				ObstacleAngle = VisionModule.GetBearing(x_ObstacleCenter) * 180 / pi
				ObstacleDistance = VisionModule.GetDistance(ObHeight, 150)

				# Add the angle and distance information to the image
				cv2.putText(robotview, f"A: {int(ObstacleAngle)} deg", (int(x_ObstacleCenter), int(y_ObstacleCenter + ObHeight / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
				cv2.putText(robotview, f"D: {int(ObstacleDistance)} cm", (int(x_ObstacleCenter), int(y_ObstacleCenter)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)

		
		# Assuming contoursMarkers is a list of contours found using cv2.findContours
		contoursMarkers, MarkerMask = VisionModule.findMarkers(imgHSV)

		# Get the list of detected markers' center and dimensions
		detected_markers = VisionModule.GetContoursObject(contoursMarkers, robotview, (0, 255, 255), "Circ", Draw=DebugDraw)

		if detected_markers is not None:
			for marker in detected_markers:
				x_MarkerCenter, y_MarkerCenter, MaHeight, MaWidth = marker
				MarkerAngle = VisionModule.GetBearing(x_MarkerCenter) * 180 / pi
				MarkerDistance = VisionModule.GetDistance(MaHeight, 70)
				cv2.putText(robotview, f"A: {int(MarkerAngle)} deg", (int(x_MarkerCenter), int(y_MarkerCenter + MaHeight / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
				cv2.putText(robotview, f"D: {int(MarkerDistance)} cm", (int(x_MarkerCenter), int(y_MarkerCenter)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100),1)
				# You can now process each marker as needed
		return robotview

	@classmethod
	def Pipeline(cls, draw=True):
		img, imgHSV, robotview = Specific.get_image()

		
		contoursShelf, ShelfMask = cls.findShelf(imgHSV, 200, cv2.CHAIN_APPROX_NONE)

		# Get the list of detected shelves' centers and dimensions
		detected_shelves = cls.GetContoursObject(contoursShelf, robotview, (0, 255, 255), "She", Draw=draw)


		# Detect obstacles in the HSV image
		contoursObstacle, ObstacleMask = cls.findObstacle(imgHSV, cv2.CHAIN_APPROX_NONE)
		
		WallRGB,  WallImgGray, WallMask = cls.findWall(imgHSV,img)
		ContoursMarkers, mask1 = cls.findMarkers(WallImgGray, WallMask)
		avg_center, marker_bearing, marker_distance, aisle = cls.GetInfoMarkers(robotview, ContoursMarkers, img, draw=draw)

		if draw and aisle is not None and aisle != 0:
			cv2.drawMarker(robotview, (int(avg_center[0]), int(avg_center[1])), (255, 255, 0), cv2.MARKER_SQUARE, 12, 3)
			cv2.putText(robotview, f"{marker_distance:.1f} cm", (int(avg_center[0]), int(avg_center[1])-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)


		contours = [c for c in contoursShelf if cv2.contourArea(c) > 100] + [c for c in contoursObstacle if cv2.contourArea(c) > 100]

		# sort the remaining ones by their area
		contours = sorted(contours, key=lambda cont: -cv2.contourArea(cont))
		
		
		
		return robotview, VisionOutput(aisle=aisle, marker_distance=marker_distance, marker_bearing=marker_bearing, contours=contours, detected_shelves=detected_shelves)


	@classmethod
	def ExportImage(cls, WindowName, view, FPS=False):
		if FPS:
			fps = 1.0 / (time.time() - cls.t1)  # calculate frame rate
			cls.t1 = time.time()
			cv2.putText(view, f'{int(fps)}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 2)  # Display the FPS on the screen
		
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
		ItemMask1 = cv2.inRange(imgHSV, cls.color_ranges['orange1'][0], cls.color_ranges['orange1'][1])
		ItemMask2 = cv2.inRange(imgHSV, cls.color_ranges['orange2'][0], cls.color_ranges['orange2'][1])
		ItemMask = ItemMask1 | ItemMask2  # Combine masks
		contoursItem, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return contoursItem, ItemMask

	@classmethod
	def findShelf(cls, imgHSV, area_threshold=10000, chain=cv2.CHAIN_APPROX_SIMPLE):
		# Create a mask for the blue color range
		ShelfMask = cv2.inRange(imgHSV, cls.color_ranges['blue'][0], cls.color_ranges['blue'][1])
		
		# Find contours on the mask
		contoursShelf, _ = cv2.findContours(ShelfMask, cv2.RETR_EXTERNAL,chain)
		
		# Filter out small contours by area
		#filtered_contours = [cnt for cnt in contoursShelf if cv2.contourArea(cnt) > area_threshold]
		
		return contoursShelf, ShelfMask

	@classmethod
	def findLoadingArea(cls, imgHSV):
		LoadingAreaMask = cv2.inRange(imgHSV, cls.color_ranges['yellow'][0], cls.color_ranges['yellow'][1])
		contoursLoadingArea, _ = cv2.findContours(LoadingAreaMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return contoursLoadingArea, LoadingAreaMask

	@classmethod
	def findObstacle(cls, imgHSV, chain=cv2.CHAIN_APPROX_SIMPLE):
		ObstacleMask = cv2.inRange(imgHSV, cls.color_ranges['green'][0], cls.color_ranges['green'][1])
		contoursObstacle, _ = cv2.findContours(ObstacleMask, cv2.RETR_EXTERNAL, chain)
		return contoursObstacle, ObstacleMask
	
	@classmethod
	def findWall(cls, imgHSV, imgRGB):
		# Create masks for the orange color
		WallMask = cv2.inRange(imgHSV, cls.color_ranges['wall'][0], cls.color_ranges['wall'][1])
		
		# Find contours in the mask
		contoursWall1, _ = cv2.findContours(WallMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		
		# Check if any contours are found
		if contoursWall1:
			# Find the largest contour
			largest_contour = max(contoursWall1, key=cv2.contourArea)
			
			# Create an empty mask and draw the largest contour on it
			filledWallMask = np.zeros_like(WallMask)
			cv2.drawContours(filledWallMask, [largest_contour], -1, (255), thickness=cv2.FILLED)
			
			# Apply Gaussian blur to the filled mask
			filledWallMask = cv2.GaussianBlur(filledWallMask, (9, 9), 2)
			
			# Use the filled mask to extract the wall image
			WallImg = cv2.bitwise_and(imgRGB, imgRGB, mask=filledWallMask)
			
			# Convert the extracted image to grayscale
			WallImgGray = cv2.cvtColor(WallImg, cv2.COLOR_BGR2GRAY)
		else:
			# No contours found, return original image and empty mask
			WallImg = np.zeros_like(imgRGB)
			WallImgGray = np.zeros_like(cv2.cvtColor(imgRGB, cv2.COLOR_BGR2GRAY))
			filledWallMask = np.zeros_like(WallMask)
		
		return WallImg, WallImgGray, filledWallMask

	@classmethod
	def findMarkers(cls, WallImgGray, WallMask):
		_, mask = cv2.threshold(WallImgGray, 110, 255, cv2.THRESH_BINARY_INV)
		markers = cv2.bitwise_and(WallMask, mask)
		_, mask1 = cv2.threshold(markers, 110, 255, cv2.THRESH_BINARY)
		ContoursMarkers, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		return ContoursMarkers, mask1
	
	@classmethod
	def GetInfoMarkers(cls, robotview, ContoursMarkers, imgRGB, draw=True):
		distance = []
		bearing = []
		centers = []

		for contours in ContoursMarkers:
			if cv2.contourArea(contours) > 100:
				(x, y), radius = cv2.minEnclosingCircle(contours)
				center = (int(x), int(y))
				circle_area = np.pi * (radius ** 2)
				contour_area = cv2.contourArea(contours)
				
				# Define the acceptable difference threshold
				area_difference_threshold = 5000  # You can adjust this value

				# Check if the difference between areas is within the threshold
				if abs(contour_area - circle_area) <= area_difference_threshold:
					MarkerAngle = cls.GetBearing(x)
					MarkerDistance = cls.GetDistance(radius * 2, 70)
					if draw:
						# cv2.putText(robotview, f"A: {int(MarkerAngle)} deg", (int(x), int(y + radius / 2)), 
						#             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
						# cv2.putText(robotview, f"D: {int(MarkerDistance)} cm", (int(x), int(y)), 
						#             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)
						cv2.circle(robotview, center, int(radius), (255, 255), 2)
						cv2.drawMarker(robotview, center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=5, thickness=2)
						cv2.putText(robotview, f"M", (int(x - 6), int(y + radius / 2)), 
									cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))
					
					# Store the distance, bearing, and center
					distance.append(MarkerDistance)
					bearing.append(MarkerAngle)
					centers.append(center)

		# Calculate the average center if there are any centers
		if centers:
			avg_x = sum([c[0] for c in centers]) / len(centers)
			avg_y = sum([c[1] for c in centers]) / len(centers)
			avg_center = (int(avg_x), int(avg_y))
			avg_distance = sum(distance) / len(distance)
			avg_bearing = sum(bearing) / len(bearing)
			shape_count = len(centers)
			cv2.drawMarker(robotview, avg_center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=5, thickness=2)
			cv2.putText(robotview, f"A: {int(avg_bearing)} deg", (int(avg_x), int(avg_y + 20)), 
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
			cv2.putText(robotview, f"D: {int(avg_distance)} cm", (int(avg_x), int(avg_y)),
						cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)
			cv2.putText(robotview, f"{shape_count}", (int(avg_x - 10), int(avg_y)),
						cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
		else:
			avg_center = None  # If no centers were found, return None or a default value
			avg_bearing = None
			avg_distance = None
			shape_count = 0

		return avg_center, avg_bearing, avg_distance, shape_count

	
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
				cv2.putText(output, text, center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				if Draw:
					# Draw the contour
					cv2.drawContours(output, [contour], -1, colour, 1)
					
					# Draw the text at the center of the contour
					cv2.putText(output, text, center, 
								cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
					
				detected = True
				# If you want to calculate a bounding box

		if detected:
			return center
		else:
			return None
		
	@classmethod
	def GetContoursObject(cls, contours, output, colour, text, Draw=True):
		detected_objects = []  # List to store detected object info
		
		for contour in contours:
			if cv2.contourArea(contour) > 50:
				x, y, width, height = cv2.boundingRect(contour)
				
				if Draw:
					cv2.rectangle(output, (x, y), (x + width, y + height), colour, 1)
					cv2.putText(output, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
				
				# Calculate center
				x_center, y_center = x + width // 2, y + height // 2
				
				# Append this object's properties to the list
				detected_objects.append((x_center, y_center, height, width))
		
		# Return list of detected objects
		if detected_objects:
			return detected_objects
		else:
			return None

	
  
	@classmethod
	def GetDistance(cls, width, real_width):
		return (cls.focal_length * real_width) / width + 4
	
	@classmethod
	def GetBearing(cls, x_center): # RADIANS!!!
		offset_pixels = x_center - SCREEN_WIDTH/ 2
		return offset_pixels * FOV_HORIZONTAL / SCREEN_WIDTH

