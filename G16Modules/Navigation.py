import numpy as np, cv2
from enum import Enum
from math import cos, sin, pi
import time

from .Globals import *
from .Vision import VisionModule
# from .ItemCollection import ItemCollectionModule
from .PathProcess import PathProcess

STATE = Enum('STATE', [
	'LOST',
	'LOST_OUTSIDE_AISLE',
	'LOST_INSIDE_AISLE',
	'AISLE_DOWN',
	'BLIND_MOVE', # This one is more of an "action" than a state
	'COLLECT_ITEM',
	'AISLE_OUT',
	'APPROACH_PACKING',
	'DROP_ITEM',
	'VEGETABLE',
	'FACE_AISLE_2',
	'AVOID_MOVE'
	])

PHASE = Enum('PHASE', [
	'COLLECT',
	'DROPOFF'
	])


class NavigationModule:
	
	MAX_ROBOT_VEL = 0.13 # m/s
	ROTATIONAL_BIAS = 0.83 #tweak this parameter to be more or less aggressive with turns vs straight
	Kp = 2.4 # proportional term. beware if its too high we will try to go backwards for sharp turns
	MAX_ROBOT_ROT = pi/3 # rad/s
	RADIUS = 0.13 # how far to stay away from wall

	@classmethod
	def set_velocity(cls, fwd, rot, delta=0, fwdlen=None, rotlen=None):
		#cls.forced_avoidance_timer_update(fwd, delta)

		# Specific.set_velocity(fwd, rot)
		if fwdlen is None and rotlen is None:
			PathProcess.set_velocity(fwd, rot)
		else:
			PathProcess.new_path([(fwd, rot, fwdlen, rotlen)])


	#region state machine
	# For each state define a start and update function stored in this dict
	state_callbacks = {}
	current_state = STATE.LOST
	current_phase = PHASE.COLLECT

	@classmethod
	def process_instruction(cls):

		instruction = cls.instructions[cls.current_instruction]

		cls.target_aisle = int(instruction[0])
		cls.target_bay = int(instruction[1])
		cls.target_side = instruction[2]
		cls.target_height = int(instruction[3])
		cls.target_object = instruction[4]
		print(f"Aisle: {cls.target_aisle}, Bay: {cls.target_bay}, Side: {cls.target_side}")

		cls.shelf_length = 112 #cm
		cls.bay_width = cls.shelf_length / 4
		cls.target_bay_distance = cls.shelf_length - cls.bay_width/2 - cls.target_bay*cls.bay_width
		print(f"We want to be {cls.target_bay_distance} cm from aisle marker {cls.target_aisle}")

	@classmethod
	def init(cls, initial_state, instructions, starting_instruction=0):

		cls.instructions = instructions
		cls.current_instruction = starting_instruction
		cls.process_instruction()

		cls.state_callbacks[STATE.LOST] = (cls.LOST_start, cls.LOST_update)
		cls.state_callbacks[STATE.LOST_OUTSIDE_AISLE] = (cls.LOST_OUTSIDE_AISLE_start, cls.LOST_OUTSIDE_AISLE_update)
		cls.state_callbacks[STATE.LOST_INSIDE_AISLE] = (cls.LOST_INSIDE_AISLE_start, cls.LOST_INSIDE_AISLE_update)
		cls.state_callbacks[STATE.AISLE_DOWN] = (cls.AISLE_DOWN_start, cls.AISLE_DOWN_update)
		cls.state_callbacks[STATE.BLIND_MOVE] = (cls.BLIND_MOVE_start, cls.BLIND_MOVE_update)
		cls.state_callbacks[STATE.COLLECT_ITEM] = (cls.COLLECT_ITEM_start, cls.COLLECT_ITEM_update)
		cls.state_callbacks[STATE.AISLE_OUT] = (cls.AISLE_OUT_start, cls.AISLE_OUT_update)
		cls.state_callbacks[STATE.APPROACH_PACKING] = (cls.APPROACH_PACKING_start, cls.APPROACH_PACKING_update)
		cls.state_callbacks[STATE.DROP_ITEM] = (cls.DROP_ITEM_start, cls.DROP_ITEM_update)
		cls.state_callbacks[STATE.VEGETABLE] = (lambda *args: None, lambda *args: None)
		cls.state_callbacks[STATE.FACE_AISLE_2] =  (cls.FACE_AISLE_2_start, cls.FACE_AISLE_2_update)
		cls.state_callbacks[STATE.AVOID_MOVE] =  (cls.AVOID_MOVE_start, cls.AVOID_MOVE_update)

		cls.current_state = initial_state
		cls.t_now = time.time()
		cls.call_current_start()
		#cls.forced_avoidance_start()

		PathProcess.RADIUS = cls.RADIUS
		PathProcess.Start()
	
	@classmethod
	def end(cls):
		PathProcess.End()
	

	# Call the start function for the current state
	@classmethod
	def call_current_start(cls):
		print(f"State is now {cls.current_state.name}")
		return cls.state_callbacks[cls.current_state][0]()
	
	# Call the update function for the current state
	@classmethod
	def call_current_update(cls, delta,*args):
		return cls.state_callbacks[cls.current_state][1](delta,*args)

	# Call current update function then update the current state
	@classmethod
	def update(cls,*args):
		t_last = cls.t_now
		cls.t_now = time.time()
		delta = cls.t_now - t_last

		new_state, debug_img = cls.call_current_update(delta,*args)
		if new_state != cls.current_state:
			cls.current_state = new_state
			cls.call_current_start()


		img2 = np.zeros_like(debug_img)
		def mark(point, color=(255,255,255), marker =cv2.MARKER_DIAMOND):
			cv2.drawMarker(img2, tuple((np.array([SCREEN_WIDTH/2, SCREEN_HEIGHT/2]) + np.array([100, -100]) * point[::-1]).astype(np.int32)), color, cv2.MARKER_DIAMOND, 4, 2)
		for point in PathProcess.get_tracked_points_relative(bank=1):
			mark(point)
		mark(PathProcess.transform_world_to_bot(np.array([[0,0]]))[0], (255,0,0))
		mark(np.array([0]), (0,255,255), marker=cv2.MARKER_CROSS)

		return debug_img, img2
	#endregion

	#region Utility Functions
	
	
	
	@classmethod
	def expand_safety(cls, dist_map):
		d_theta = FOV_HORIZONTAL / SCREEN_WIDTH # angle per pixel horizontally
		p_factor = SCREEN_WIDTH / FOV_HORIZONTAL
		# left to right
		
		should_expand = dist_map[:, 1]
		dist_map = dist_map[:,0].copy()


		def safety_pass(enumeration, indexf):
			effect = []
			dist_n = dist_map.copy()
			r_f = 1.0
			for i, curr_dist in enumeration:
				

				# the points are in order of expiry to so only check the first few until we find one that isnt expired
				found = False
				while len(effect) > 0 and not found:
					if effect[0][1] < i:
						effect.pop(0)
					else:
						found = True
				
				# How long would this point be effective for if we stored it
				new_expiry_pix = int(i + p_factor * np.arctan(cls.RADIUS/curr_dist))

				# if effect is empty include this point and continue
				if len(effect) == 0:
					dist_n[indexf(i)] = curr_dist - r_f*cls.RADIUS
					if should_expand[indexf(i)]:
						effect.append((curr_dist, new_expiry_pix))
					continue

				# The maximum distance allowed at this point is the minimum dist of points still in effect
				# The minimum effective point.
				min_dist_effect, min_dist_expiry = effect[0]

				# Diamond form
				circle_r_pixels = p_factor * np.arctan(cls.RADIUS/min_dist_effect)
				circle_x_pixels = circle_r_pixels - (min_dist_expiry - i)
				center_proximity_fac = 1.0 - abs(circle_x_pixels/circle_r_pixels) # range from 0 to 1
				min_dist_effect = min_dist_effect - cls.RADIUS * r_f * center_proximity_fac

				if curr_dist - r_f*cls.RADIUS < min_dist_effect: # If this point is a new minimum, store it at the beginning
					# update the current point
					dist_n[indexf(i)] = curr_dist - r_f*cls.RADIUS


					# add the current point
					if should_expand[indexf(i)]:
						# first we need to remove any points that expire before this new one
						found = False
						while len(effect) > 0 and not found:
							if effect[0][1] <= new_expiry_pix:
								effect.pop(0)
							else:
								found = True
						effect.insert(0, (curr_dist, new_expiry_pix))
				
				else: # If this point is not a new minimum then take the minimum effective dist
					# update the current point


					dist_n[indexf(i)] = min_dist_effect

					# Still make this point effective if it might expire later than the current minimum effective point
					if should_expand[indexf(i)] and (new_expiry_pix > min_dist_expiry):
						# first we need to remove any points that expire before this new one
						
						# get j to be the index of the first point with dist >= new (curr) dist
						j = 0
						while j < len(effect) and effect[j][0] < curr_dist:
							j += 1

						found = False
						while j < len(effect) and not found:
							if effect[j][1] <= new_expiry_pix:
								effect.pop(j)
							else:
								found = True

						# add the current point
						effect.insert(j, (curr_dist, new_expiry_pix))
			return dist_n


		# We have to do this now so first pass doesnt influence 2nd pass
		safety_1 = safety_pass(enumerate(dist_map), lambda i:i)
		safety_2 = safety_pass(enumerate(reversed(dist_map)), lambda i:-i-1)

		safety_map = np.minimum(safety_1, safety_2)

		return safety_map

	@staticmethod
	def checkContour(cont):
		return cont is not None and len(cont) > 0


	@classmethod
	def forced_avoidance_corner_tracking(cls, corners):
		
		# Find which points in track points 1 moved out of the screen and add them to track 2.
		points = PathProcess.get_tracked_points_relative(bank=0)
		if points is not None and len(points) > 0:
			pixels = VisionModule.project_to_screen(points)
			margin = 8 # pixels
			for point, pixel in zip(points, pixels):
				if pixel[1] > SCREEN_HEIGHT-1-margin or pixel[0] < 0+margin or pixel[0] > SCREEN_WIDTH-1-margin:
					PathProcess.add_tracked_point_relative(point, bank=1)
					print(f"got a point to avoid {point}")

		# Filter to only keep the closest point on each side in track 2.
		cls.process_tracked_points(bank=1)

		# clear track 1 and add current corners to track 1
		PathProcess.clear_tracked_points(bank=0)
		corner_points = VisionModule.project_to_ground(corners)
		for point in corner_points:
			PathProcess.add_tracked_point_relative(point, bank=0)

	@classmethod
	def fa_process_safety(cls, safety_map):
		# Update safety_map from bank 1 (closest tracked point left and right)
		points = PathProcess.get_tracked_points_relative(bank=1)
		if points is not None and len(points) > 0:
			pixels = VisionModule.project_to_screen(points)

			for point, pixel in zip(points, pixels):
				# Update safety map here
				#angle = np.arctan2(point[1], point[0])#(pixel[0] - SCREEN_WIDTH/2) * FOV_HORIZONTAL / SCREEN_WIDTH
				dist = np.linalg.norm(point)

				#print(f"Update safety map with angle {angle}, dist {dist}")

				r_pixels = np.arctan(cls.RADIUS/dist) * SCREEN_WIDTH / FOV_HORIZONTAL
				if point[1] < 0:
					effective_edge = int(pixel[0] + r_pixels)
					#print(f"Point is on the left, affects up until: {effective_edge}")
					for i in range(0, min(effective_edge, SCREEN_WIDTH-1)):
						# Diamond form
						x_pixels = r_pixels - (effective_edge - i)
						center_proximity_fac = 1.0 - abs(x_pixels/r_pixels) # range from 0 to 1

						safety_map[i] = min(safety_map[i], dist - center_proximity_fac * cls.RADIUS)


				else:
					effective_edge = int(pixel[0] - r_pixels)
					#print(f"Point is on the right, affects from: {effective_edge}")
					for i in range(max(0,effective_edge), SCREEN_WIDTH-1):
						# Diamond form
						x_pixels = r_pixels + (effective_edge - i)
						center_proximity_fac = 1.0 - abs(x_pixels/r_pixels) # range from 0 to 1

						safety_map[i] = min(safety_map[i], dist - center_proximity_fac * cls.RADIUS)
				

		return safety_map

	@classmethod
	def process_tracked_points(cls, bank=0):
		# Delete points which are in view, behind robot, or too far away
		relative = PathProcess.get_tracked_points_relative(bank)
		if relative is None or len(relative)<1:
			return
		
		# Behind
		del1_mask = relative[:, 0] < 0 
		
		# Not effective to safety map 
		# This could be deterimental if we turn away then turn back which we probably will!
		# pixels = VisionModule.project_to_screen(relative)
		# angle = (pixels[0] - SCREEN_WIDTH/2) * FOV_HORIZONTAL / SCREEN_WIDTH
		# dist = np.linalg.norm(relative, axis=1)
		# effective_edge = pixels[:, 0] - np.sign(relative[:, 1]) * np.arctan(cls.RADIUS/dist) * SCREEN_WIDTH / FOV_HORIZONTAL
		# del2_mask = (effective_edge < 0) | (effective_edge > SCREEN_WIDTH-1)

		# Too far away
		del3_mask = relative[:, 0]**2+relative[:, 1]**2 > 0.8**2 

		# if del1_mask.any():
		#     print(f"{np.sum(del1_mask)} points are behind")
		# # if del2_mask.any():
		# #     print(f"{np.sum(del2_mask)} points are not effective")
		# if del3_mask.any():
		#     print(f"{np.sum(del3_mask)} points are too far")

		relative = relative[~(del1_mask  | del3_mask), :]
		dists = np.linalg.norm(relative, axis=1)


		# Only keep closest point on left and right
		PathProcess.clear_tracked_points(bank)
		left_mask = relative[:, 1] < 0
		if left_mask.any():
			for left in relative[left_mask, :]:
			# left = np.argmin(dists[left_mask])
			# left = relative[left_mask, :][left, :]
				PathProcess.add_tracked_point_relative(left, bank)

		right_mask = relative[:, 1] > 0
		if right_mask.any():
			for right in relative[right_mask, :]:
			# right = np.argmin(dists[right_mask])
			# right = relative[right_mask][right, :]
				PathProcess.add_tracked_point_relative(right, bank)


	@classmethod
	def move_into_path(cls, target_bearing, debug_img, contours, handle_outer=True, handle_outer_value=SCREEN_HEIGHT-1, force_forward=False):
		# Todo implement this as an funciton to do all the shelf/obstacle contour point processing and avoidance
		# We might want to move towards the marker if visible, or track the marker from where we last saw it??
		# Or an esimated entry point

		def calculate_fwd_rot(goal_error):
			rotational_vel = max(min(goal_error*cls.Kp, cls.MAX_ROBOT_ROT), -cls.MAX_ROBOT_ROT)
			forward_vel = cls.MAX_ROBOT_VEL * (1.0 - cls.ROTATIONAL_BIAS*abs(rotational_vel)/cls.MAX_ROBOT_ROT)
			return forward_vel, rotational_vel 

		if debug_img is not None:
			cv2.drawContours(debug_img, contours, -1, (0,0,255), 1)

		points = VisionModule.combine_contour_points(contours, exclude_horizontal_overlap=False)
		if handle_outer:
			points = VisionModule.handle_outer_contour(points, handle_outer_value)
		points, projected_floor = VisionModule.project_and_filter_contour(points)



		if points is not None and points.shape[0] > 3:
			dist_map = VisionModule.get_dist_map(points, projected_floor) # dist map column 0 is dist, column 1 is real point
		else:
			dist_map = np.ones((SCREEN_WIDTH, 2), np.float64)

		# Dist map processing
		dist_map[:, 0] = np.clip(dist_map[:, 0], None, 0.56) # ignore anything far away

		safety_map = NavigationModule.expand_safety(dist_map)
		safety_map = cls.fa_process_safety(safety_map)

		# Target
		# If target is none go longest
		attractive_map = np.zeros(safety_map.size)
		if type(target_bearing) == str and target_bearing == 'longest':
			pass
		else:
			for i in range(safety_map.size):
				bearing = (i/SCREEN_WIDTH - 1/2) * FOV_HORIZONTAL
				attractive_map[i] = 0.5 - 0.15 * abs(bearing - target_bearing)
		potential_map = attractive_map + safety_map

		# if debug_img is not None:
		# 	debug_img = cv2.drawContours(debug_img, contours, -1, (0,0,255),2)
		# 	debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - dist_map[:,0]/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 0), 1) # draw
		# 	debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (255, 255, 0), 1) # draw
				
		# Only can occur when there is no target
		if abs(potential_map.min() - potential_map.max()) < 0.01:
			if not force_forward:
				# No safe path: move backwards
				#cls.forced_avoidance_start() #reset FA
				forward_vel = -cls.MAX_ROBOT_VEL/2
				rotational_vel = 0
				path_distance = 0
			else:
				if type(target_bearing) == str and target_bearing == 'longest':
					goal_error = 0
				else:
					goal_error = target_bearing
				path_distance = None
				forward_vel, rotational_vel = calculate_fwd_rot(goal_error)

				if debug_img is not None:
					debug_img = cv2.drawMarker(debug_img, (int(target_bearing*SCREEN_WIDTH/FOV_HORIZONTAL+SCREEN_WIDTH/2), SCREEN_HEIGHT//2), (0,255,255), cv2.MARKER_STAR, 10)
		else:
			# Move into highest potential

			goal_error = (potential_map.argmax() - SCREEN_WIDTH/2) / (SCREEN_WIDTH) * FOV_HORIZONTAL
			path_distance = max(dist_map[:,0])
			forward_vel, rotational_vel = calculate_fwd_rot(goal_error)

			if debug_img is not None:
				debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - potential_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw
				debug_img = cv2.drawMarker(debug_img, (potential_map.argmax(), int(SCREEN_HEIGHT - potential_map.max()/2 * SCREEN_HEIGHT)), (0,255,255), cv2.MARKER_STAR, 10)
			

		return forward_vel, rotational_vel, path_distance
		
		# if force_forward:
		# 	if type(target_bearing) == str and target_bearing == 'longest':
		# 		goal_error = 0
		# 	else:
		# 		goal_error = target_bearing
		# 	path_distance = None
		# 	forward_vel, rotational_vel = calculate_fwd_rot(goal_error)

		# 	if debug_img is not None:
		# 		debug_img = cv2.drawMarker(debug_img, (int(goal_error*SCREEN_WIDTH/FOV_HORIZONTAL+SCREEN_WIDTH/2), SCREEN_HEIGHT//2), (0,255,255), cv2.MARKER_STAR, 10)
		# 	return forward_vel, rotational_vel, path_distance
				
		# return -cls.MAX_ROBOT_VEL/4, cls.MAX_ROBOT_ROT/2, None

	
	times = []
	tnames = []
	@classmethod
	def time_start(cls):
		cls.times = [time.time()]
		cls.tnames = []
	
	@classmethod
	def time_tick(cls, name):
		cls.times.append(time.time())
		cls.tnames.append(name)

	
	@classmethod
	def time_show(cls):
		for i in range(len(cls.tnames)):
			print(f"{cls.tnames[i]}: \t{cls.times[i+1] - cls.times[i]:.4f}")


	#endregion
	
	#region State Callbacks

	@classmethod
	def LOST_start(cls):
		Specific.leds(0b001)
		print("Lost!")
		cls.is_inside_aisle = True
		PathProcess.new_path([(0, cls.MAX_ROBOT_ROT, None, 2*pi)])
	
	@classmethod
	def LOST_update(cls, delta, debug_img, visout):
		# turn 360
		# if we at any point see no shelves, -> LOST_OUTSIDE_AISLE
		# if not, -> LOST_INSIDE_AISLE
		# optionally calculate a (m/s to pwm factor)
		# also if we are in dropoff, try to remember where the packing station is. 

		if visout.detected_shelves is None:
			cls.is_inside_aisle = False
			return STATE.LOST_OUTSIDE_AISLE, debug_img
	
		if PathProcess.completed:
			return STATE.LOST_INSIDE_AISLE, debug_img


		return STATE.LOST, debug_img



	@classmethod
	def LOST_OUTSIDE_AISLE_start(cls):
		Specific.leds(0b001)
		if cls.current_phase == PHASE.COLLECT:
			# if we're going to aisle, find (estimate) entry point
			cls.found_entry_point = False
			cls.at_entry_point = False
			if cls.target_aisle == 1:
				cls.set_velocity(0, cls.MAX_ROBOT_ROT)
			elif cls.target_aisle == 2:
				cls.set_velocity(0, cls.MAX_ROBOT_ROT)
			elif cls.target_aisle == 3:
				cls.set_velocity(0, -cls.MAX_ROBOT_ROT)
		elif cls.current_phase == PHASE.DROPOFF:
			# if we're going to packing station, go there
			# (face it then -> APPROACH_PACKING)
			cls.set_velocity(0, -cls.MAX_ROBOT_ROT)
	
	@classmethod
	def LOST_OUTSIDE_AISLE_update(cls, delta, debug_img, visout):
		if cls.current_phase == PHASE.COLLECT:
			# if we're going to aisle, find (estimate) entry point
			# if not cls.at_entry_point and visout.aisle == cls.target_aisle and not cls.checkContour(visout.contoursLoadingArea):
			# 	return STATE.AISLE_DOWN, debug_img
			
			if not cls.found_entry_point:
				if ((cls.target_aisle == 1 or cls.target_aisle == 2 or cls.target_aisle == 3) and visout.detected_shelves is not None):# or\
					#((cls.target_aisle == 2) and visout.detected_shelves is not None):# and abs(sum((shelf[0]-SCREEN_WIDTH/2)*FOV_HORIZONTAL/SCREEN_WIDTH for shelf in visout.detected_shelves)/len(visout.detected_shelves)) < 10*pi/180):
					print("Found entry point")
					cls.found_entry_point = True
				# elif cls.target_aisle == 2 and visout.detected_shelves is None:
				# 	return STATE.FACE_AISLE_2, debug_img

			if cls.found_entry_point and not cls.at_entry_point:
				if not cls.checkContour(visout.contoursShelf):
					print("Saw shelf but can't see it anymore")
					cls.found_entry_point = False
					if cls.target_aisle == 1:
						cls.set_velocity(0, cls.MAX_ROBOT_ROT)
					elif cls.target_aisle == 2:
						cls.set_velocity(0, cls.MAX_ROBOT_ROT)
					elif cls.target_aisle == 3:
						cls.set_velocity(0, -cls.MAX_ROBOT_ROT)
					return STATE.LOST_OUTSIDE_AISLE, debug_img

				if cls.target_aisle == 1 or cls.target_aisle == 2:
					def get_left_side(cont):
						x,y,w,h = cv2.boundingRect(cont.astype(np.int32))
						return x
					bearing = get_left_side(min(visout.contoursShelf, key=get_left_side)-SCREEN_WIDTH/2)*FOV_HORIZONTAL/SCREEN_WIDTH
					fwd, rot, distance = cls.move_into_path(bearing, debug_img, visout.contours) # target=None is longest path
				# elif cls.target_aisle == 2:
				# 	avoid_bearing = None
				# 	if visout.aisle > 0 and visout.aisle != cls.target_aisle:
				# 		avoid_bearing = visout.marker_bearing # don't go into the wrong aisle
				# 	fwd, rot, distance = cls.move_into_path('longest', debug_img, visout.contours, avoid_bearing=avoid_bearing) # target=None is longest path
				elif cls.target_aisle == 3:
					def get_right_side(cont):
						x,y,w,h = cv2.boundingRect(cont.astype(np.int32))
						return x+w
					bearing = get_right_side(max(visout.contoursShelf, key=get_right_side)-SCREEN_WIDTH/2)*FOV_HORIZONTAL/SCREEN_WIDTH
					fwd, rot, distance = cls.move_into_path(bearing, debug_img, visout.contours) # target=None is longest path



				
				if distance is not None and ((cls.target_aisle == 1 and distance < 0.23) or\
					(cls.target_aisle == 2 and distance < 1.2) or\
					(cls.target_aisle == 3 and distance < 0.23)):
					if visout.detected_shelves[0][0] > SCREEN_WIDTH/2:
						# shelf is on the right
						cls.set_velocity(0, cls.MAX_ROBOT_ROT, rotlen=2*pi)
					else:
						# shelf is on the left
						cls.set_velocity(0, -cls.MAX_ROBOT_ROT, rotlen=2*pi)
					print(f"At entry point ({distance:.2f})")
					cls.at_entry_point = True
				else:
					cls.set_velocity(fwd, rot, delta, rotlen=2*pi)
			

			if cls.at_entry_point:
				# Assume we are at the right aisle
				# Accept just a bearing (marker can be obscured)
				# Note aisle_down might need the marker not to be obscured...
				#if visout.marker_bearing is not None:
				if visout.aisle == cls.target_aisle:
					return STATE.AISLE_DOWN, debug_img
					# cls.next_state = STATE.AISLE_DOWN
					# cls.set_velocity(-cls.MAX_ROBOT_VEL, cls.MAX_ROBOT_ROT, rotlen=visout.marker_bearing)
					# return STATE.BLIND_MOVE, debug_img
				elif visout.aisle > 0 and not cls.checkContour(visout.contoursLoadingArea):
					# We can see a marker but not the right one

					print(f"We're at the wrong aisle. (aisle {visout.aisle})")

					# Go out of the aisle
					cls.set_velocity(0, cls.MAX_ROBOT_ROT, rotlen=pi)
					cls.next_state =  STATE.AISLE_OUT
					return STATE.BLIND_MOVE, debug_img

					# if cls.target_aisle > visout.aisle:
					# 	# go right
					# 	PathProcess.new_path([(0, cls.MAX_ROBOT_ROT, None, 2*pi/3),(cls.MAX_ROBOT_VEL/2, 0, 0.4, None)])
					# elif cls.target_aisle < visout.aisle:
					# 	# go left
					# 	PathProcess.new_path([(0, -cls.MAX_ROBOT_ROT, None, 2*pi/3),(cls.MAX_ROBOT_VEL/2, 0, 0.4, None)])
					# cls.next_state = STATE.LOST_OUTSIDE_AISLE
					# return STATE.BLIND_MOVE, debug_img
				
				if PathProcess.completed:
					# Will need another case here or it will get stuck
					print("At entry point but can't see marker")


		elif cls.current_phase == PHASE.DROPOFF:
			# if we're going to packing station, go there
			# (face it then -> APPROACH_PACKING)
			if cls.checkContour(visout.contoursLoadingArea):
				
				x, y, w, h = cv2.boundingRect(visout.contoursLoadingArea[0])
				cx = x + w/2
				bearing = (cx - SCREEN_WIDTH/2) * FOV_HORIZONTAL/SCREEN_WIDTH

				cls.set_velocity(0, cls.Kp*bearing, rotlen=bearing)

				if abs(bearing) < 10 * pi/180:
					return STATE.APPROACH_PACKING, debug_img
			else:
				cls.set_velocity(0, cls.MAX_ROBOT_ROT)
			
		return STATE.LOST_OUTSIDE_AISLE, debug_img

	@classmethod
	def FACE_AISLE_2_start(cls):
		Specific.leds(0b011)
		cls.fa2_stage = 0
		cls.set_velocity(0, cls.MAX_ROBOT_ROT)
	
	@classmethod
	def FACE_AISLE_2_update(cls, delta, debug_img, visout):
		# Starting from the outside of the aisles, face the aisle 2 entry point by finding the center of visible shelves.

		# stage 0: turn until we can't see shelf
		# stage 1: turn until we can see shelf and record time
		# stage 2: turn until we can't see shelf and recird time
		# stage 3: turn other way half of the time difference
		if visout.aisle == cls.target_aisle:
			return STATE.AISLE_DOWN, debug_img

		if cls.fa2_stage == 0:
			#cls.set_velocity(0, cls.MAX_ROBOT_ROT)
			if visout.detected_shelves is None:
				cls.fa2_stage += 1
		elif cls.fa2_stage == 1:
			#cls.set_velocity(0, cls.MAX_ROBOT_ROT)
			if visout.detected_shelves is not None:
				
				points = VisionModule.combine_contour_points(visout.contours, exclude_horizontal_overlap=False)
				points, projected_floor = VisionModule.project_and_filter_contour(points)
				cls.fa2_L1 = min(projected_floor[:, 0])

				print("Left shelf side")
				cls.fa2_t1 = time.time()
				cls.fa2_stage += 1
		elif cls.fa2_stage == 2:
			cls.set_velocity(0, cls.MAX_ROBOT_ROT)
			if visout.detected_shelves is None:
				print("Right shelf side")
				cls.fa2_t2 = time.time()
				shelf_width_seconds = cls.fa2_t2 - cls.fa2_t1

				alpha = shelf_width_seconds * cls.MAX_ROBOT_ROT - FOV_HORIZONTAL
				point = ((cls.fa2_L1 * sin(alpha))/2, (cls.fa2_L1 * cos(alpha) + cls.fa2_L2)/2)
				gamma = np.arctan2(point[1], point[0])
				cls.fa2_entrydist = np.sqrt(point[0]**2 + point[1]**2)

				print(f"alpha = {alpha:.3f}, gamma = {gamma:.3f}, entry_dist = {cls.fa2_entrydist:.3f}")

				cls.set_velocity(0, -cls.MAX_ROBOT_ROT, rotlen=gamma + FOV_HORIZONTAL/2)
				cls.fa2_stage += 1
			else:
				points = VisionModule.combine_contour_points(visout.contours, exclude_horizontal_overlap=False)
				points, projected_floor = VisionModule.project_and_filter_contour(points)
				if projected_floor is None or len(projected_floor) < 1:
					pass
				else:
					cls.fa2_L2 = min(projected_floor[:, 0])
		elif cls.fa2_stage == 3:
			if PathProcess.completed:
				print("Facing aisle 2")
				# fwd, rot, dist = cls.move_into_path(0, debug_img, visout.contours)
				# cls.am_target_x = dist
				# cls.am_target_y = 0
				# cls.am_proximity_thresh = 0.15
				# cls.am_traversed_thresh = dist
				
				# print(f"Entry dist {cls.fa2_entrydist:.2f}")
				# cls.set_velocity(cls.MAX_ROBOT_VEL, 0, fwdlen=cls.fa2_entrydist)
				# cls.next_state = STATE.LOST_OUTSIDE_AISLE
				
				if cls.fa2_L1 > cls.fa2_L2:
					cls.set_velocity(0, -cls.MAX_ROBOT_ROT, rotlen=0.8*(cls.fa2_t2 - cls.fa2_t1) * cls.MAX_ROBOT_ROT-FOV_HORIZONTAL/2)
					cls.next_state = STATE.LOST_OUTSIDE_AISLE
					return STATE.BLIND_MOVE, debug_img
				else:
					cls.set_velocity(0, -cls.MAX_ROBOT_ROT, rotlen=pi/8)
					cls.next_state = STATE.LOST_OUTSIDE_AISLE
					return STATE.BLIND_MOVE, debug_img

		return STATE.FACE_AISLE_2, debug_img


	@classmethod
	def LOST_INSIDE_AISLE_start(cls):
		Specific.leds(0b001)
	
	@classmethod
	def LOST_INSIDE_AISLE_update(cls, delta, debug_img, visout):
		if cls.current_phase == PHASE.COLLECT:
			# find marker then -> AISLE_DOWN
			pass
		elif cls.current_phase == PHASE.DROPOFF:
			# find exit (2 shelves no marker) then -> AISLE_OUT
			pass
		return cls.current_state, debug_img



	@classmethod
	def AISLE_DOWN_start(cls):
		Specific.leds(0b100)
	
	@classmethod
	def AISLE_DOWN_update(cls, delta, debug_img, visout):
		# move forward avoid obstacles. 
		# try to move towards the marker, if we can't see the marker then use the longest safe path
		# Once we reach target distance (or less) queue a blind_move into collect_item into blind_move to face out
		# For bay3 an addition forward move is required

		if visout.marker_bearing is not None:
			fwd, rot, dist = cls.move_into_path(visout.marker_bearing, debug_img, visout.contours)
			dist = visout.marker_distance / 100
			
		else:
			fwd, rot, dist = cls.move_into_path('longest', debug_img, visout.contours)
			
		if dist is None:
			print ("This is a problem")


		if cls.target_bay == 3:
			margin = 0.4 
			thresh = cls.target_bay_distance/100 + margin

			if dist <= thresh:
				PathProcess.new_path([(cls.MAX_ROBOT_VEL, 0, margin, None)])
				cls.next_state = STATE.COLLECT_ITEM
				return STATE.BLIND_MOVE, debug_img
			else:
				cls.set_velocity(fwd, rot, delta, fwdlen = dist-thresh)
		else:
			if dist <= cls.target_bay_distance/100:
				return STATE.COLLECT_ITEM, debug_img
			else:
				cls.set_velocity(fwd, rot, delta, fwdlen = dist-cls.target_bay_distance/100)
			
		
		return STATE.AISLE_DOWN, debug_img



	@classmethod
	def BLIND_MOVE_start(cls):
		Specific.leds(0b010)
	
	@classmethod
	def BLIND_MOVE_update(cls, delta, debug_img, visout):
		# This one is more of an "action" than a state
		# Use PathProcess and switch to another state when the movement is done

		if PathProcess.completed:
			return cls.next_state, debug_img

		return STATE.BLIND_MOVE, debug_img


	@classmethod
	def AVOID_MOVE_start(cls):
		# our target should be given by a point x,y relative to our current position and heading
		# x is distance in front and y is left-right
		# we set our position in pathprocess to the negative of this and try to navigate towards 0
		# can either terminate when estimated distance to the target is less than a threshold,
		# or when the estimated traversed distance exceeds a threshold.

		PathProcess.localize(-cls.am_target_x,cls.am_target_y, 0)

		if not hasattr(cls, 'am_proximity_thresh'):
			cls.am_proximity_thresh = None
		if not hasattr(cls, 'am_traversed_thresh'):
			cls.am_traversed_thresh = None
		Specific.leds(0b001)
		cls.avoid_moved = 0

		pass
	
	@classmethod
	def AVOID_MOVE_update(cls, delta, debug_img, visout):
		# like blind move but should avoid shelves and obstacles.
		# less strict about the movement obviously. We have a target direction and distance
		# target direction should be tracked through rotations
		# when we have moved target distance or get stuck then go to next state.

		# THis should also serve as our biased_move function to be able to move forward while turning left around the aisle. 

		x, y, h = PathProcess.get_current_track()
		cls.forced_avoidance_corner_tracking(np.array([s[0] for s in visout.shelfCorners]))
		

		if cls.am_traversed_thresh is not None and (x+cls.am_target_x)**2+(y+cls.am_target_y)**2 >= cls.am_traversed_thresh**2:
			return cls.next_state, debug_img
		elif cls.am_proximity_thresh is not None and x**2+y**2 <= cls.am_proximity_thresh**2:
			return cls.next_state, debug_img
		else:
			bearing = h - np.arctan2(-y, -x)
			cv2.drawMarker(debug_img, (int(bearing*SCREEN_WIDTH/FOV_HORIZONTAL + SCREEN_WIDTH/2), SCREEN_HEIGHT - int(200*np.linalg.norm(np.array([x,y])))), (255,0,0), cv2.MARKER_DIAMOND, 12)


			fwd, rot, dist = cls.move_into_path(bearing, debug_img, visout.contours, handle_outer=True, handle_outer_value = 1) # handle outer stuff means empty column (wall) is treated as far away.

			#print(f"{x = :.3f}, \t{y = :.3f}, \t{h = :.3f}, \t{bearing = :.3f}, \t{fwd = :.3f}, \t{rot = :.3f}, \tdist = {dist if dist is not None else -8888:.3f}")

			# Turn bearing then go straight...
			# cls.set_velocity(0,0)
			PathProcess.new_path([(fwd, rot, None, bearing), (fwd, 0, None, None)])

		# fwd, rot, dist = cls.move_into_path('longest', debug_img, visout.contours, handle_outer=False, force_forward=True)
		# cls.set_velocity(fwd,rot)

		# cls.avoid_moved += delta * fwd
		# if cls.avoid_moved >= cls.avoid_dist:
		# 	return cls.next_state, debug_img

		return STATE.AVOID_MOVE, debug_img
	


	@classmethod
	def COLLECT_ITEM_start(cls):
		Specific.leds(0b110)
		cls.collect_item_stage = 0
		if cls.target_side == 'Right':
			cls.set_velocity(0,cls.MAX_ROBOT_ROT,rotlen=pi/2)
		else:
			cls.set_velocity(0,-cls.MAX_ROBOT_ROT,rotlen=pi/2)
	
	@classmethod
	def COLLECT_ITEM_update(cls, delta, debug_img, visout):
		# Attempt to collect the item.

		if debug_img is not None and cls.checkContour(visout.contoursItem):
			cv2.drawContours(debug_img, visout.contoursItem, -1, (255,151,0), 1)

		if cls.collect_item_stage == 0:
			# Turn to bay
			if PathProcess.completed:
				cls.set_velocity(0,0)
				cls.collect_item_stage += 1
		elif cls.collect_item_stage == 1:
			
			if cls.checkContour(visout.contoursItem):
				
				largest_item = max(visout.contoursItem, key=lambda x:cv2.contourArea(x))

				x, y, w, h = cv2.boundingRect(largest_item)
				cx = x + w/2

				if debug_img is not None:
					cv2.drawMarker(debug_img, (int(cx), int(y+h/2)), (255,151,0), cv2.MARKER_STAR, 12)

				bearing = (cx - SCREEN_WIDTH/2) * FOV_HORIZONTAL/SCREEN_WIDTH
				
				if PathProcess.completed:#if abs(bearing) < 0.1:
					cls.set_velocity(0,0)
					cls.collect_item_stage += 1
				else:
					cls.set_velocity(0, cls.Kp * bearing, rotlen=abs(bearing))

			else:
				print("At bay but can't see item")
				if cls.target_side == 'Right':
					cls.set_velocity(0,-cls.MAX_ROBOT_ROT*0.8)
				else:
					cls.set_velocity(0,cls.MAX_ROBOT_ROT*0.8)

		elif cls.collect_item_stage == 2:
			# Lower item collection
			# Not implemented
			cls.set_velocity(cls.MAX_ROBOT_VEL/2,0,fwdlen=0.12)
			cls.collect_item_stage += 1
		elif cls.collect_item_stage == 3:
			# Move forward
			if PathProcess.completed:
				cls.set_velocity(0,0)
				cls.collect_item_stage += 1
		elif cls.collect_item_stage == 4:
			# Close gripper
			# Not implemented
			Specific.item_collection("CLOSE")
			cls.set_velocity(-cls.MAX_ROBOT_VEL/2,0,fwdlen=0.12)
			cls.collect_item_stage += 1
		elif cls.collect_item_stage == 5:
			# Move backwards
			if PathProcess.completed:
				if cls.target_side == 'Right':
					cls.set_velocity(0,cls.MAX_ROBOT_ROT,rotlen=pi/2)
				else:
					cls.set_velocity(0,-cls.MAX_ROBOT_ROT,rotlen=pi/2)
				cls.collect_item_stage += 1
		elif cls.collect_item_stage == 6:
			# Face outwards
			if PathProcess.completed:
				cls.current_phase = PHASE.DROPOFF
				return STATE.AISLE_OUT, debug_img

		return STATE.COLLECT_ITEM, debug_img



	@classmethod
	def AISLE_OUT_start(cls):
		Specific.leds(0b101)
		cls.last_shelf_side = None
		cls.aisle_out_stage = -1
	
	@classmethod
	def AISLE_OUT_update(cls, delta, debug_img, visout):
		# We should be able to see two shelves and no marker. (or marker and packing station)
		# Move towards the exit point, which is the center of the gap between the shelves
		# When we can no longer see the shelves, queue a blind move forwards into -> LOST_OUTSIDE_AISLE
		
		if cls.aisle_out_stage == -1:
			if visout.detected_shelves == None:
				cls.next_state = STATE.APPROACH_PACKING
				cls.avoid_dist = 0.4
				return STATE.AVOID_MOVE, debug_img
			cls.aisle_out_stage = 0

		if cls.aisle_out_stage == 0:

		# if visout.detected_wall is not None:
		# 	fwd, rot, dist = cls.move_into_path((visout.detected_wall[0][0]-SCREEN_WIDTH/2)*FOV_HORIZONTAL/SCREEN_WIDTH, debug_img, visout.contours, handle_outer=False)
		# 	cls.set_velocity(fwd, rot)
		# else:
			fwd, rot, dist = cls.move_into_path('longest', debug_img, visout.contours, handle_outer=False)
			cls.set_velocity(fwd, rot)

			if visout.detected_shelves is None:
				cls.ao_t1 = time.time()
				cls.set_velocity(0, -cls.MAX_ROBOT_ROT * np.sign(cls.last_shelf_side))
				cls.aisle_out_stage += 1
			elif visout.detected_shelves is not None:
				cls.last_shelf_side = (visout.detected_shelves[0][0]-SCREEN_WIDTH/2) * FOV_HORIZONTAL/SCREEN_WIDTH
		elif cls.aisle_out_stage == 1:
			if visout.detected_shelves is not None:
				cls.ao_t2 = time.time()
				PathProcess.new_path([(0, cls.MAX_ROBOT_ROT * np.sign(cls.last_shelf_side), None, (cls.ao_t2 - cls.ao_t1) * cls.MAX_ROBOT_ROT/2)])
				cls.aisle_out_stage += 1
		elif cls.aisle_out_stage == 2:
			if PathProcess.completed:
				cls.next_state = STATE.APPROACH_PACKING
				cls.avoid_dist = 0.4
				return STATE.AVOID_MOVE, debug_img



		return STATE.AISLE_OUT, debug_img



	@classmethod
	def APPROACH_PACKING_start(cls):
		Specific.leds(0b010)
		cls.loading_area_approach_stage = 0
		cls.set_velocity(0, cls.MAX_ROBOT_ROT, rotlen = 2*pi)
	
	@classmethod
	def APPROACH_PACKING_update(cls, delta, debug_img, visout):
		# We should be able to see the packing station at all times.
		# Move towards the center of the packing station until its bounding box touches left and right of screen
		# then move towards the marker until the packing station's bounding box touches the bottom of the screen
		# If collect then -> DROP_ITEM otherwise LOST_OUTSIDE_AISLE

		if cls.loading_area_approach_stage == 0:
			# Find loading area
			if cls.checkContour(visout.contoursLoadingArea):
				cls.loading_area_approach_stage += 1
			elif PathProcess.completed:
				return STATE.LOST_OUTSIDE_AISLE, debug_img
		elif cls.loading_area_approach_stage == 1:
			# move towards it
			if not cls.checkContour(visout.contoursLoadingArea):
				cls.loading_area_approach_stage -= 1
				cls.set_velocity(0, cls.MAX_ROBOT_ROT, rotlen = 2*pi)
			else:
				x,y,w,h = cv2.boundingRect(visout.contoursLoadingArea[0])
				if y+h >= SCREEN_HEIGHT-1:
					cls.loading_area_approach_stage += 1
					cls.set_velocity(0,0)
				else:
					cx = x + w/2
					bearing = (cx - SCREEN_WIDTH/2) * FOV_HORIZONTAL/SCREEN_WIDTH
					fwd, rot, dist = cls.move_into_path(bearing, debug_img, visout.contours, handle_outer=False, force_forward=True)
					cls.set_velocity(fwd, rot, delta)
		elif cls.loading_area_approach_stage == 2:
			# face marker
			if visout.marker_bearing is not None:
				if abs(visout.marker_bearing) < 0.1:
					return STATE.DROP_ITEM, debug_img
				cls.set_velocity(0, cls.Kp * visout.marker_bearing, rotlen=abs(visout.marker_bearing))
			else:
				print("At packing but can't see marker")
				cls.set_velocity(0, cls.MAX_ROBOT_ROT)

		return STATE.APPROACH_PACKING, debug_img



	@classmethod
	def DROP_ITEM_start(cls):
		Specific.leds(0b010)
		cls.drop_item_stage = 0
		cls.set_velocity(cls.MAX_ROBOT_VEL/2,0,fwdlen=0.53)
	
	@classmethod
	def DROP_ITEM_update(cls, delta, debug_img, visout):
		# Forward, drop, backwards, turn, forwards, -> LOST_OUTSIDE_AISLE

		if cls.drop_item_stage == 0:
			# Move forward
			if PathProcess.completed or visout.marker_bearing is None:
				cls.set_velocity(0,0)
				cls.drop_item_stage += 1
		elif cls.drop_item_stage == 1:
			# Drop item
			# Not implemented
			Specific.item_collection("OPEN")
			cls.set_velocity(-cls.MAX_ROBOT_VEL/2,0,fwdlen=0.43)
			cls.drop_item_stage += 1
		elif cls.drop_item_stage == 2:
			# Move backward
			if PathProcess.completed:
				if cls.target_aisle == 1: # This is the aisle we came from
					PathProcess.new_path([(0, -cls.MAX_ROBOT_ROT, None, pi/2)])
				elif cls.target_aisle == 2 or cls.target_aisle == 3:
					PathProcess.new_path([(0, -cls.MAX_ROBOT_ROT, None, 5*pi/6)])
				cls.drop_item_stage += 1
		elif cls.drop_item_stage == 3:
			# Turn around
			if PathProcess.completed:
				cls.current_phase = PHASE.COLLECT
				cls.current_instruction = (cls.current_instruction % len(cls.instructions)) + 1
				cls.process_instruction()

				cls.avoid_dist = 0.5
				cls.next_state = STATE.LOST
				return STATE.AVOID_MOVE, debug_img

		return STATE.DROP_ITEM, debug_img


	#endregion


	#region OLD State Callbacks
	# @classmethod
	# def lost_start(cls):
	# 	print("Lost!")

	# @classmethod
	# def lost_update(cls,delta, debug_img, *args):
	# 	return STATE.WANDER, debug_img

	# @classmethod
	# def wander_start(cls):
	# 	pass


	# @classmethod
	# def wander_update(cls, delta, debug_img, visout):
		
	# 	if visout.marker_distance is not None and abs(visout.marker_bearing) < 0.8 and visout.aisle == cls.target_aisle:
	# 		cls.wander_until_distance = None
	# 		return STATE.AISLE_DOWN, debug_img

	# 	points = VisionModule.combine_contour_points(visout.contours, False)
	# 	points = VisionModule.handle_outer_contour(points)
	# 	points, projected_floor = VisionModule.project_and_filter_contour(points)


	# 	dist_map = None
	# 	if points is not None and points.shape[0] > 3:
	
	# 		dist_map = VisionModule.get_dist_map(points, projected_floor) # dist map column 0 is dist, column 1 is real point
		
	# 		if debug_img is not None:
	# 			# draw
	# 			cv2.polylines(debug_img, [points[:, 0:2].astype(np.int32)], False, (0, 0, 255), 1) # draw
	# 			cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - dist_map[:,0]/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 0), 1) # draw
		
	# 		dist_map = cls.forced_avoidance(dist_map)

	# 		if debug_img is not None:
	# 			if cls.avoid_left > 0:
	# 				debug_img = cv2.putText(debug_img, "A", (20,20), 0, 1, (0,0,255), 2)
	# 			if cls.avoid_right > 0:
	# 				debug_img = cv2.putText(debug_img, "A", (SCREEN_WIDTH-20,20), 0, 1, (0,0,255), 2)

	# 		safety_map = cls.expand_safety(dist_map)
			
	# 		if debug_img is not None:
	# 			debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw


	# 		if abs(safety_map.max() - safety_map.min()) < 0.01:
	# 			# no safe path- turn and dont move forward
				
	# 			goal_error = (dist_map[:,0].argmax() - SCREEN_WIDTH/2) / (SCREEN_WIDTH) * FOV_HORIZONTAL
	# 			rotational_vel = max(min(goal_error*cls.Kp, cls.MAX_ROBOT_ROT), -cls.MAX_ROBOT_ROT)

	# 			#forward_vel = MAX_ROBOT_VEL * (1.0 - ROTATIONAL_BIAS*abs(rotational_vel)/MAX_ROBOT_ROT)
	# 			forward_vel = 0

	# 			if debug_img is not None:
	# 				debug_img = cv2.drawMarker(debug_img, (dist_map[:,0].argmax(), int(SCREEN_HEIGHT - dist_map[:,0].max()/2 * SCREEN_HEIGHT)), (0,0,255), cv2.MARKER_STAR, 10)

	# 			if abs(goal_error) < 0.05: #radians
	# 				print("Probably stuck")
	# 				cls.wander_until_distance = None
	# 				return STATE.FIND_AISLE_FROM_OUTSIDE, debug_img
				
	# 		else:
	# 			# move into the longest safe path
	# 			goal_error = (safety_map.argmax() - SCREEN_WIDTH/2) / (SCREEN_WIDTH) * FOV_HORIZONTAL
	# 			rotational_vel = max(min(goal_error*cls.Kp, cls.MAX_ROBOT_ROT), -cls.MAX_ROBOT_ROT)
	# 			forward_vel = cls.MAX_ROBOT_VEL * (1.0 - cls.ROTATIONAL_BIAS*abs(rotational_vel)/cls.MAX_ROBOT_ROT)

	# 			if debug_img is not None:
	# 				debug_img = cv2.drawMarker(debug_img, (safety_map.argmax(), int(SCREEN_HEIGHT - safety_map.max()/2 * SCREEN_HEIGHT)), (0,255,255), cv2.MARKER_STAR, 10)


	# 		if hasattr(cls, 'wander_until_distance') and cls.wander_until_distance is not None and dist_map[:, 0].max() < cls.wander_until_distance:
	# 			print(f"We wandered until {dist_map[:, 0].max()} < {cls.wander_until_distance}")
	# 			cls.wander_until_distance = None
	# 			return cls.next_state, debug_img
				
	# 	else:
	# 		cls.forced_avoidance(None)
	# 		rotational_vel = cls.MAX_ROBOT_ROT
	# 		forward_vel = 0

		
	# 	cls.set_velocity(forward_vel,rotational_vel, delta)


	# 	return STATE.WANDER, debug_img
	
	# @classmethod
	# def fafo_start(cls):
	# 	if not hasattr(cls, 'fafo_react_marker'):
	# 		cls.fafo_react_marker = False

	# @classmethod
	# def fafo_update(cls, delta, debug_img, visout):
	# 	# Aisle 1: rotate right until we cant see shelf then change to wander to continue rotating right until we see a shelf
	# 	# Aisle 2: rotate right until we cant see shelf then change to wander to continue rotating right until we see a shelf and move forward until we are 1m away then switch back to fafo
	# 	# Aisle 3: rotate until we see a marker

	# 	if cls.fafo_react_marker and visout.aisle is not None and visout.aisle == cls.target_aisle:
	# 		return STATE.AISLE_DOWN, debug_img


	# 	if cls.target_aisle == 1:
	# 		if visout.detected_shelves is not None:
	# 			cls.set_velocity(0, cls.MAX_ROBOT_ROT, delta)
	# 		else:
	# 			# Wander until we get stuck
	# 			cls.fafo_react_marker = True # When we get stuck we go to fafo then we should find the marker
	# 			return STATE.WANDER, debug_img
	# 	elif cls.target_aisle == 2:
	# 		if visout.detected_shelves is not None:
	# 			cls.set_velocity(0, cls.MAX_ROBOT_ROT, delta)
	# 		else:
	# 			# Wander until we are that far from furthest point
	# 			cls.fafo_react_marker = True # When we get back to fafo then we should find the marker
	# 			cls.wander_until_distance = 1.3 # "metres"
	# 			cls.next_state = STATE.FIND_AISLE_FROM_OUTSIDE
	# 			return STATE.WANDER, debug_img
				
	# 	elif cls.target_aisle == 3:
	# 		if visout.aisle is None or visout.aisle < 1:
	# 			cls.set_velocity(0, -cls.MAX_ROBOT_ROT, delta)
	# 		else:
	# 			return STATE.AISLE_DOWN, debug_img
	# 	else:
	# 		print("Target Aisle out of bounds")

	# 	return STATE.FIND_AISLE_FROM_OUTSIDE, debug_img

	# @classmethod
	# def aisle_down_start(cls):
	# 	pass
		

	# @classmethod
	# def aisle_down_update(cls, delta, debug_img, visout):


	# 	if visout.marker_distance is None:
	# 		return STATE.LOST, debug_img
		
	# 	if cls.target_bay < 3 and visout.marker_distance < cls.target_bay_distance:
	# 		return STATE.FACE_BAY, debug_img
	# 	elif cls.target_bay == 3 and visout.marker_distance < (cls.target_bay_distance + cls.bay_width):
	# 		return STATE.AISLE_DOWN_BAY3, debug_img

	# 	if visout.aisle > cls.target_aisle:
	# 		# Go back to fafo if we can see MORE markers than we should.
	# 		# If we can see less markers the markers could be occluded
	# 		return STATE.FIND_AISLE_FROM_OUTSIDE, debug_img
		

	# 	points = VisionModule.combine_contour_points(visout.contours, False)
	# 	points = VisionModule.handle_outer_contour(points)
	# 	points, projected_floor = VisionModule.project_and_filter_contour(points)
	# 	dist_map = None
	# 	if points is not None and points.shape[0] > 3:
			
	# 		dist_map = VisionModule.get_dist_map(points, projected_floor) # dist map column 0 is dist, column 1 is real point

	# 		if debug_img is not None:
	# 			# draw
	# 			cv2.polylines(debug_img, [points[:, 0:2].astype(np.int32)], False, (0, 0, 255), 1) # draw
	# 			cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - dist_map[:,0]/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 0), 1) # draw
		
	# 		dist_map = cls.forced_avoidance(dist_map)
			
	# 		if debug_img is not None:
	# 			if cls.avoid_left > 0:
	# 				debug_img = cv2.putText(debug_img, "A", (20,20), 0, 1, (0,0,255), 2)
	# 			if cls.avoid_right > 0:
	# 				debug_img = cv2.putText(debug_img, "A", (SCREEN_WIDTH-20,20), 0, 1, (0,0,255), 2)

	# 		safety_map = cls.expand_safety(dist_map)
	# 		# repulsive_map = (safety_map < 0.3)*1

	# 		attractive_map = np.zeros(safety_map.size)
	# 		for i in range(safety_map.size):
	# 			bearing = (i/SCREEN_WIDTH - 1/2) * FOV_HORIZONTAL
	# 			attractive_map[i] = 1.0 - 0.6 * abs(bearing - visout.marker_bearing)
			
	# 		if debug_img is not None:
	# 			debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - safety_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (255, 255, 0), 1) # draw

	# 		if abs(safety_map.min() - safety_map.max()) < 0.01:
	# 			cls.forced_avoidance_start() #reset FA
	# 			forward_vel = -cls.MAX_ROBOT_VEL/2
	# 			rotational_vel = 0
	# 		else:
	# 			potential_map = attractive_map + safety_map

	# 			if debug_img is not None:
	# 				debug_img = cv2.polylines(debug_img, [np.array([range(0, SCREEN_WIDTH), SCREEN_HEIGHT - potential_map/2 * SCREEN_HEIGHT]).T.astype(np.int32)], False, (0, 255, 255), 1) # draw
	# 				debug_img = cv2.drawMarker(debug_img, (potential_map.argmax(), int(SCREEN_HEIGHT - potential_map.max()/2 * SCREEN_HEIGHT)), (0,255,255), cv2.MARKER_STAR, 10)
				

	# 			goal_error = (potential_map.argmax() - SCREEN_WIDTH/2) / (SCREEN_WIDTH) * FOV_HORIZONTAL
	# 			rotational_vel = max(min(goal_error*cls.Kp, cls.MAX_ROBOT_ROT), -cls.MAX_ROBOT_ROT)
	# 			forward_vel = cls.MAX_ROBOT_VEL * (1.0 - cls.ROTATIONAL_BIAS*abs(rotational_vel)/cls.MAX_ROBOT_ROT)

			
			
	# 		cls.set_velocity(forward_vel,rotational_vel, delta)
	# 	else:
	# 		return STATE.LOST, debug_img
		

	# 	return STATE.AISLE_DOWN, debug_img
	
	# @classmethod
	# def adb3_start(cls):
	# 	cls.adb3_remaining = cls.bay_width/100 # cm to m
	# 	cls.set_velocity(0,0,0)

	# @classmethod
	# def adb3_update(cls, delta, debug_img, visout):

	# 	if visout.marker_bearing is not None:
	# 		goal_error = visout.marker_bearing
	# 		rotational_vel = max(min(goal_error*cls.Kp, cls.MAX_ROBOT_ROT), -cls.MAX_ROBOT_ROT)
	# 		forward_vel = cls.MAX_ROBOT_VEL * (1.0 - cls.ROTATIONAL_BIAS*abs(rotational_vel)/cls.MAX_ROBOT_ROT)
	# 	else:
	# 		forward_vel = cls.MAX_ROBOT_VEL
	# 		rotational_vel = 0
		
	# 	cls.set_velocity(forward_vel, rotational_vel, delta)

	# 	cls.adb3_remaining -= forward_vel*delta
	# 	if cls.adb3_remaining <= 0:
	# 		return STATE.FACE_BAY, debug_img


	# 	return STATE.AISLE_DOWN_BAY3, debug_img

	# @classmethod
	# def face_bay_start(cls):
	# 	cls.turn_to_bay_remaining = pi/2
	# 	cls.set_velocity(0,0, 0)
	
	# @classmethod
	# def face_bay_update(cls, delta, debug_img, visout):
		
	# 	rot = 0.35
		
	# 	cls.turn_to_bay_remaining -= rot*delta
	# 	if cls.turn_to_bay_remaining <= 0:
	# 		return STATE.COLLECT_ITEM, debug_img

	# 	if cls.target_side == "Left":
	# 		rot = -rot
	# 	cls.set_velocity(0, rot, delta)

	# 	return STATE.FACE_BAY, debug_img


	# @classmethod
	# def collect_item_start(cls):
	# 	cls.set_velocity(cls.MAX_ROBOT_VEL/2,0, 0)
	# 	cls.collect_item_fwd_remaining = 0.06
	
	# @classmethod
	# def collect_item_update(cls, delta, debug_img, visout):
	# 	if cls.collect_item_fwd_remaining > 0:
	# 		fwd = cls.MAX_ROBOT_VEL/2
	# 		cls.set_velocity(fwd,0,delta)
	# 		cls.collect_item_fwd_remaining -= fwd*delta
	# 	else:
	# 		cls.set_velocity(0,0,delta)
	# 		# ItemCollectionModule.gripper_close(1)


	# 	return STATE.COLLECT_ITEM, debug_img
	
	#endregion