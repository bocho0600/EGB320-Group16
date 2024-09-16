from math import pi


SCREEN_WIDTH = 410
SCREEN_HEIGHT = 308

is_simulator = False
is_hitl = True

if is_simulator:
	FOV_HORIZONTAL = 60 * pi/180 # radians

	DIST_X = 0 # cameraDistanceFromRobotCenter
	DIST_Z = 0.0752 # cameraHeightFromFloor
	TILT = 1.5 * 3.1415926535 / 180

	from .SimSpecific import SimSpecific as Specific
else:
	FOV_HORIZONTAL = 64.4 * pi/180 # radians

	DIST_X = 0.09 # cameraDistanceFromRobotCenter
	DIST_Z = 0.109 # cameraHeightFromFloor
	TILT = -8.785 * pi / 180
	
	from .RealSpecific import RealSpecific as Specific

