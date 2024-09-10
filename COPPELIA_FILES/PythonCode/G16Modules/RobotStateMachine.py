from .Navigation import *
from .SimVision import *

# This is the class for hosting the control logic (logic happens in navigation state callbacks).
# It should be decoupled from any coppeliasim- or picamera- specific code
class RobotStateMachine:

	# Initial State
	current_state = STATE.WANDER

	@classmethod
	def init(cls, initial_state = STATE.WANDER, get_image_callback = None, set_velocity_callback = None):
		# Initialise vision and navigation 

		cls.current_state = initial_state
		Vision.get_image_callback = get_image_callback
		

		Navigation.initialise_callbacks()
		Navigation.set_velocity_callback = set_velocity_callback

		Navigation.calculate_view_transforms()
		Navigation.state_callbacks[cls.current_state][0]()

	# Call the start function for the current state
	@classmethod
	def call_current_start(cls):
		return Navigation.state_callbacks[cls.current_state][0]()
	
	# Call the update function for the current state
	@classmethod
	def call_current_update(cls):
		return Navigation.state_callbacks[cls.current_state][1]()

	# Call current update function then update the current state
	@classmethod
	def update(cls):
		new_state = cls.call_current_update()
		if new_state != cls.current_state:
			cls.current_state = new_state
			cls.call_current_start()