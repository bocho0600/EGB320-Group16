import picamera2
import cv2
from .Globals import *

class RealSpecific:
	cap = None

	@classmethod
	def CaptureImage(cls):
		frame = cls.cap.capture_array()  # Use the instance variable
		frame = cv2.flip(frame, 0)  # OPTIONAL: Flip the image vertically
		img = cv2.resize(img, (SCREEN_WIDTH, SCREEN_HEIGHT))
		
		frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert to HSV
		return frame, frameHSV

	# alias
	get_image = CaptureImage
	
	@classmethod
	def initialize_camera(cls, frame_height=820, frame_width=616, format='XRGB8888'):
		# Create a camera object and store it as an instance variable
		cls.cap = picamera2.Picamera2()
		config = cls.cap.create_video_configuration(main={"format": format, "size": (frame_height, frame_width)})
		cls.cap.configure(config)
		
		
		# cls.cap.set_controls({"ExposureTime": 11000, "AnalogueGain": 1.5,  "ColourGains": (1.22,2.12)})
		cls.cap.set_controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})
		cls.image_width = frame_width
		cls.image_center = cls.image_width // 2 # Calculate the center of the image
		cls.cap.start()
	
	def set_velocity(fwd, rot):
		# not implemented
		pass

	@classmethod
	def start(cls):
		cls.initialize_camera()

	@classmethod
	def update(cls):
		pass

	@classmethod
	def end(cls):
		cls.cap.close()
