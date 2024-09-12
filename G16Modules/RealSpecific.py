import picamera2
import cv2
from .Globals import *
import time
from threading import Thread, Condition
import numpy as np

class RealSpecific:

	# Specific should implement:
	# get_frame
	# set_velocity
	# start
	# update
	# stop

	camera = None
	frameGrabber = None

	@classmethod
	def CaptureFrame(cls):
		# Run in the thread called by CamFrameGrabber
		frame = cls.camera.capture_array()  # Use the instance variable
		return frame

	@classmethod
	def CaptureFrameAsync(cls, sig_function):
		# Run in the thread called by CamFrameGrabber

		frame = cls.camera.capture_array(signal_function = sig_function)  # Use the instance variable
		return frame
	

	@classmethod
	def ConvertImage(cls, frame):
		# Run in main, called after frame is transferred
		img = cv2.flip(frame, 0)  # OPTIONAL: Flip the image vertically
		img = cv2.resize(img, (SCREEN_WIDTH, SCREEN_HEIGHT), cv2.INTER_NEAREST)
		
		imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
		return img, imgHSV
	

	@classmethod
	def get_image(cls):
		if cls.frameGrabber is not None:
			frame = cls.frameGrabber.getCurrentFrame
		else:
			frame = cls.CaptureFrame()
		return cls.ConvertImage(frame)

	@classmethod
	def initialize_camera(cls, frame_height=820, frame_width=616, format='XRGB8888'):
		# Create a camera object and store it as an instance variable
		cls.camera = picamera2.Picamera2()
		config = cls.camera.create_video_configuration(main={"format": format, "size": (frame_height, frame_width)})
		cls.camera.configure(config)
		
		
		# cls.cap.set_controls({"ExposureTime": 11000, "AnalogueGain": 1.5,  "ColourGains": (1.22,2.12)})
		# cls.cap.set_controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})
		cls.camera.set_controls({"ExposureTime": 70000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})

		cls.image_width = frame_width
		cls.image_center = cls.image_width // 2 # Calculate the center of the image
		cls.camera.start()
	
	def set_velocity(fwd, rot):
		# not implemented
		pass

	@classmethod
	def start(cls, grabber = True):
		cls.initialize_camera()
		if grabber:
			cls.frameGrabber = CamFrameGrabber(SCREEN_WIDTH, SCREEN_HEIGHT)
			cls.frameGrabber.start()

	@classmethod
	def update(cls):
		pass

	@classmethod
	def end(cls):
		cls.frameGrabber.stop()
		cls.camera.close()


# Define the CamFrameGrabber class
class CamFrameGrabber:
	# FOV = number of degrees for camera view
	def __init__(self, width, height):
		# Initialize the camera
		
		self.cameraStopped = False
		self.gotFrame = False
		self.currentFrame = np.zeros((height, width, 3), np.uint8)
		self.frameid = 0  # Initialize frame ID
		self.newFrameAvailable = False
		self.time_of_frame = 0
		self.fps = 0
		self.new_frame_condition = Condition()

		# Capture the first frame
		self.currentFrame = Specific.CaptureFrame()

	def start(self):
		# Start the image capturing thread
		self.previous_frame_id = -1
		self.grabberThread = Thread(target=self.captureImage, args=()).start()  # Running the camera capturing in background threads
		return self

	def captureImage(self):
		# Continuously capture frames until stopped
		while True:
			if self.cameraStopped:
				return
			
			# Capture frame from the camera
			with self.new_frame_condition:
				Specific.CaptureFrameAsync(self.captureImageAsyncSignal)
				self.new_frame_condition.wait()
			time.sleep(0.01)
			
			

	def captureImageAsyncSignal(self, job):
		with self.new_frame_condition:
			self.currentFrame = RealSpecific.camera.wait(job)
			t_last = self.time_of_frame
			self.time_of_frame = time.time()
			self.fps = 1.0 / (self.time_of_frame - t_last)
			
			self.frameid += 1  # Increment the frame ID after capturing each frame
			self.newFrameAvailable = True
			self.new_frame_condition.notify()

	def getCurrentFrame(self):
		# Return the current frame
		with self.new_frame_condition:
			self.newFrameAvailable = False
			
			return self.currentFrame.copy()

	def getFrameID(self):
		# Return the current frame ID
		return self.frameid

	def stop(self):
		# Stop the camera capture
		self.cameraStopped = True
		if self.grabberThread.is_alive():
			self.grabberThread.join()


	# def Displaying(self, WindowName, imgRGB):
	#     # Initialize previous_frame_id if it hasn't been initialized yet
	#     if not hasattr(self, 'previous_frame_id'):
	#         self.previous_frame_id = -1
		
	#     current_frame_id = self.getFrameID()
		
	#     # Only display the frame if the frame ID is different from the previous one
	#     if current_frame_id != self.previous_frame_id:
	#         # Display the frame using OpenCV
	#         cv2.imshow(WindowName, imgRGB)
		
	#     # Update the previous frame ID
	#     self.previous_frame_id = current_frame_id


	# def __del__(self):
	#     # Release the camera and clean up OpenCV windows
	#     self.camera.release()
	#     cv2.destroyAllWindows()
