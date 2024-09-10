import cv2
import picamera2
import numpy as np
from threading import Thread, Lock
import time
import vision as vs
# Define the runVision function (can be customized as needed)
def runVision():
    # Placeholder for vision processing logic
    range = 1
    bearing = 2
    return range, bearing

# Define the CamFrameGrabber class
class CamFrameGrabber:
    # FOV = number of degrees for camera view
    def __init__(self, src, width, height):
        # Initialize the camera
        self.camera = picamera2.Picamera2()
        self.width = width
        self.height = height
        
        # Define the configuration for the camera
        config = self.camera.create_video_configuration(
            main={"format": 'XRGB8888', "size": (height, width)}
        )
        self.camera.configure(config)
        self.camera.start()
        
        self.cameraStopped = False
        self.gotFrame = False
        self.currentFrame = np.zeros((height, width, 3), np.uint8)
        self.frameid = 0  # Initialize frame ID
        self.lock = Lock()  # Create a lock for thread safety
        
        # Capture the first frame
        self.currentFrame = self.camera.capture_array()

    def start(self):
        # Start the image capturing thread
        Thread(target=self.captureImage, args=()).start()  # Running the camera capturing in background threads
        return self

    def captureImage(self):
        # Continuously capture frames until stopped
        while True:
            if self.cameraStopped:
                return
            # Capture frame from the camera
            frame = self.camera.capture_array()
            with self.lock:
                self.currentFrame = frame  # Safely update the frame
            self.frameid += 1  # Increment the frame ID after capturing each frame

    def getCurrentFrame(self):
        # Safely return the current frame
        with self.lock:
            return self.currentFrame.copy()

    def getFrameID(self):
        # Return the current frame ID
        return self.frameid

    def stop(self):
        # Stop the camera capture
        self.cameraStopped = True

    def Displaying(self, WindowName, imgRGB):
        # Display the frame using OpenCV
        cv2.imshow(WindowName, imgRGB)

    def __del__(self):
        # Clean up OpenCV windows
        cv2.destroyAllWindows()

# Main function to demonstrate the usage of CamFrameGrabber
def main():
    # Define the width and height for the camera feed
    width = 616
    height = 820
    
    # Initialize the CamFrameGrabber
    cam = CamFrameGrabber(src=0, width=width, height=height)
    
    # Start capturing frames
    cam.start()
    
    try:
        while True:
            # Get the current frame and frame ID
            frame = cam.getCurrentFrame()
            
            # Display the frame
            cam.Displaying("testing", frame)
            
            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    except KeyboardInterrupt:
        # Gracefully handle a keyboard interrupt (Ctrl+C)
        print("Stopping camera capture...")

    # Stop the camera capture
    cam.stop()
    
    # Release resources and close windows
    cv2.destroyAllWindows()

# Run the main function
if __name__ == "__main__":
    main()
