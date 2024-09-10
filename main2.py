import cv2
import numpy as np
import time
import vision as vs
import RP2040 as I2C
import csv 
import csvread as csvread
import picamera2
from threading import Thread
from sys import argv

# Define the width and height for the camera feed
width = 616
height = 820
i2c = I2C.I2C()
vision = vs.VisionModule()
# Initialize the CamFrameGrabber
cam = vs.CamFrameGrabber(src=0, width=width, height=height)

def CapturingImage():
      frame = cam.getCurrentFrame()
      frame = cv2.flip(frame, 0)  # OPTIONAL: Flip the image vertically
      imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert to HSV
      robotview = img.copy()  # Preserve the original image
      #current_frame_id = cam.getFrameID()
      return frame, imgHSV, robotview
# Main function to demonstrate the usage of CamFrameGrabber
def main():
      # Define the width and height for the camera feed
      
      # Start capturing frames
      cam.start()
      
      # Create a window to display the frames
      #cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
      
      previous_frame_id = -1  # Store the previous frame ID for comparison
      
      try:
            while True:
                  # Start measuring time at the beginning of the frame capture process
                  
                  # Get the current frame and frame ID
                  frame = cam.getCurrentFrame()
                  frame = cv2.flip(frame, 0)  # OPTIONAL: Flip the image vertically
                  imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert to HSV
                  current_frame_id = cam.getFrameID()
                  
                  # Only display the frame if the frame ID is different from the previous one
                  if current_frame_id != previous_frame_id:
                  # Display the frame using OpenCV
                        cv2.imshow("Original Image", frame)
                        #cv2.imshow("", robotview)
                  # Update the previous frame ID
                        previous_frame_id = current_frame_id
                  
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

if __name__ == "__main__": # Run the main function
      CSV = csvread.CSVReader('Order_1.csv')
      CSV.read_csv()
      instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions
      
      main()
