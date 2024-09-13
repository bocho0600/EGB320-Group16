# -*- coding: utf-8 -*-
import cv2
import picamera2
import numpy as np
from threading import Thread
import vision as vs
import time

def runVision():
    # Placeholder for actual vision processing
    range = 1
    bearing = 2
    return range, bearing


class CamFrameGrabber:
      # FOV = number of degrees for camera view
      def __init__(self, src, height, width):
            self.camera = picamera2.Picamera2()
            self.width = width
            self.height = height

            # Configure the camera
            config = self.camera.create_video_configuration(main={"format": 'XRGB8888', "size": (height, width)})
            self.camera.configure(config)
            self.camera.set_controls({"ExposureTime": 70000, "AnalogueGain": 1,  "ColourGains": (1.4,1.5)}) 
            self.camera.start()

            self.cameraStopped = False
            self.prev_frame_id = -1
            self.frame_id = 0
            self.currentFrame = np.zeros((height, width, 3), np.uint8)
            self.currentFrame = self.camera.capture_array()

      def start(self):
            self.t1 = time.time()
            Thread(target=self.captureImage, args=()).start()
            return self

      def captureImage(self):
            # Continuously capture frames
            while True:
                  if self.cameraStopped:
                        return
                  # Capture current frame
                  self.currentFrame = self.camera.capture_array()
                  self.frame_id += 1

      def getCurrentFrame(self):
            self.imgFlip = cv2.resize(self.currentFrame, (410, 308))
            imgRGB = cv2.rotate(self.imgFlip, cv2.ROTATE_180)
            imgHSV = cv2.cvtColor(imgRGB, cv2.COLOR_BGR2HSV)  # Convert to HSV
            RobotView = imgRGB.copy()  # Preserve the original image
            return imgRGB, imgHSV, RobotView
      
      def getFrameID(self):
            return self.frame_id
      
      def DisplayFrame(self, frame_id, FPS=False, frame=None, frame1=None, frame2=None, frame3=None, frame4=None):
            if frame_id != self.prev_frame_id:
                  if FPS:
                        fps = 1.0 / (time.time() - self.t1)  # calculate frame rate
                        self.t1 = time.time()
                        cv2.putText(frame, f'{int(fps)}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 2)  # Display the FPS on the screen
                  
                  cv2.imshow('Frame', frame)
                  
                  if frame1 is not None:
                        cv2.imshow('Frame1', frame1)
                  if frame2 is not None:
                        cv2.imshow('Frame2', frame2)
                  if frame3 is not None:
                        cv2.imshow('Frame3', frame3)
                  if frame4 is not None:
                        cv2.imshow('Frame4', frame4)
                  
                  self.prev_frame_id = frame_id

            

      def stop(self):
            self.cameraStopped = True
            
      def __del__(self):
            # There is no release method in picamera2, so stop the camera instead
            self.camera.stop()
            cv2.destroyAllWindows()


def main():
      
      FRAME_WIDTH = 820
      FRAME_HEIGHT = 616
      cam = CamFrameGrabber(src=0, height=FRAME_WIDTH, width=FRAME_HEIGHT)
      cam.start()
      vision = vs.VisionModule()
      try:
            while True:
                  imgRGB, imgHSV, RobotView = cam.getCurrentFrame()
                  frame_id = cam.getFrameID()
                  CenterCoord = vision.draw_crosshair(RobotView)
                  
                  #Find contours for the shelves
                  contoursShelf, ShelfMask = vision.findShelf(imgHSV)
                  # Get the detected shelf centers
                  ShelfCenters = vision.GetContoursShelf(contoursShelf, RobotView, (0, 0, 255), "S", Draw=True)
                  ShelfCenter, ShelfBearing = vision.GetInfoShelf(RobotView, ShelfCenters, imgRGB)


                  # Detect obstacles in the HSV image
                  contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
                  # Get the list of detected obstacles' centers and dimensions
                  detected_obstacles = vision.GetContoursObject(contoursObstacle, RobotView, (0, 255, 255), "Obs", Draw=True)
                  ObsCenters, ObsDistance, ObsBearing = vision.GetInfoObject(RobotView, detected_obstacles, imgRGB)


                  
                  WallRGB,  WallImgGray, WallMask = vision.findWall(imgHSV,imgRGB)
                  ContoursMarkers, mask1 = vision.findMarkers(WallImgGray, WallMask)
                  avg_center, avg_distance, avg_center, shape_count = vision.GetInfoMarkers(RobotView, ContoursMarkers, imgRGB)


                  cam.DisplayFrame(frame_id, FPS=True, frame=RobotView)
                  # Break the loop if 'q' is pressed
                  if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

      except KeyboardInterrupt:
            # Handle when user interrupts the program
            print("Stopping camera capture...")

      # Stop the camera and close windows
      cam.stop()
      cv2.destroyAllWindows()


if __name__ == "__main__":
    
    main()
