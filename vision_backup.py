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

                  # If any shelves were detected
                  if ShelfCenters is not None:
                  # Loop through each detected shelf center
                        for ShelfCenter in ShelfCenters:
                              x_center, y_center = ShelfCenter
                              
                              # Calculate the bearing (angle) for each shelf
                              ShelfAngle = vision.GetBearing(x_center, imgRGB)
                              
                              # Display the angle on the image
                              cv2.putText(RobotView, f"A: {int(ShelfAngle)}", (int(x_center), int(y_center)), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 200), 1)

                  # Detect obstacles in the HSV image
                  contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)

                  # Get the list of detected obstacles' centers and dimensions
                  detected_obstacles = vision.GetContoursObject(contoursObstacle, RobotView, (0, 255, 255), "Obs", Draw=True)

                  # Check if any obstacles were detected
                  if detected_obstacles is not None:
                  # Loop through each detected obstacle and process it
                        for obstacle in detected_obstacles:
                              x_ObstacleCenter, y_ObstacleCenter, ObHeight, ObWidth = obstacle
                              
                              # Calculate the obstacle's angle and distance
                              ObstacleAngle = vision.GetBearing(x_ObstacleCenter, imgRGB)
                              ObstacleDistance = vision.GetDistance(ObHeight, 150)

                              # Add the angle and distance information to the image
                              cv2.putText(RobotView, f"A: {int(ObstacleAngle)} deg", (int(x_ObstacleCenter), int(y_ObstacleCenter + ObHeight / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
                              cv2.putText(RobotView, f"D: {int(ObstacleDistance)} cm", (int(x_ObstacleCenter), int(y_ObstacleCenter)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)

                  
                  # Assuming contoursMarker is obtained from some previous processing step
                  
                  WallRGB,  WallImgGray, WallMask = vision.findWall(imgHSV,imgRGB)
                  ContoursMarkers, mask1 = vision.findMarkers(WallImgGray, WallMask)
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
                                    MarkerAngle = vision.GetBearing(x, imgRGB)
                                    MarkerDistance = vision.GetDistance(radius * 2, 70)
                                    cv2.putText(RobotView, f"A: {int(MarkerAngle)} deg", (int(x), int(y + radius / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (237, 110, 255), 1)
                                    cv2.putText(RobotView, f"D: {int(MarkerDistance)} cm", (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 1)
                                    cv2.circle(RobotView, center, int(radius), (255, 255), 2)
                                    cv2.drawMarker(RobotView, center, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=5, thickness=2)
                                    cv2.putText(RobotView, f"M", (int(x - 5), int(y + radius/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                        

                  
                  #cam.DisplayFrame(frame_id, FPS=True, frame=RobotView, frame1 = WallImgGray, frame2 = WallMask, frame3 = markers, frame4 = mask1)
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
