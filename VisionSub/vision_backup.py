import cv2
import picamera2
import numpy as np
from threading import Thread
import vision as vs
import time
import RP2040 as rp

# Servo S3 is 6 kg/cm3
# Servo S2 is 3 kg/cm3
ground_points = np.float32([[-18,47],[18,47],[-18,150],[18,150]]) #off-set in cm
homography_matrix = np.array([[-2.11252555e-02,  6.33812791e-03,  7.01814123e+00],[ 1.80590083e-04, -8.03575074e-03, -1.43661305e+01]
 ,[ 2.56484473e-05, -4.57580665e-03,  1.00000000e+00]])

def main():
      i2c = rp.I2C()
      i2c.ServoSet(3,0)
      FRAME_WIDTH = 820
      FRAME_HEIGHT = 616
      cam = vs.CamFrameGrabber(src=0, height=FRAME_WIDTH, width=FRAME_HEIGHT)
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
                  closest_point = None
                  max_y = -1

                  # Iterate through each contour
                  for contour in contoursShelf:
                        for point in contour:
                              x, y = point[0]  # Each point is a nested array, so extract x and y
                              if y > max_y:    # If this point is closer to the bottom
                                    max_y = y    # Update max_y to the current point's y
                                    closest_point = (x, y)  # Save the point
                  if closest_point:
                        cv2.circle(RobotView, closest_point, 5, (0, 0, 255), -1)  # Red circle with radius 5
                        pred_point = cv2.perspectiveTransform(np.float32(closest_point).reshape(-1,1,2), homography_matrix)
                        real_points = 100 + pred_point[0][0][1]
                        print(real_points)

                  contoursLoadingBay, LoadingBayMask = vision.findLoadingArea(imgHSV)
                  LoadingBayCenters = vision.GetContoursShelf(contoursLoadingBay, RobotView, (0, 255, 0), "L", Draw=True)
                  LoadingBayCenter, LoadingBayBearing = vision.GetInfoShelf(RobotView, LoadingBayCenters, imgRGB)





                  # Detect obstacles in the HSV image
                  contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
                  # Get the list of detected obstacles' centers and dimensions
                  detected_obstacles = vision.GetContoursObject(contoursObstacle, RobotView, (0, 255, 255), "Obs", Draw=True)
                  ObsCenters, ObsDistance, ObsBearing = vision.GetInfoObject(RobotView, detected_obstacles, imgRGB)


                  
                  WallRGB,  WallImgGray, WallMask,contoursWall1 = vision.findWall(imgHSV,imgRGB)
                  ContoursMarkers, mask1 = vision.findMarkers(WallImgGray, WallMask)
                  avg_center, avg_bearing, avg_distance, shape_count = vision.GetInfoMarkers(RobotView, ContoursMarkers, imgRGB)


                  cam.DisplayFrame(frame_id, FPS=True, frame=RobotView, frame1 = WallMask) # Display the frame with the detected objects.
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