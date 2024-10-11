import cv2
import picamera2
import numpy as np
from threading import Thread
import vision as vs
import time
import RP2040 as rp
from Vision2 import VisionModule
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
      VisionModule.calculate_projection_transform()
      try:
            def closest_point(contours, text):
                  contours = np.array(contours)
                  if contours.size > 0:
                        points = VisionModule.combine_contour_points(contours, exclude_horizontal_overlap=False)
                  
                        if points is not None:
                              closest_point_i = np.argmax(points[:, 1])
                              closest_point = points[closest_point_i, :]

                              projected = VisionModule.project_point_to_ground(np.array([closest_point]))[0]
                              if projected is not None:
                                    # Dist map processing
                                    dist = np.sqrt(projected[0]**2 + projected[1]**2)
                                    cv2.drawMarker(RobotView, (int(closest_point[0]), int(closest_point[1])), (0,0,255), cv2.MARKER_DIAMOND, 4)
                                    cv2.putText(RobotView, f"{text}: {dist*100:.2f}", (int(closest_point[0]), int(closest_point[1])-20),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
                                    return dist
                        
                  return None

            while True:

                  imgRGB, imgHSV, RobotView = cam.getCurrentFrame()
                  frame_id = cam.getFrameID()
                  CenterCoord = vision.draw_crosshair(RobotView)
                  
                  

                  #Find contours for the shelves
                  contoursShelf, ShelfMask = vision.findShelf(imgHSV)
                  # Get the detected shelf centers
                  ShelfCenters = vision.GetContoursShelf(contoursShelf, RobotView, (0, 0, 255), "S", Draw=True)
                  ShelfCenter, ShelfBearing = vision.GetInfoShelf(RobotView, ShelfCenters, imgRGB)
                  closest_point(contoursShelf, "Shelf")

                  

                  contoursLoadingBay, LoadingBayMask = vision.findLoadingArea(imgHSV)
                  LoadingBayCenters = vision.GetContoursShelf(contoursLoadingBay, RobotView, (0, 255, 0), "L", Draw=True)
                  LoadingBayCenter, LoadingBayBearing = vision.GetInfoShelf(RobotView, LoadingBayCenters, imgRGB)
                  closest_point(contoursLoadingBay, "Packing")

                  # Detect obstacles in the HSV image
                  contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
                  # Get the list of detected obstacles' centers and dimensions
                  detected_obstacles = vision.GetContoursObject(contoursObstacle, RobotView, (0, 255, 255), "Obs", Draw=True)
                  ObsCenters, ObsDistance, ObsBearing = vision.GetInfoObject(RobotView, detected_obstacles, imgRGB)


                  
                  WallRGB,  WallImgGray, WallMask,contoursWall1, GrayScale_Image = vision.findWall(imgHSV,imgRGB)
                  ContoursMarkers, mask1 = vision.findMarkers(WallImgGray, WallMask)
                  avg_center, avg_bearing, avg_distance, shape_count = vision.GetInfoMarkers(RobotView, ContoursMarkers, imgRGB)
                  
                  closest_point(contoursWall1, "Wall")

                  # gray = cv2.cvtColor(imgRGB, cv2.COLOR_BGR2GRAY)
                  # ret, WM = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)
                  # inverted_loading_bay_mask = cv2.bitwise_not(LoadingBayMask)
                  # masked_wall = cv2.bitwise_and(WM, inverted_loading_bay_mask)
                  
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