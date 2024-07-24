import cv2
import numpy as np
import time
from G16Modules.Globals import *
from G16Modules.Vision import VisionModule, CamFrameGrabber
#from G16Modules.Navigation import NavigationModule, STATE
import RP2040 as I2C
import csvread as csvread


def main(): # Main function
      #i2c = I2C.I2C()
      cap = Specific.initialize_camera()
      cam = CamFrameGrabber(SCREEN_WIDTH, SCREEN_HEIGHT)
      VisionModule.setGrabber(cam)

      cam.start()
      try:
            while(1):
                  img, imgHSV = VisionModule.Capturing()
                  robotview = img
                  print(f"\n ===============    {int(VisionModule.fps)}, {int(cam.fps)} ==================")
                  
                  
                  contoursShelf, ShelfMask = VisionModule.findShelf(imgHSV)
                  xs1, ys1, xs2, ys2 = VisionModule.GetContoursShelf(contoursShelf, robotview, (0, 0, 255), "Shelf", Draw = False)

                  contoursObstacle, ObstacleMask = VisionModule.findObstacle(imgHSV)
                  x_ObstacleCenter, y_ObstacleCenter  = VisionModule.GetContoursObject(contoursObstacle, robotview, (0, 255, 255), "Obstacle", Draw = False)
                  if x_ObstacleCenter != None:
                        ObstacleAngle = VisionModule.GetBearing(x_ObstacleCenter)
                  
                  
                  contoursLoading, LoadingMask = VisionModule.findLoadingArea(imgHSV)
                  xl1, yl1, xl2, yl2 = VisionModule.GetContoursShelf(contoursLoading, robotview, (120, 120, 255), "LoadingArea", Draw = False)

                  contoursMarkers, MarkerMask = VisionModule.findMarkers(imgHSV)
                  shapeCount, distances, bearings, xs, ys = VisionModule.MarkerShapeDetection(contoursMarkers, robotview, Draw = False)
                  aisleNumber, distance, bearing, x_center, y_center = VisionModule.ProcessAisleMarkers(shapeCount, distances, bearings, xs, ys)

                  VisionModule.ExportImage("RobotView", img, FPS = True)

                  if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
                        break
      finally:
            cam.stop()
            Specific.end()

if __name__ == "__main__": # Run the main function
      CSV = csvread.CSVReader('Order_1.csv')
      CSV.read_csv()
      instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions

      main()
