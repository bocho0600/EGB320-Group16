import cv2
import numpy as np
import time
import vision as vs
import RP2040 as I2C
import csv 
import csvread as csvread
import picamera2
from sys import argv

def main(): # Main function
      i2c = I2C.I2C()
      vision = vs.VisionModule()
      cap = vision.initialize_camera()
      while(1):
            img,imgHSV,robotview = vision.Capturing()
            imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            #contoursItem, ItemMask = vision.findItems(imgHSV)
            #xi1, yi1, xi2, yi2 = vision.GetContoursObject(contoursItem, robotview, (0, 255, 0), "Item", Draw = True)

            #contoursShelf, ShelfMask = vision.findShelf(imgHSV)
            #xs1, ys1, xs2, ys2 = vision.GetContoursShelf(contoursShelf, robotview, (0, 0, 255), "Shelf", Draw = True)

            #contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
            #xo1, yo1, xo2, yo2 = vision.GetContoursObject(contoursObstacle, robotview, (0, 255, 255), "Obstacle", Draw = True)

            WallMask, contoursMarkers, WallImage = vision.findWall(imgGray, img)
            detected_shapes, shape_count,shape = vision.MarkersDetection(contoursMarkers, robotview, (0, 255, 0), Draw=True)
            print(shape)
            if shape_count > 0:
                  print("Detected ", shape_count) 
            vision.ExportImage("RobotView", robotview, FPS = True)
            vision.ExportImage("WallMask", WallImage, FPS = True)

            if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
                  break
      cap.close()

if __name__ == "__main__": # Run the main function
      CSV = csvread.CSVReader('Order_1.csv')
      CSV.read_csv()
      instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions

      main()
