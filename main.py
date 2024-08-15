import cv2
import numpy as np
import time
import vision as vs
import csv 
import csvread as csvread
import picamera2
from sys import argv

def main(): # Main function
      vision = vs.VisionModule()
      cap = vision.initialize_camera()

      while(1):
            img,imgHSV,robotview, t1 = vision.Capturing()
            
            contoursItem, ItemMask = vision.findItems(imgHSV)
            contoursShelf, ShelfMask = vision.findShelf(imgHSV)
            contoursLoadingArea, LoadingAreaMask = vision.findLoadingArea(imgHSV)
            contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)

            Mask = ItemMask | ShelfMask
            xi1, yi1, xi2, yi2 = vision.GetContours(contoursItem, robotview, (0, 255, 0), "Item", Draw = True)
            xs1, ys1, xs2, ys2 = vision.GetContours(contoursShelf, robotview, (0, 0, 255), "Shelf", Draw = True)
            xl1, yl1, xl2, yl2 = vision.GetContours(contoursLoadingArea, robotview, (255, 0, 0), "Loading Area", Draw = True)
            xo1, yo1, xo2, yo2 = vision.GetContours(contoursObstacle, robotview, (0, 255, 255), "Obstacle", Draw = True)
            
            #result = cv2.bitwise_and(robotview, robotview, mask=Mask)

            #vision.ExportImage("Masking", result, FPS = False)
            vision.ExportImage("RobotView", robotview, FPS = True)
            if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
                  break
                  
      cap.close()

if __name__ == "__main__": # Run the main function
      CSV = csvread.CSVReader('Order_1.csv')
      CSV.read_csv()
      instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions

      main()
  