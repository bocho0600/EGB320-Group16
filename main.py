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

            contoursItem, ItemMask = vision.findItems(imgHSV)
            xi1, yi1, xi2, yi2 = vision.GetContoursObject(contoursItem, robotview, (0, 255, 0), "Item", Draw = True)

            contoursShelf, ShelfMask = vision.findShelf(imgHSV)
            xs1, ys1, xs2, ys2 = vision.GetContoursShelf(contoursShelf, robotview, (0, 0, 255), "Shelf", Draw = True)

            #contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
            #xo1, yo1, xo2, yo2 = vision.GetContoursObject(contoursObstacle, robotview, (0, 255, 255), "Obstacle", Draw = True)
            BlackContours, BlackMask = cv2.threshold(imgGray, 120, 255, cv2.THRESH_BINARY_INV) # Apply thresholding to get white colour filter
            BlackMask = BlackMask - ShelfMask - ItemMask
            contoursMarkers, MarkerMask = cv2.findContours(BlackMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contoursMarkers:
                  if cv2.contourArea(contour)>2000 and cv2.contourArea(contour)<15000:  # Skip small contours
                        epsilon = 0.03 * cv2.arcLength(contour, True)  # Calculate the perimeter of the contour
                        
                        # Approximate the contour
                        ShapeContours = cv2.approxPolyDP(contour, epsilon, True)
                        # Determine the number of vertices
                        num_vertices = len(ShapeContours)
                        # Identify the shape based on the number of vertices
                        if num_vertices == 3:
                              shape = "Triangle"
                        elif num_vertices == 4:
                              shape = "Square"
                        elif num_vertices > 4 and num_vertices < 12:
                              shape = "Circle"  # Assuming more than 4 sides is a circle for simplicity
                        else:
                              shape = "Unknown"

                        # Only recognize squares and circles
                        if shape in ["Square", "Circle"]:
                              cv2.drawContours(robotview, [ShapeContours], -1, [123,123,123], 3)
                              x, y = ShapeContours.ravel()[0], ShapeContours.ravel()[1]
                              cv2.putText(robotview, shape, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)



            #WallMask, contoursMarkers, WallImage = vision.findWall(imgGray, img)
            #detected_shapes, shape_count,shape = vision.MarkersDetection(contoursMarkers, robotview, (0, 255, 0), Draw=True)
   
  

            vision.ExportImage("RobotView", robotview, FPS = True)
            vision.ExportImage("WallMask", BlackMask, FPS = True)

            if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
                  break
      cap.close()

if __name__ == "__main__": # Run the main function
      CSV = csvread.CSVReader('Order_1.csv')
      CSV.read_csv()
      instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions

      main()
