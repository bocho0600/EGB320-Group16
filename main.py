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
            #print(img.shape)
            #contoursItem, ItemMask = vision.findItems(imgHSV)
            #xi1, yi1, xi2, yi2 = vision.GetContoursObject(contoursItem, robotview, (0, 255, 0), "Item", Draw = True)

            contoursShelf, ShelfMask = vision.findShelf(imgHSV)
            xs1, ys1, xs2, ys2 = vision.GetContoursShelf(contoursShelf, robotview, (0, 0, 255), "Shelf", Draw = True)

            contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
            xo1, yo1, xo2, yo2 = vision.GetContoursObject(contoursObstacle, robotview, (0, 255, 255), "Obstacle", Draw = True)

            contoursLoading, LoadingMask = vision.findLoadingArea(imgHSV)
            xl1, yl1, xl2, yl2 = vision.GetContoursShelf(contoursLoading, robotview, (120, 120, 255), "LoadingArea", Draw = True)

            BlackMask = vision.findBlack(imgHSV)
            BlackMask = BlackMask 
            #BlackContours, BlackMask = cv2.threshold(imgGray, 150, 255, cv2.THRESH_BINARY_INV) # Apply thresholding to get white colour filter
            #BlackContours, BlackMask = cv2.threshold(imgGray, 160, 255, cv2.THRESH_BINARY) # Apply thresholding to get white colour filter
            #BlackMask = BlackMask - ObstacleMask# -ShelfMask
            #BlackMask = cv2.bitwise_and(BlackMask,BlackMask12)
            contoursMarkers, MarkerMask = cv2.findContours(BlackMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            focal_length = 0 #cm
            real_circle_diameter = 70 #cm
            #distance = 100 #cm
            circle_count = 0  # Initialize circle counter
            for contour in contoursMarkers:
                  if cv2.contourArea(contour)>300 and cv2.contourArea(contour)<50000:  # Skip small contours
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

                              # Get the bounding rectangle for the square
                              x, y, w, h = cv2.boundingRect(contour)
                              aspect_ratio = float(w) / h

                              if 0 < aspect_ratio < 3:  # Check if it's a square (aspect ratio close to 1)
                                    # Draw the square's bounding box on the robotview
                                    cv2.drawContours(robotview, [ShapeContours], -1, (0, 0, 255), 3)

                                    # Display the width of the square marker on the image
                                    cv2.putText(robotview, f"Width: {w} px", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)
                              
                        elif num_vertices > 4 and num_vertices < 12:
                              shape = "Circle"  # Assuming more than 4 sides is a circle for simplicity
                              # Find the minimum enclosing circle and calculate diameter
                              (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
                              diameter = 2 * radius  # Diameter is twice the radius
                              distance = vision.GetDistance(diameter,70)
                              
                              
                              offset_pixels = x_center - 820/2
                              angle = (offset_pixels / 820) * 22.3
                              circle_count += 1 
                              # Draw the circle
                              cv2.circle(robotview, (int(x_center), int(y_center)), int(radius), (0, 255, 0), 2)
                              # Add text for diameter
                              cv2.circle(robotview, (int(820/2), int(616/2)), int(22), (255, 0, 0), 2)
                              #cv2.putText(robotview, f"Distance: {int(distance)}", (int(x_center), int(y_center)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)
                              cv2.putText(robotview, f"distance: {int(distance)}", (int(x_center), int(y_center)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)
                        else:
                              shape = "Unknown"

                        # Only recognize squares and circles
                        if shape in ["Circle"]:
                              cv2.drawContours(robotview, [ShapeContours], -1, [123,123,123], 3)
                              x, y = ShapeContours.ravel()[0], ShapeContours.ravel()[1]
                              cv2.putText(robotview, shape, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)

            #print("detected ", circle_count," amount of ",shape)

            #WallMask, contoursMarkers, WallImage = vision.findWall(imgGray, img)
            #detected_shapes, shape_count,shape = vision.MarkersDetection(contoursMarkers, robotview, (0, 255, 0), Draw=True)
   
  

            vision.ExportImage("RobotView", robotview, FPS = True)
            vision.ExportImage("BlackMask", BlackMask, FPS = True)
            #vision.ExportImage("BlueMask", ShelfMask, FPS = True)

            if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
                  break
      cap.close()

if __name__ == "__main__": # Run the main function
      CSV = csvread.CSVReader('Order_1.csv')
      CSV.read_csv()
      instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions

      main()
