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
      a = 2
      while(1):
            img,imgHSV,robotview = vision.Capturing()
            imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            #print(imgGray.shape)
            contoursItem, ItemMask = vision.findItems(imgHSV)
            contoursShelf, ShelfMask = vision.findShelf(imgHSV)
            #contoursLoadingArea, LoadingAreaMask = vision.findLoadingArea(imgHSV)
            contoursObstacle, ObstacleMask = vision.findObstacle(imgHSV)
            #contoursWall,WallMask = vision.findWall(imgHSV)
            a += 10
            i2c.LedWrite(0, "ON")  # Turn LED0 on
            print(a)
            i2c.ServoWrite(1, a)  # Turn Servo1 to 180 degrees
            if a >= 180:
                  a = 0
            #WallMask = ItemMask #There is no white wall at home so i use orange instead
            #result = cv2.bitwise_and(img, img, mask=WallMask)
            #contoursMarker, MarkerMask = vision.findMarkers(imgHSV) #search for markers on the wall
            
            xi1, yi1, xi2, yi2 = vision.GetContoursObject(contoursItem, robotview, (0, 255, 0), "Item", Draw = True)
            xs1, ys1, xs2, ys2 = vision.GetContoursShelf(contoursShelf, robotview, (0, 0, 255), "Shelf", Draw = True)
            #xl1, yl1, xl2, yl2 = vision.GetContoursObject(contoursLoadingArea, robotview, (255, 0, 0), "Loading Area", Draw = True)
            xo1, yo1, xo2, yo2 = vision.GetContoursObject(contoursObstacle, robotview, (0, 255, 255), "Obstacle", Draw = True)
            #xm1, ym1, xm2, ym2, radiusm, marker_count = vision.MarkersContours(contoursMarker, robotview, (255, 255, 0), "Marker", Draw = True)
            print(imgHSV[:,:,0])
            WallMask = (imgGray > 180).astype('uint8') * 255
            result = cv2.bitwise_and(img, img, mask=WallMask)
            #result = cv2.bitwise_and(img, img, mask=TestingMask)
            vision.ExportImage("RobotView", robotview, FPS = True)
            #vision.ExportImage("ItemMask", result, FPS = False)
            #vision.ExportImage("Marker", MarkerMask, FPS = False)
            if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
                  break
      cap.close()

if __name__ == "__main__": # Run the main function
      CSV = csvread.CSVReader('Order_1.csv')
      CSV.read_csv()
      instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions

      main()
  