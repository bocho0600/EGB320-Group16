import cv2
import numpy as np
import time
import vision as vision
import csv 
import csvread as csvread

def main(): # Main function
      vision = vision.VisionModule()
      cap = vision.initialize_camera()

      while(1):
            print(instruction)

            img, robotview, t1 = vision.StartCapturing()
            
            contoursItem, ItemMask = vision.findItems(img)
            contoursShelf, ShelfMask = vision.findShelf(img)
            contoursLoadingArea, LoadingAreaMask = vision.findLoadingArea(img)
            Mask = ItemMask | ShelfMask
            robotview, x1, y1, x2, y2 = vision.DrawContours(contoursItem, robotview, (0, 255, 0), "Item")
            robotview, x1, y1, x2, y2 = vision.DrawContours(contoursShelf, robotview, (0, 0, 255), "Shelf")
            result = cv2.bitwise_and(robotview, robotview, mask=Mask)

            calculate_and_display_fps(robotview, t1)

            cv2.imshow("RobotView", robotview)     # Display the obtained frame in a window called "CameraImage"
            if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
                  break
                  
      cap.close()

if __name__ == "__main__": # Run the main function
      CSV = CSVReader('Order_1.csv')
      CSV.read_csv()
      instruction = CSV.RobotInstruction() # Generating robot instructions and print instructions
      main()
