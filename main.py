import cv2
import numpy as np
import time
import vision as vision

def main(): # Main function
      vision = vision.VisionModule()
      cap = vision.initialize_camera()
      
      while(1):

            t1 = time.time()                     # for measuring fps
            
            img = vision.CaptureImage()          # capture a single image frame (should not modify in advance)
            robotview = img.copy() # preserve the original image
            
            contoursItem, ItemMask = vision.findItems(img)
            contoursShelf, ShelfMask = vision.findShelf(img)
            contoursLoadingArea, LoadingAreaMask = vision.findLoadingArea(img)
            
            robotview, ix1, iy1, ix2, iy2 = vision.DrawContours(contoursItem, robotview, (0, 255, 0), "Item")
            robotview, sx1, sy1, sx2, sy2 = vision.DrawContours(contoursShelf, robotview, (0, 0, 255), "Shelf")
            
            fps = 1.0/(time.time() - t1)         # calculate frame rate
            cv2.putText(robotview, f'{int(fps)}', (20,30), cv2.FONT_HERSHEY_TRIPLEX ,0.7,(255,255,100),2) # Display the FPS on the screen

            cv2.imshow("RobotView", robotview)     # Display the obtained frame in a window called "CameraImage"
            if cv2.waitKey(1) & 0xFF == ord('q'): # Press 'q' to quit
                  break
      
      cap.close()

if __name__ == "__main__": # Run the main function
    main()
