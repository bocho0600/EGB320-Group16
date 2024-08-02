import io
import time
from picamera2 import Picamera2, Preview
from base_camera import BaseCamera
import cv2
import numpy as np



class Camera(BaseCamera):
    
    color_ranges = {
        'wall': (np.array([39, 0, 0]), np.array([162, 255, 255])),
        'yellow': (np.array([9, 85, 0]), np.array([19  , 255, 255])),
        'blue': (np.array([39, 0, 0]), np.array([162, 255, 255])),
        'green': (np.array([33, 0, 0]), np.array([94, 255, 255])),
        'orange1': (np.array([5, 150, 150]), np.array([20, 255, 255])),
        'orange2': (np.array([165, 150, 150]), np.array([180, 255, 255])),
        'black': (np.array([0, 0, 43]), np.array([179, 55, 109]))
    }
    

    @staticmethod
    def frames():
        with Picamera2() as camera:
            camera.start()

            # let camera warm up
            time.sleep(2) 

            pTime = 0.
            cTime = 0.
            try:
                while True:
                    img = cv2.rotate(cv2.cvtColor(camera.capture_array('main'), cv2.COLOR_BGR2RGB), cv2.ROTATE_180)

                    org = img.copy() # preserve the original image
                    
                    cTime = time.time() # Current time for setting FPS
                    fps = 1./(cTime - pTime)
                    pTime = cTime
                    # Convert the image to HSV
                    imgHSV = cv2.cvtColor(org, cv2.COLOR_BGR2HSV)


                    # Create masks for the orange color
                    mask1 = cv2.inRange(imgHSV, Camera.color_ranges['orange1'][0], Camera.color_ranges['orange1'][1])
                    mask2 = cv2.inRange(imgHSV, Camera.color_ranges['orange2'][0], Camera.color_ranges['orange2'][1])
                    ItemMask = mask1 | mask2  # Combine masks

                    # Apply the mask to get the filtered color
                    result = cv2.bitwise_and(org, org, mask=ItemMask)

                    # Find contours in the mask
                    contours, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    #cv2.drawContours(result, contours, -1, (0, 255, 0), 2)
                    
                    for contour in contours:  # Draw bounding rectangles around detected contours
                        if cv2.contourArea(contour) > 500:  # Filter out small contours
                            print("Item detected")
                            x, y, width, height = cv2.boundingRect(contour) # Take the bounding rectangle of the contour
                            cv2.rectangle(img, (x, y), (x + width, y + height), (0, 255, 0), 2)  # Draw a rectangle
                            cv2.putText(img, "Item", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Display the original image and the filtered image with rectangles
                    cv2.putText(img, f'{fps:.1f}', (20,30), cv2.FONT_HERSHEY_TRIPLEX ,0.7,(255,0,100),2) # Display the FPS on the screen
                    # cv2.imshow("Robot View", img)
                    # cv2.imshow("Filtered Image", result)
                    # cv2.imshow("Original Image", org)

                    
                    yield cv2.imencode('.jpg', img)[1].tobytes()
                    
                    #time.sleep(1)


            finally:
                camera.stop()
