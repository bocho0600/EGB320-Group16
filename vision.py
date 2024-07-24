import cv2
import picamera2
import numpy as np
import time

# MADE BY KELVIN LE, QUT EGB320 GROUP16 
# StudentID: n11429984
# Email: minhnguyen.le@qut.edu.au

class VisionModule:
    color_ranges = {
        'wall': (np.array([39, 0, 0]), np.array([162, 255, 255])),
        'yellow': (np.array([20, 120, 153]), np.array([25, 233, 218])),
        'blue': (np.array([90, 136, 9]), np.array([120, 255, 94])),
        'green': (np.array([55, 7, 38]), np.array([100, 137, 88])),
        'orange1': (np.array([0, 158, 45]), np.array([13, 255, 235])),
        'orange2': (np.array([165, 150, 150]), np.array([180, 255, 255])),
        'black': (np.array([0, 0, 43]), np.array([179, 55, 109]))
    }

    def __init__(self):
        self.cap = None  # Initialize the camera object as an instance variable
        self.t1 = None

    def CaptureImage(self):
        frame = self.cap.capture_array()  # Use the instance variable
        frame = cv2.flip(frame, 0)  # OPTIONAL: Flip the image vertically
        return frame

    def initialize_camera(self, frame_height=320*2, frame_width=240*2, format='XRGB8888'):
        # Create a camera object and store it as an instance variable
        self.cap = picamera2.Picamera2()
        config = self.cap.create_video_configuration(main={"format": format, "size": (frame_height, frame_width)})
        self.cap.configure(config)
        self.cap.start()

    def Capturing(self):
        kernel = np.ones((2, 2), np.uint8)
        self.t1 = time.time()  # For measuring FPS
        img = self.CaptureImage()  # Capture a single image frame
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
        imgHSV = cv2.erode(imgHSV, kernel, iterations=1)
        imgHSV = cv2.dilate(imgHSV, kernel, iterations=1)
        robotview = img.copy()  # Preserve the original image
        return img, imgHSV, robotview, self.t1

    def ExportImage(self, WindowName, view , FPS = False):
        if FPS:
            fps = 1.0/(time.time() - self.t1)         # calculate frame rate
            cv2.putText(view, f'{int(fps)}', (20,30), cv2.FONT_HERSHEY_SIMPLEX ,0.5,(255,255,100), 2) # Display the FPS on the screen

        cv2.imshow(WindowName, view)
        
    def findItems(self, imgHSV):
        # Create masks for the orange color
        ItemMask1 = cv2.inRange(imgHSV, self.color_ranges['orange1'][0], self.color_ranges['orange1'][1])
        ItemMask2 = cv2.inRange(imgHSV, self.color_ranges['orange2'][0], self.color_ranges['orange2'][1])
        ItemMask = ItemMask1 | ItemMask2  # Combine masks
        contoursItem, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursItem, ItemMask

    def findShelf(self, imgHSV):
        ShelfMask = cv2.inRange(imgHSV, self.color_ranges['blue'][0], self.color_ranges['blue'][1])
        contoursShelf, _ = cv2.findContours(ShelfMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursShelf, ShelfMask

    def findLoadingArea(self, imgHSV):
        LoadingAreaMask = cv2.inRange(imgHSV, self.color_ranges['yellow'][0], self.color_ranges['yellow'][1])
        contoursLoadingArea, _ = cv2.findContours(LoadingAreaMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursLoadingArea, LoadingAreaMask

    def findObstacle(self, imgHSV):
        ObstacleMask = cv2.inRange(imgHSV, self.color_ranges['green'][0], self.color_ranges['green'][1])
        contoursObstacle, _ = cv2.findContours(ObstacleMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursObstacle, ObstacleMask

    def GetContours(self, contours, output, colour, text, Draw = True):
        detected = False
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                x, y, width, height = cv2.boundingRect(contour)
                if Draw:
                    cv2.rectangle(output, (x, y), (x + width, y + height), colour, 2)
                    cv2.putText(output, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                detected = True
        if detected:
            x1, y1, x2, y2 = x, y, x + width, y + height
            return x1, y1, x2, y2
        else:
            return  None, None, None, None
