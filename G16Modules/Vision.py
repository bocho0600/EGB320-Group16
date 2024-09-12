import cv2
import picamera2
import numpy as np
import time
from threading import Thread, Lock, Condition

from .Globals import *
#from threading import Thread
# MADE BY KELVIN LE, QUT EGB320 GROUP16 
# StudentID: n11429984
# Email: minhnguyen.le@qut.edu.au

class VisionModule:
    color_ranges = {
        'wall': (np.array([39, 0, 0]), np.array([162, 255, 255])),
        'floor': (np.array([0, 0, 0]), np.array([179, 255, 255])),
        'yellow': (np.array([25, 90, 0]), np.array([36, 233, 255])),
        'blue': (np.array([81, 0, 0]), np.array([116, 255, 255])),
        'green': (np.array([55, 79, 0]), np.array([70, 255, 255])),
        'orange1': (np.array([0, 100, 0]), np.array([20, 255, 255])),
        'orange2': (np.array([165, 100, 0]), np.array([180, 255, 255])),
        'black': (np.array([29,53, 0]), np.array([51, 103, 81]))
    }


    focal_length = 70 #cm
    real_circle_diameter = 70 #cm

 
    cap = None  # Initialize the camera object as an instance variable
    t1 = time.time()
    fps = 0

    grabber = None

    @classmethod
    def setGrabber(cls, grabber):
        cls.grabber = grabber

    @classmethod
    def Capturing(cls):

        frame = cls.grabber.getCurrentFrame()  # Capture a single image frame

        t2 = time.time()
        cls.fps = 1.0 / (t2-cls.t1)
        cls.t1 = t2

        img, imgHSV = Specific.ConvertImage(frame)
#        robotview = img.copy()  # Preserve the original image

        return img, imgHSV #, robotview

    @classmethod
    def ExportImage(cls, WindowName, view, FPS=False):
        if FPS:
            cv2.putText(view, f'{int(cls.fps)}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 2)  # Display the FPS on the screen
        
        cv2.imshow(WindowName, view)

    @classmethod
    def findBlack(cls, imgHSV):
        # Create masks for the orange color
        #HSV90 = cv2.rotate(imgHSV, cv2.ROTATE_180)
        BlackMask = cv2.inRange(imgHSV, cls.color_ranges['black'][0], cls.color_ranges['black'][1])
        #contoursBlack, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return BlackMask

    @classmethod
    def findItems(cls, imgHSV):
        # Create masks for the orange color
        #HSV90 = cv2.rotate(imgHSV, cv2.ROTATE_180)
        ItemMask1 = cv2.inRange(imgHSV, cls.color_ranges['orange1'][0], cls.color_ranges['orange1'][1])
        ItemMask2 = cv2.inRange(imgHSV, cls.color_ranges['orange2'][0], cls.color_ranges['orange2'][1])
        ItemMask = ItemMask1 | ItemMask2  # Combine masks
        contoursItem, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursItem, ItemMask

    @classmethod
    def findShelf(cls, imgHSV, area_threshold=10000):
        # Create a mask for the blue color range
        ShelfMask = cv2.inRange(imgHSV, cls.color_ranges['blue'][0], cls.color_ranges['blue'][1])
        
        # Find contours on the mask
        contoursShelf, _ = cv2.findContours(ShelfMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter out small contours by area
        filtered_contours = [cnt for cnt in contoursShelf if cv2.contourArea(cnt) > area_threshold]
        
        return contoursShelf, ShelfMask

    @classmethod
    def findLoadingArea(cls, imgHSV):
        LoadingAreaMask = cv2.inRange(imgHSV, cls.color_ranges['yellow'][0], cls.color_ranges['yellow'][1])
        contoursLoadingArea, _ = cv2.findContours(LoadingAreaMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursLoadingArea, LoadingAreaMask

    @classmethod
    def findObstacle(cls, imgHSV):
        ObstacleMask = cv2.inRange(imgHSV, cls.color_ranges['green'][0], cls.color_ranges['green'][1])
        contoursObstacle, _ = cv2.findContours(ObstacleMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursObstacle, ObstacleMask
    
    @classmethod
    def findWall(cls, imgGray, imgRGB):
        _, WhiteMask = cv2.threshold(imgGray, 165, 255, cv2.THRESH_BINARY) # Apply thresholding to get white colour filter
        WallMask = np.zeros_like(imgGray) # Create an empty mask for the wall
        contours, _ = cv2.findContours(WhiteMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 300 :
                cv2.drawContours(WallMask, [contour], -1, 255, thickness=cv2.FILLED)
        # Extract the wall region (WILL NEED TO BE ADJUSTED)
        WallImage = cv2.bitwise_and(imgRGB, imgRGB, mask=WallMask)
        WallImage = cv2.cvtColor(WallImage, cv2.COLOR_BGR2GRAY)
        _, MarkerMask = cv2.threshold(imgGray, 120, 255, cv2.THRESH_BINARY_INV)
        # Find contours in the MarkerMask
        contoursMarkers, _ = cv2.findContours(WallImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return WallMask, contoursMarkers, WallImage

    @classmethod
    def findMarkers(cls,imgHSV):
        BlackMask = cv2.inRange(imgHSV, cls.color_ranges['black'][0], cls.color_ranges['black'][1])
        contoursMarkers, MarkerMask = cv2.findContours(BlackMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contoursMarkers, MarkerMask




    @classmethod
    def MarkerShapeDetection(cls, contoursMarkers, output, Draw = True):
        detected = False
        shapeCount = 0
        distances = []  # List to store distances for all detected circles
        bearings = []
        xs = []
        ys = []
        for contour in contoursMarkers:
            if cv2.contourArea(contour) > 300 and cv2.contourArea(contour) < 50000:
                epsilon = 0.03 * cv2.arcLength(contour, True)
                ShapeContours = cv2.approxPolyDP(contour, epsilon, True)
                num_vertices = len(ShapeContours)
                if num_vertices == 4:
                    shape = "Square"
                elif num_vertices in [4, 12]:
                    shape = "Circle"
                    shapeCount += 1
                    (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
                    diameter = 2 * radius
                    distance = cls.GetDistance(diameter, 70)
                    bearing = cls.GetBearing(x_center)

                    distances.append(distance)  # Store each distance
                    bearings.append(bearing)
                    xs.append(x_center)
                    ys.append(y_center)
                    
                    if Draw:
                        cv2.putText(output, f"Distance: {distance} cm", (x_center, y_center), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (123, 200, 100), 2)
                        cv2.putText(output, f"Circle: {shapeCount}", (int(x_center), int(y_center)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)
                        cv2.circle(output, (int(x_center), int(y_center)), int(radius), (0, 255, 0), 2)
                    detected = True

        if detected:
            return shapeCount, distances, bearings, xs, ys  # Return list of distances
        else:
            return None, [], [], [], []  # Return empty list if no circles detected

    @classmethod
    def ProcessAisleMarkers(cls, shapeCount, distances, bearings, xs, ys):
        # return aisle, distance, bearing
        # detected_shapes if a list of tuples (ShapeContours, shape), shape in ["Square", "Circle"]
        if shapeCount == 0:
            return 0, None, None, None, None
                
        distance = np.array(distances).mean()
        bearing = np.array(bearings).mean()
        x_center = np.array(xs).mean()
        y_center = np.array(ys).mean()

        return shapeCount, distance, bearing, x_center, y_center
  

    @classmethod
    def GetContoursShelf(cls, contours, output, colour, text, Draw=True):
        detected = False
        center = (0, 0)
        radius = 0
        for contour in contours:
            if cv2.contourArea(contour) > 1000:
                # Calculate the contour's center using moments
                M = cv2.moments(contour)
                if M['m00'] != 0:  # Avoid division by zero
                    center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                else:
                    center = (0, 0)
                
                if Draw:
                    # Draw the contour
                    cv2.drawContours(output, [contour], -1, colour, 2)
                    
                    # Draw the text at the center of the contour
                    cv2.putText(output, text, center, 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                detected = True
                # If you want to calculate a bounding box
                x, y, w, h = cv2.boundingRect(contour)
                x1, y1, x2, y2 = x, y, x + w, y + h
            else:
                x1, y1, x2, y2 = None, None, None, None

        if detected:
            return x1, y1, x2, y2
        else:
            return None, None, None, None
        
    @classmethod
    def GetContoursObject(cls, contours, output, colour, text, Draw = True):
        detected = False
        for contour in contours:
            if cv2.contourArea(contour) > 500:
                x, y, width, height = cv2.boundingRect(contour)
                if Draw:
                    cv2.rectangle(output, (x, y), (x + width, y + height), colour, 2)
                    cv2.putText(output, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                detected = True
        if detected:
            x_center, y_center = x + width // 2, y + height // 2
            return x_center, y_center
        else:
            return  None, None
    
    @classmethod
    def GetDistance(cls, width, real_width):
        return (cls.focal_length * real_width) / width + 4
    
    @classmethod
    def GetBearing(cls, x_center):
        offset_pixels = x_center - SCREEN_WIDTH/ 2
        return (offset_pixels / SCREEN_WIDTH) * 22.3

# Define the CamFrameGrabber class
class CamFrameGrabber:
    # FOV = number of degrees for camera view
    def __init__(self, width, height):
        # Initialize the camera
        
        self.cameraStopped = False
        self.gotFrame = False
        self.currentFrame = np.zeros((height, width, 3), np.uint8)
        self.frameid = 0  # Initialize frame ID
        self.newFrameAvailable = False
        self.time_of_frame = 0
        self.fps = 0
        #self.lock = Lock()  # Create a lock for thread safety
        self.new_frame_condition = Condition()

        # Capture the first frame
        self.currentFrame = Specific.CaptureFrame()

    def start(self):
        # Start the image capturing thread
        self.previous_frame_id = -1
        Thread(target=self.captureImage, args=()).start()  # Running the camera capturing in background threads
        return self

    def captureImage(self):
        # Continuously capture frames until stopped
        while True:
            if self.cameraStopped:
                return
            # Capture frame from the camera
            #with self.lock:
            with self.new_frame_condition:
                Specific.CaptureFrameAsync(self.captureImageAsyncSignal)
                self.new_frame_condition.wait()
            time.sleep(0.01)
            
            

    def captureImageAsyncSignal(self, job):
        with self.new_frame_condition:
            self.currentFrame = Specific.cam_wait_job(job)
            t_last = self.time_of_frame
            self.time_of_frame = time.time()
            self.fps = 1.0 / (self.time_of_frame - t_last)
            #print("new frame captured: %d" % self.frameid)
            self.frameid += 1  # Increment the frame ID after capturing each frame
            self.newFrameAvailable = True
            self.new_frame_condition.notify()

    def getCurrentFrame(self):
        # Return the current frame
        #with self.lock:
        with self.new_frame_condition:
            self.newFrameAvailable = False
            #print("get current frame: %d" % self.frameid)
            return self.currentFrame.copy()

    def getFrameID(self):
        # Return the current frame ID
        return self.frameid

    def stop(self):
        # Stop the camera capture
        self.cameraStopped = True


    # def Displaying(self, WindowName, imgRGB):
    #     # Initialize previous_frame_id if it hasn't been initialized yet
    #     if not hasattr(self, 'previous_frame_id'):
    #         self.previous_frame_id = -1
        
    #     current_frame_id = self.getFrameID()
        
    #     # Only display the frame if the frame ID is different from the previous one
    #     if current_frame_id != self.previous_frame_id:
    #         # Display the frame using OpenCV
    #         cv2.imshow(WindowName, imgRGB)
        
    #     # Update the previous frame ID
    #     self.previous_frame_id = current_frame_id


    # def __del__(self):
    #     # Release the camera and clean up OpenCV windows
    #     self.camera.release()
    #     cv2.destroyAllWindows()
