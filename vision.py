import cv2
import picamera2
import numpy as np
import time
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

    def __init__(self):
        self.cap = None  # Initialize the camera object as an instance variable
        self.t1 = None

    def CaptureImage(self):
        frame = self.cap.capture_array()  # Use the instance variable
        frame = cv2.flip(frame, 0)  # OPTIONAL: Flip the image vertically
        return frame

    def initialize_camera(self, frame_height=820, frame_width=616, format='XRGB8888'):
        # Create a camera object and store it as an instance variable
        self.cap = picamera2.Picamera2()
        config = self.cap.create_video_configuration(main={"format": format, "size": (frame_height, frame_width)})
        self.cap.configure(config)
   
        
        #self.cap.set_controls({"ExposureTime": 11000, "AnalogueGain": 1.5,  "ColourGains": (1.22,2.12)})
        self.cap.set_controls({"ExposureTime": 70000, "AnalogueGain": 1,  "ColourGains": (1.4,1.5)}) 
        #self.cap.set_controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})
        #self.cap.set_controls({"ExposureTime": 70000, "AnalogueGain": 1,  "ColourGains": (1.4,1.5)}) 
        self.image_width = frame_width
        self.image_center = self.image_width // 2 # Calculate the center of the image
        self.cap.start()

    def Capturing(self):
        self.t1 = time.time()  # For measuring FPS
        img = self.CaptureImage()  # Capture a single image frame
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
        #imgHSV = cv2.erode(imgHSV, kernel, iterations=1)
        #imgHSV = cv2.dilate(imgHSV, kernel, iterations=1)
        robotview = img.copy()  # Preserve the original image
        return img, imgHSV, robotview

    def ExportImage(self, WindowName, view, FPS=False):
        if FPS:
            fps = 1.0 / (time.time() - self.t1)  # calculate frame rate
            cv2.putText(view, f'{int(fps)}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 2)  # Display the FPS on the screen

        cv2.imshow(WindowName, view)

    def findBlack(self, imgHSV):
        # Create masks for the orange color
        #HSV90 = cv2.rotate(imgHSV, cv2.ROTATE_180)
        BlackMask = cv2.inRange(imgHSV, self.color_ranges['black'][0], self.color_ranges['black'][1])
        #contoursBlack, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return BlackMask

    def findItems(self, imgHSV):
        # Create masks for the orange color
        #HSV90 = cv2.rotate(imgHSV, cv2.ROTATE_180)
        ItemMask1 = cv2.inRange(imgHSV, self.color_ranges['orange1'][0], self.color_ranges['orange1'][1])
        ItemMask2 = cv2.inRange(imgHSV, self.color_ranges['orange2'][0], self.color_ranges['orange2'][1])
        ItemMask = ItemMask1 | ItemMask2  # Combine masks
        contoursItem, _ = cv2.findContours(ItemMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursItem, ItemMask

    def findShelf(self, imgHSV, area_threshold=10000):
        # Create a mask for the blue color range
        ShelfMask = cv2.inRange(imgHSV, self.color_ranges['blue'][0], self.color_ranges['blue'][1])
        
        # Find contours on the mask
        contoursShelf, _ = cv2.findContours(ShelfMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Filter out small contours by area
        filtered_contours = [cnt for cnt in contoursShelf if cv2.contourArea(cnt) > area_threshold]
        
        return contoursShelf, ShelfMask

    def findLoadingArea(self, imgHSV):
        LoadingAreaMask = cv2.inRange(imgHSV, self.color_ranges['yellow'][0], self.color_ranges['yellow'][1])
        contoursLoadingArea, _ = cv2.findContours(LoadingAreaMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursLoadingArea, LoadingAreaMask

    def findObstacle(self, imgHSV):
        ObstacleMask = cv2.inRange(imgHSV, self.color_ranges['green'][0], self.color_ranges['green'][1])
        contoursObstacle, _ = cv2.findContours(ObstacleMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contoursObstacle, ObstacleMask
    
    def findWall(self, imgGray, imgRGB):
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

    def MarkersDetection(self, contoursMarkers, output, colour, Draw=True):
        shape_count = 0  # Initialize the shape counter
        detected_shapes = []  # List to store detected shape information
        shape = None
        for contour in contoursMarkers:
            if cv2.contourArea(contour)>300 and cv2.contourArea(contour) < 1000:  # Skip small contours
                epsilon = 0.04 * cv2.arcLength(contour, True)  # Calculate the perimeter of the contour
                # Approximate the contour
                ShapeContours = cv2.approxPolyDP(contour, epsilon, True)
                # Determine the number of vertices
                num_vertices = len(ShapeContours)
                # Identify the shape based on the number of vertices
                if num_vertices == 4:
                    shape = "Square"
                elif num_vertices > 4 and num_vertices < 8:
                    shape = "Circle"  # Assuming more than 4 sides is a circle for simplicity
                else:
                    shape = "Unknown"

                # Only recognize squares and circles
                if shape in ["Square", "Circle"]:
                    shape_count += 1  # Increment the counter
                    detected_shapes.append((ShapeContours, shape))  # Store shape details
                    if Draw:
                        cv2.drawContours(output, [ShapeContours], -1, colour, 3)
                        x, y = ShapeContours.ravel()[0], ShapeContours.ravel()[1]
                        cv2.putText(output, shape, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)

        # Return all detected shapes and the shape count
        return detected_shapes, shape_count, shape
    

    def GetContoursShelf(self, contours, output, colour, text, Draw=True):
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
        
    def GetContoursObject(self, contours, output, colour, text, Draw = True):
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
    
    def GetDistance(self, width, real_width):
        return (self.focal_length * real_width) / width + 4
        

# Define the CamFrameGrabber class
class CamFrameGrabber:
    # FOV = number of degrees for camera view
    def __init__(self, src, width, height):
        # Initialize the camera
        self.camera = picamera2.Picamera2()
        self.width = width
        self.height = height
        
        # Define the configuration for the camera
        config = self.camera.create_video_configuration(
            main={"format": 'XRGB8888', "size": (height, width)}
        )
        
        self.camera.configure(config)
        self.camera.set_controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})
        self.camera.start()
        
        self.cameraStopped = False
        self.gotFrame = False
        self.currentFrame = np.zeros((height, width, 3), np.uint8)
        self.frameid = 0  # Initialize frame ID
        
        # Capture the first frame
        self.currentFrame = self.camera.capture_array()

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
            self.currentFrame = self.camera.capture_array()
            self.frameid += 1  # Increment the frame ID after capturing each frame

    def getCurrentFrame(self):
        # Return the current frame
        return self.currentFrame

    def getFrameID(self):
        # Return the current frame ID
        return self.frameid

    def stop(self):
        # Stop the camera capture
        self.cameraStopped = True


    def Displaying(self, WindowName, imgRGB):
        # Initialize previous_frame_id if it hasn't been initialized yet
        if not hasattr(self, 'previous_frame_id'):
            self.previous_frame_id = -1
        
        current_frame_id = self.getFrameID()
        
        # Only display the frame if the frame ID is different from the previous one
        if current_frame_id != self.previous_frame_id:
            # Display the frame using OpenCV
            cv2.imshow(WindowName, imgRGB)
        
        # Update the previous frame ID
        self.previous_frame_id = current_frame_id






    def __del__(self):
        # Release the camera and clean up OpenCV windows
        self.camera.release()
        cv2.destroyAllWindows()