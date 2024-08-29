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
        'floor': (np.array([0, 0, 0]), np.array([179, 255, 255])),
        'yellow': (np.array([20, 120, 153]), np.array([25, 233, 218])),
        'blue': (np.array([102, 0, 0]), np.array([179, 179, 255])),
        'green': (np.array([30, 8, 49]), np.array([68, 119, 156])),
        'orange1': (np.array([0, 168, 0]), np.array([20, 255, 255])),
        #'orange2': (np.array([165, 150, 150]), np.array([180, 255, 255])),
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
   
        self.cap.set_controls({"ExposureTime": 21999, "AnalogueGain": 2,  "ColourGains": (1.69,1.45)})
        
        self.cap.start()

    def Capturing(self):
        kernel = np.ones((2, 2), np.uint8)
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

    def findItems(self, imgHSV):
        # Create masks for the orange color
        #HSV90 = cv2.rotate(imgHSV, cv2.ROTATE_180)
        ItemMask1 = cv2.inRange(imgHSV, self.color_ranges['orange1'][0], self.color_ranges['orange1'][1])
        #ItemMask2 = cv2.inRange(HSV90, self.color_ranges['orange2'][0], self.color_ranges['orange2'][1])
        ItemMask = ItemMask1# | ItemMask2  # Combine masks
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
                if num_vertices == 3:
                    shape = "Triangle"
                elif num_vertices == 4:
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
        for contour in contours:
            if cv2.contourArea(contour) > 1000:
                # Get the minimum enclosing circle
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                radius = int(radius)
                
                if Draw:
                    # Draw the circle around the object
                    #cv2.circle(output, center, radius, colour, 2)
                    cv2.drawContours(output, contours, -1, colour, 2)
                    #cv2.drawContours(output, [contour], -1, colour, 2)
                    # Optionally, draw the text near the circle
                    cv2.putText(output, text, (center[0] - radius, center[1] - radius - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                detected = True
        if detected:
            x1, y1, x2, y2 = center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius
            return x1, y1, x2, y2, 
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
        
        

