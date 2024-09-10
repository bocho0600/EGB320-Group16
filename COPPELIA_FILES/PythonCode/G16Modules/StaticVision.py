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
        'yellow': (np.array([25, 90, 0]), np.array([36, 233, 255])),
        'blue': (np.array([104, 85, 0]), np.array([123, 255, 255])),
        'green': (np.array([30, 36, 0]), np.array([96, 255, 226])),
        'orange1': (np.array([0, 100, 0]), np.array([20, 255, 255])),
        'orange2': (np.array([165, 100, 0]), np.array([180, 255, 255])),
        'black': (np.array([0, 0, 43]), np.array([127, 123, 105]))
    }


    focal_length = 50 #cm
    real_circle_diameter = 40 #cm

    cap = None
    t1 = None

    @classmethod
    def CaptureImage(cls):
        frame = cls.cap.capture_array()  # Use the instance variable
        frame = cv2.flip(frame, 0)  # OPTIONAL: Flip the image vertically
        return frame

    @classmethod 
    def get_image(cls):
        return cls.CaptureImage()
    
    @classmethod
    def initialize_camera(cls, frame_height=820, frame_width=616, format='XRGB8888'):
        # Create a camera object and store it as an instance variable
        cls.cap = picamera2.Picamera2()
        config = cls.cap.create_video_configuration(main={"format": format, "size": (frame_height, frame_width)})
        cls.cap.configure(config)
   
        
       # cls.cap.set_controls({"ExposureTime": 11000, "AnalogueGain": 1.5,  "ColourGains": (1.22,2.12)})
        cls.cap.set_controls({"ExposureTime": 100000, "AnalogueGain": 1.0, "ColourGains": (1.4,1.5)})
        cls.image_width = frame_width
        cls.image_center = cls.image_width // 2 # Calculate the center of the image
        cls.cap.start()
 
    @classmethod
    def Capturing(cls):
        kernel = np.ones((2, 2), np.uint8)
        cls.t1 = time.time()  # For measuring FPS
        img = cls.CaptureImage()  # Capture a single image frame
        imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # Convert to HSV
        #imgHSV = cv2.erode(imgHSV, kernel, iterations=1)
        #imgHSV = cv2.dilate(imgHSV, kernel, iterations=1)
        robotview = img.copy()  # Preserve the original image
        return img, imgHSV, robotview
  
    @classmethod
    def ExportImage(cls, WindowName, view, FPS=False):
        if FPS:
            fps = 1.0 / (time.time() - cls.t1)  # calculate frame rate
            cv2.putText(view, f'{int(fps)}', (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 100), 2)  # Display the FPS on the screen

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
        
        return filtered_contours, ShelfMask
  
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
   
    @staticmethod    
    def findWall(imgGray, imgRGB):
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
    def MarkersDetection(cls, contoursMarkers, output, colour, Draw=True):
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
    
   
    @staticmethod
    def GetContoursShelf(contours, output, colour, text, Draw=True):
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
   
    @staticmethod
    def GetContoursObject(contours, output, colour, text, Draw = True):
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
    
    @classmethod
    def GetDistance(cls, length, shape):
        if shape == "Circle":
            return (cls.focal_length * cls.real_circle_diameter) / length
        elif shape == "Square":
            return (cls.focal_length * cls.square_width) / length
        

