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
        'wall': (np.array([29, 5, 112]), np.array([46, 54, 241])),
        'floor': (np.array([0, 0, 0]), np.array([179, 255, 255])),
        'yellow': (np.array([29, 177, 244]), np.array([30, 251, 255])),
        'blue': (np.array([81, 0, 0]), np.array([116, 255, 255])),
        'green': (np.array([55, 79, 0]), np.array([70, 255, 255])),
        'orange1': (np.array([0, 100, 0]), np.array([20, 255, 255])),
        'orange2': (np.array([165, 100, 0]), np.array([180, 255, 255])),
        'black': (np.array([38,31, 45]), np.array([66, 121 , 88]))
    }


    focal_length = 30 #cm
    real_circle_diameter = 70 #cm

    def __init__(self):
        self.cap = None  # Initialize the camera object as an instance variable
        self.t1 = None
    import cv2

    def draw_crosshair(self, frame, color=(255, 255, 255), thickness=2):
        # Get the dimensions of the frame
        height, width = frame.shape[:2]
        
        # Calculate the center of the frame
        center_x = width // 2
        center_y = height // 2
        FrameCenter = (center_x, center_y)
        # Define the length of the crosshair arms
        crosshair_length = 5
        
        # Draw the vertical line of the crosshair

        cv2.drawMarker(frame, FrameCenter, color, markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
        # Draw the horizontal line of the crosshair
        return FrameCenter


    def CaptureImage(self):
        self.frame = self.cap.capture_array()  # Capture an image from the camera
        self.frame = cv2.resize(self.frame, (410, 308))
        self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
        return self.frame


    def initialize_camera(self, frame_width=820, frame_height=616, format='XRGB8888'):
        # Create a camera object and store it as an instance variable
        self.cap = picamera2.Picamera2()
        config = self.cap.create_video_configuration(main={"format": format, "size": (frame_width, frame_height)})
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


    def findWall(self, imgHSV, imgRGB):
        # Create masks for the orange color
        WallMask = cv2.inRange(imgHSV, self.color_ranges['wall'][0], self.color_ranges['wall'][1])
        
        # Find contours in the mask
        contoursWall1, _ = cv2.findContours(WallMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Check if any contours are found
        if contoursWall1:
            # Find the largest contour
            largest_contour = max(contoursWall1, key=cv2.contourArea)
            
            # Create an empty mask and draw the largest contour on it
            filledWallMask = np.zeros_like(WallMask)
            cv2.drawContours(filledWallMask, [largest_contour], -1, (255), thickness=cv2.FILLED)
            
            # Apply Gaussian blur to the filled mask
            filledWallMask = cv2.GaussianBlur(filledWallMask, (9, 9), 2)
            
            # Use the filled mask to extract the wall image
            WallImg = cv2.bitwise_and(imgRGB, imgRGB, mask=filledWallMask)
            
            # Convert the extracted image to grayscale
            WallImgGray = cv2.cvtColor(WallImg, cv2.COLOR_BGR2GRAY)
        else:
            # No contours found, return original image and empty mask
            WallImg = np.zeros_like(imgRGB)
            WallImgGray = np.zeros_like(cv2.cvtColor(imgRGB, cv2.COLOR_BGR2GRAY))
            filledWallMask = np.zeros_like(WallMask)
        
        return WallImg, WallImgGray, filledWallMask


    def findMarkers(self, WallImgGray, WallMask):
        _, mask = cv2.threshold(WallImgGray, 110, 255, cv2.THRESH_BINARY_INV)
        markers = cv2.bitwise_and(WallMask, mask)
        _, mask1 = cv2.threshold(markers, 110, 255, cv2.THRESH_BINARY)
        ContoursMarkers, _ = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return ContoursMarkers, mask1



    def findItems(self, imgHSV):
        # Create masks for the orange color
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
    
    # Function to check contour circularity

    def MarkerShapeDetection(self, contoursMarkers, output,image):
        detected = False
        shapeCount = 0
        distances = []  # List to store distances for all detected circles
        bearings = []   # List to store bearings for all detected circles

        for contour in contoursMarkers:
            if (1 < cv2.contourArea(contour) < 50000):  # Area filter
                epsilon = 0.03 * cv2.arcLength(contour, True)
                ShapeContours = cv2.approxPolyDP(contour, epsilon, True)
                #num_vertices = len(ShapeContours)
                num_vertices = 8
                print(num_vertices)
                print(shapeCount, "markers detected")

                #if num_vertices == 4:
                    #shape = "Square"
                if num_vertices in [4, 12]:  # Avoiding conflict with squares
                    shapeCount += 1
                    print(shapeCount, "circle detected")
                    shape = "Circle"

                    # Find the center and radius of the circle
                    (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
                    diameter = 2 * radius

                    # Calculate distance and bearing
                    distance = self.GetDistance(diameter, 70)
                    bearing = self.GetBearing(x_center,image)

                    distances.append(distance)  # Store each distance
                    bearings.append(bearing)    # Store each bearing

                    # Draw text and circle on the output image
                    cv2.putText(output, f"Distance: {distance:.2f} cm", 
                                (int(x_center), int(y_center)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (123, 200, 100), 2)
                    cv2.putText(output, f"Circle: {shapeCount}", 
                                (int(x_center), int(y_center) - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 100, 100), 2)
                    cv2.circle(output, (int(x_center), int(y_center)), 
                            int(radius), (0, 255, 0), 2)

                    detected = True

        if detected:
            return shapeCount, distances, bearings  # Return list of distances and bearings
        else:
            return 0, [], []  # Return zero shapes, and empty lists for distances and bearings



        
    def GetContoursShelf(self, contours, output, colour, text, Draw=True, min_area=1000):
        detected_centers = []  # List to store the centers of detected shelves
        
        for contour in contours:
            if cv2.contourArea(contour) > min_area:
                # Calculate the contour's center using moments
                M = cv2.moments(contour)
                if M['m00'] != 0:  # Avoid division by zero
                    center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                else:
                    center = (0, 0)

                if Draw:
                    # Draw the contour
                    cv2.drawContours(output, [contour], -1, colour, 1)
                    
                    # Draw the text at the center of the contour
                    cv2.putText(output, text, center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Store the detected center
                detected_centers.append(center)

        # Return the list of detected centers, or None if no contours are detected
        if detected_centers:
            return detected_centers
        else:
            return None

    def GetContoursMarkers(self, contours, output, colour, text, Draw=True, min_area=1000):
        detected_centers = []  # List to store the centers of detected shelves
        
        for contour in contours:
            if cv2.contourArea(contour) > min_area:
                # Calculate the contour's center using moments
                M = cv2.moments(contour)
                if M['m00'] != 0:  # Avoid division by zero
                    center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
                else:
                    center = (0, 0)

                if Draw:
                    # Draw the contour
                    cv2.drawContours(output, [contour], -1, colour, 1)
                    
                    # Draw the text at the center of the contour
                    cv2.putText(output, text, center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Store the detected center
                detected_centers.append(center)

        # Return the list of detected centers, or None if no contours are detected
        if detected_centers:
            return detected_centers
        else:
            return None


        
    def GetContoursObject(self, contours, output, colour, text, Draw=True):
        detected_objects = []  # List to store detected object info
        
        for contour in contours:
            if cv2.contourArea(contour) > 50:
                x, y, width, height = cv2.boundingRect(contour)
                
                if Draw:
                    cv2.rectangle(output, (x, y), (x + width, y + height), colour, 1)
                    cv2.putText(output, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Calculate center
                x_center, y_center = x + width // 2, y + height // 2
                
                # Append this object's properties to the list
                detected_objects.append((x_center, y_center, height, width))
        
        # Return list of detected objects
        if detected_objects:
            return detected_objects
        else:
            return None
    #def ShowDetails(Type, Contours)
    def GetContoursObject(self, contours, output, colour, text, Draw=True):
        detected_objects = []  # List to store detected object info
        
        for contour in contours:
            if cv2.contourArea(contour) > 50:
                x, y, width, height = cv2.boundingRect(contour)
                
                if Draw:
                    cv2.rectangle(output, (x, y), (x + width, y + height), colour, 1)
                    cv2.putText(output, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Calculate center
                x_center, y_center = x + width // 2, y + height // 2
                
                # Append this object's properties to the list
                detected_objects.append((x_center, y_center, height, width))
        
        # Return list of detected objects
        if detected_objects:
            return detected_objects
        else:
            return None


    def GetContoursObject(self, contours, output, colour, text, Draw=True):
        detected_objects = []  # List to store detected object info
        
        for contour in contours:
            if cv2.contourArea(contour) > 50:
                x, y, width, height = cv2.boundingRect(contour)
                
                if Draw:
                    cv2.rectangle(output, (x, y), (x + width, y + height), colour, 1)
                    cv2.putText(output, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Calculate center
                x_center, y_center = x + width // 2, y + height // 2
                
                # Append this object's properties to the list
                detected_objects.append((x_center, y_center, height, width))
        
        # Return list of detected objects
        if detected_objects:
            return detected_objects
        else:
            return None

    def GetContoursMarker(self, contours, output, colour, text, Draw=True):
        detected_objects = []  # List to store detected object info
        
        for contour in contours:
            if cv2.contourArea(contour) > 50:
                # Get the minimum enclosing circle for the contour
                (x_center, y_center), radius = cv2.minEnclosingCircle(contour)
                x_center, y_center = int(x_center), int(y_center)
                radius = int(radius)
                
                if Draw:
                    # Draw the circle around the detected object
                    cv2.circle(output, (x_center, y_center), radius, colour, 2)
                    
                    # Draw the text at the center of the circle
                    cv2.putText(output, text, (x_center, y_center - radius - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Append this object's properties to the list (center and radius)
                detected_objects.append((x_center, y_center, radius))
        
        # Return list of detected objects
        if detected_objects:
            return detected_objects
        else:
            return None


    
    def GetDistance(self, width, real_width):
        return (self.focal_length * real_width) / width + 4
    
    def GetBearing(self, x_center,image):
        offset_pixels = x_center - image.shape[1]/ 2
        return (offset_pixels / image.shape[1]) * 70

