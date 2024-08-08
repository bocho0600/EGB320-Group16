import cv2
import numpy as np
import picamera2
import time

# MADE BY KELVIN LE, QUT EGB320 GROUP16 
# StudentID: n11429984
# Email: minhnguyen.le@qut.edu.au

def nothing(x):
    pass

# Initialize PiCamera
def initialize_camera(frame_height=320*2, frame_width=240*2, format='XRGB8888'):
    cap = picamera2.Picamera2()
    config = cap.create_video_configuration(main={"format": format, "size": (frame_width, frame_height)})
    cap.configure(config)
    cap.start()
    return cap

# Create a window
cv2.namedWindow('image')

# Create trackbars for color change
cv2.createTrackbar('HMin', 'image', 0, 179, nothing)
cv2.createTrackbar('SMin', 'image', 0, 255, nothing)
cv2.createTrackbar('VMin', 'image', 0, 255, nothing)
cv2.createTrackbar('HMax', 'image', 0, 179, nothing)
cv2.createTrackbar('SMax', 'image', 0, 255, nothing)
cv2.createTrackbar('VMax', 'image', 0, 255, nothing)

# Set default value for Max HSV trackbars
cv2.setTrackbarPos('HMax', 'image', 179)
cv2.setTrackbarPos('SMax', 'image', 255)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize HSV min/max values
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0

# Initialize the camera
cap = initialize_camera()

try:
    while True:
        # Capture frame-by-frame
        frame = cap.capture_array()
        frame = cv2.flip(frame, 0)  # OPTIONAL: Flip the image vertically

        # Get current positions of all trackbars
        hMin = cv2.getTrackbarPos('HMin', 'image')
        sMin = cv2.getTrackbarPos('SMin', 'image')
        vMin = cv2.getTrackbarPos('VMin', 'image')
        hMax = cv2.getTrackbarPos('HMax', 'image')
        sMax = cv2.getTrackbarPos('SMax', 'image')
        vMax = cv2.getTrackbarPos('VMax', 'image')

        # Set minimum and maximum HSV values to display
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])

        # Convert to HSV format and color threshold
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Print if there is a change in HSV value
        if((phMin != hMin) or (psMin != sMin) or (pvMin != vMin) or (phMax != hMax) or (psMax != sMax) or (pvMax != vMax)):
            print(f"(hMin = {hMin}, sMin = {sMin}, vMin = {vMin}), (hMax = {hMax}, sMax = {sMax}, vMax = {vMax})")
            phMin = hMin
            psMin = sMin
            pvMin = vMin
            phMax = hMax
            psMax = sMax
            pvMax = vMax

        # Display result image
        cv2.imshow('image', result)
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
finally:
    # Release resources
    cap.close()
    cv2.destroyAllWindows()
