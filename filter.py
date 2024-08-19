import cv2
import numpy as np
import picamera2
import time

def nothing(x):
    pass

# Initialize PiCamera
def initialize_camera(self, frame_height=320*2, frame_width=240*2, format='XRGB8888'):
    # Create a camera object and store it as an instance variable
    cap = picamera2.Picamera2()
    config = self.cap.create_video_configuration(main={"format": format, "size": (frame_height, frame_width)})
    cap.configure(config)

    cap.set_controls({"ExposureTime": 400000, "AnalogueGain": 7.6, "ColourGains": (1.2,2)}, "AWBMode": False)
    
    cap.start()

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


try:
    cap = initialize_camera()
    while True:
        # Capture frame-by-frame
        frame = cap.capture_array()

        # Get current camera settings (metadata)
        metadata = cap.capture_metadata()
        exposure_time = metadata.get("ExposureTime", "N/A")
        awb_mode = metadata.get("AwbMode", "N/A")
        awb_gains = metadata.get("AwbGains", "N/A")

        # Print the current camera settings
        print(f"Current Exposure Time: {exposure_time}")
        print(f"Current AWB Mode: {awb_mode}")
        print(f"Current AWB Gains: {awb_gains}")

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
