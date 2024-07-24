import cv2
import picamera2
import numpy as np
import time

# Create a camera object
cap = picamera2.Picamera2()

# Print the different camera resolutions/modes the sensor can be configured for
print(cap.sensor_modes)

# Set a specific configuration
# Smaller resolution will be faster but will have a cropped field of view
config = cap.create_video_configuration(main={"format": 'XRGB8888', "size": (320, 240)})
cap.configure(config)

# Start the camera
cap.start()

# Set up the SimpleBlobDetector
def setup_blob_detector():
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 100
    params.filterByCircularity = True
    params.minCircularity = 0.7
    params.filterByConvexity = True
    params.minConvexity = 0.8
    params.filterByInertia = True
    params.minInertiaRatio = 0.5
    detector = cv2.SimpleBlobDetector_create(params)
    return detector

detector = setup_blob_detector()

# Define the orange color range in HSV
lower_orange = np.array([5, 150, 150])
upper_orange = np.array([15, 255, 255])

while True:
    t1 = time.time()  # For measuring FPS

    frame = cap.capture_array()  # Capture a single image frame

    # Convert frame to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the orange color
    mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)

    # Detect blobs
    keypoints = detector.detect(mask)
    
    # Draw detected blobs
    frame_with_blobs = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # Display the frame with blobs
    cv2.imshow("CameraImage", frame_with_blobs)
    
    # Display FPS
    fps = 1.0 / (time.time() - t1)
    print("Frame Rate: ", int(fps), end="\r")

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Clean up
cap.close()
cv2.destroyAllWindows()
