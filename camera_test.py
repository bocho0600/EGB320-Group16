import cv2
import picamera2
import numpy as np
import time

#create a camera object
cap = picamera2.Picamera2()

#print the different camera resolutions/modes 
#the sensor can be configured for
print(cap.sensor_modes)

#set a specific configuration, smaller resolution will be faster
#however will have a cropped field of view
#consider a balance between higher resolution, field of view and frame rate
config = cap.create_video_configuration(main={"format":'XRGB8888',"size":(320,240)})
cap.configure(config)

#start the camera
cap.start()

while(1):
    t1 = time.time()                     # for measuring fps
    
    frame = cap.capture_array()          # capture a single image frame
    cv2.imshow("CameraImage", frame)     # Display the obtained frame in a window called "CameraImage"
    cv2.waitKey(1)			             # Make the program wait for 1ms before continuing (also required to display image).
    
    fps = 1.0/(time.time() - t1)         # calculate frame rate
    print("Frame Rate: ", int(fps), end="\r")

cap.close()

