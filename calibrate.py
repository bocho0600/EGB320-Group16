import numpy as np
import cv2
import mediapipe as mp
import time

cap = cv2.VideoCapture(0)  # Connect to the camera (0 is the default camera)
frameWidth, frameHeight = 320, 240
cap.set(3, frameWidth)  # Set the width of the frame
cap.set(4, frameHeight)  # Set the height of the frame


def main():
    while True:
        success, img = cap.read()  # Read the image from the camera
        if not success:
            print("Failed to capture image")
            break

        cv2.imshow("Image", img)  # Display the image

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()





if __name__ == "__main__": # Run the main function
    main()