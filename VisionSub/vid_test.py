import cv2
import numpy as np
import vision as vs


import cv2
import math

import cv2
import math

import cv2
import math

def is_edge_line(pt1, pt2, image_width, image_height):
    """Check if the line touches the edges of the frame."""
    x1, y1 = pt1
    x2, y2 = pt2

    # If either point lies on the image boundary (edges), consider it an edge line
    if (x1 == 0 or x1 == image_width or y1 == 0 or y1 == image_height or
        x2 == 0 or x2 == image_width or y2 == 0 or y2 == image_height):
        return True
    return False

def draw_custom_lines(image, contours):
      lines = []  # To store the lines and their lengths
      image_height, image_width = image.shape[:2]  # Get image dimensions

      for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Only consider contours with at least 4 points
            if len(approx) >= 4:
                  # Collect line segments and their lengths
                  for i in range(len(approx)):
                        pt1 = tuple(approx[i][0])
                        pt2 = tuple(approx[(i + 1) % len(approx)][0])  # Wrap around to the first point

                        # Eliminate lines that touch the edges of the image
                        if is_edge_line(pt1, pt2, image_width, image_height):
                              continue  # Skip this line if it touches the edge

                        # Calculate the Euclidean distance (length of the line)
                        length = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2)
                        
                        # Append (pt1, pt2, length) to the list
                        lines.append((pt1, pt2, length))

      # Sort the lines by length in descending order
      lines = sorted(lines, key=lambda x: x[2], reverse=True)

      # Only take the top 4 longest lines
      longest_lines = lines[:4]

      # Now draw the longest lines and calculate their angles
      for pt1, pt2, length in longest_lines:
            # Draw the line between pt1 and pt2
            cv2.line(image, pt1, pt2, (0, 255, 255), 2)

            # Calculate the angle of the line relative to the horizontal
            dx = pt2[0] - pt1[0]
            dy = pt2[1] - pt1[1] #Take the coordinates of the vector
            angle_radians = math.atan2(dy, dx) # Calculate the angle in radians
            angle_degrees = abs(math.degrees(angle_radians)) # Convert to degrees and take the absolute value
            # Print or display the angle
            print(f"Line from {pt1} to {pt2} with length {length:.2f} has an angle of {angle_degrees:.2f} degrees")

            # Optionally, display the angle on the image at the midpoint
            midpoint = ((pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2)
            cv2.putText(image, f"{angle_degrees:.1f} deg", midpoint, cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (0, 255, 0), 1, cv2.LINE_AA)

      return image






def main():
    # Path to the image file
      image_path = '/home/po/EGB320-Group16/VisionSub/videos/img.jpg'  # Replace with the path to your image
      
      # Load the image
      image = cv2.imread(image_path)
      print(image.shape)
      image = image[700:2048, 0:2731]
      if image is None:
            print(f"Error: Could not load image from {image_path}")
            return

      FRAME_WIDTH = 820
      FRAME_HEIGHT = 616

      # Resize the image to the desired resolution (if necessary)
      image = cv2.resize(image, (FRAME_WIDTH, FRAME_HEIGHT))

      # Initialize vision module
      vision = vs.VisionModule()

      try:
            # Convert the image to different color spaces
            imgRGB = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            imgHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            RobotView = image.copy()


            # Draw crosshair at the center
            CenterCoord = vision.draw_crosshair(RobotView)

            # Find contours for the shelves
            contoursShelf, ShelfMask = vision.findShelf(imgHSV)
            ShelfCenters = vision.GetContoursShelf(contoursShelf, RobotView, (0, 0, 255), "S", Draw=True)
            ShelfCenter, ShelfBearing = vision.GetInfoShelf(RobotView, ShelfCenters, imgRGB)
            draw_custom_lines(RobotView, contoursShelf)
     


            # Display the processed image
            cv2.imshow("Processed Image", RobotView)

            # Wait indefinitely for a key press to close the window
            cv2.waitKey(0)

      except KeyboardInterrupt:
            print("Stopping image processing...")

    # Close all OpenCV windows
      cv2.destroyAllWindows()

if __name__ == "__main__":
      main()
