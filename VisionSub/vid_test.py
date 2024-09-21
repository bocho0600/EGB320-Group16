import cv2
import numpy as np
import vision as vs


def draw_custom_lines(image, contours):
      for contour in contours:
            epsilon = 0.01 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Only draw lines if there are enough points
            if len(approx) >= 4:
                  # Draw lines connecting the points
                  for i in range(len(approx)):
                        pt1 = tuple(approx[i][0])
                        pt2 = tuple(approx[(i + 1) % len(approx)][0])  # Wrap around to the first point

                        # Draw the line between pt1 and pt2
                        cv2.line(image, pt1, pt2, (0, 255, 255), 2)

                        # Calculate and draw the midpoint
                        cv2.circle(image, pt1, 5, (255, 0, 0), -1)  # Draw a filled circle at the midpoint
                        cv2.circle(image, pt2, 5, (255, 0, 0), -1)  # Draw a filled circle at the midpoint

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
