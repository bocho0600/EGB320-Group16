import numpy as np
import cv2

class ArenaMap:
    def __init__(self):
        # Let 0.0 be the packing station corner with all coordinates positive

        width = 2.0
        height = 2.0
        self.size = (width, height)
        self.wall_lines = np.array([[[0., 0.],
                                     [0., height],
                                     [width, height],
                                     [width, 0.],
                                     [0.,0.]]]) # clockwise cause inside is in bounds

        # Boundaries anticlockwise cause inside is out of bounds
        shelf_length = 1.2
        shelf_depth = 0.17
        sl = []

        shelf_1 = np.array([[0.,height-shelf_length],
                            [shelf_depth,height-shelf_length],
                            [shelf_depth,height],
                            [0.,height],
                            [0.,height-shelf_length]])
        
        shelf_2 = np.array([[-shelf_depth,height-shelf_length],
                            [+shelf_depth,height-shelf_length],
                            [+shelf_depth,height],
                            [-shelf_depth,height],
                            [-shelf_depth,height-shelf_length]])
        
        sl.append(shelf_1)
        sl.append(shelf_2 + np.array([width/3., 0.]))
        sl.append(shelf_2 + np.array([2.*width/3., 0.]))
        sl.append(shelf_1 + np.array([width-shelf_depth,0.]))
        sl = np.array(sl)

        packing_station_size = 0.45
        packing_station_flat = 0.15
        pl = np.array([[[0.,0.],
                        [packing_station_size,0.],
                        [packing_station_size,packing_station_flat],
                        [packing_station_flat,packing_station_size],
                        [0.,packing_station_size],
                        [0.,0.]]])


        self.shelf_lines = sl
        self.packing_lines = pl
        self.width = width
        self.height = height
    
    def draw_arena(self, image_width):
        image = np.zeros((image_width, image_width, 3), np.uint8)
        
        # Ground, Packing Station, Shelves
        scale = image_width/self.width
        image = cv2.fillPoly(image, pts=(self.wall_lines * scale).astype(np.int32), color=(100,100,100))
        image = cv2.fillPoly(image, pts=(self.packing_lines * scale).astype(np.int32), color=(0,255,255))
        image = cv2.fillPoly(image, pts=(self.shelf_lines * scale).astype(np.int32), color = (255, 0,0))
        
        # Show image
        image = cv2.flip(image, 0)
        cv2.imshow("Arena", image)
        cv2.waitKey(0)
    
    def closestSegment(self, point, type):
        # ...
        # inside = np.dot(1,1)
        # segment = find (...)
        # return segment, inside
        pass

    def precalculateSegments(self, image_width):
        # pseudocode:
        # closestSegments = np.zeros((image_width, image_width, 3), np.int32)
        # for x in range(image_width):
        #     for y in range(image_width):
        #         closestSegments[x,y,:] = [closestSegments(x,y,'shelf'), closestSegments(x,y,'packing'), closestSegments(x,y,'wall')]
        # self.closestSegments = closestSegments
        pass

    # Finally to get an weight, translate each observed point to the closest line segment of the correct type,
    # Then take the distance from the point to the line, then apply a weighting function (representing the observation error distribution)
    # We have a confidence for each data point, then take the product of all those to get the obesrvation confidence
    # I think the main bottleneck would be finding the closest line segment so i'm gonna just precalculate all of them



if __name__ == "__main__":
    map = ArenaMap()
    map.draw_arena(512)
