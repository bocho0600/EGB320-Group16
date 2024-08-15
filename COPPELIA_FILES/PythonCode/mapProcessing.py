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
        shelf_length = 1.13
        shelf_depth = 0.15
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

        packing_station_size = 0.6
        packing_station_flat = 0.3
        pl = np.array([[[0.,0.],
                        [packing_station_size,0.],
                        [packing_station_size,packing_station_flat],
                        [packing_station_flat,packing_station_size],
                        [0.,packing_station_size],
                        [0.,0.]]])


        self.shelf_lines = np.flip(sl, 2)
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
        return image
    
    def closestSegment(self, point, type, visiblefrom=None):
        segments = None
        match type:
            case "shelf":
                segments = self.shelf_lines
            case "packing":
                segments = self.packing_lines
            case "wall":
                segments = self.wall_lines
            case _:
                print("invalid type")
                return None
            
        # Dims: 0=line; 1=start/end; 2=x/y 
        segments = np.array([segments[:, :-1, :], segments[:, 1:, :]]).transpose((1,2,0,3)).reshape((-1, 2, 2))
        ds = segments[:, 1, :] - segments[:, 0, :] # end - start
        cs = point - segments[:, 0, :] # point - start

        if visiblefrom is not None:
            vs = visiblefrom - segments[:, 0, :]
            # Only visible (facing towards)
            mask = np.cross(ds, vs) > 0
            segments = segments[mask]
            ds = ds[mask]
            cs = cs[mask]

        ts = (cs * ds).sum(1) / (ds*ds).sum(1)
        ts = np.clip(ts, 0, 1)
        ps = segments[:, 0, :] + np.array([ts,ts]).T * ds # projections
        ls = ps - point # vector from projection to point
        qs = (ls*ls).sum(1) # square distance to segmentps
        closest_segment = segments[np.argmin(qs)]
        
        return closest_segment


    # Finally to get an weight, translate each observed point to the closest VISIBLE line segment of the correct type,
    # Then take the distance from the point to the line, then apply a weighting function (representing the observation error distribution)
    # We have a confidence for each data point, then take the product of all those to get the obesrvation confidence


map = None

def onMouseClick(event, x, y, flags, param):
    global robot_position
    scale = 512/2.0
    if flags & 1:
        point = np.array([x, y]) / scale
        sexy = (map.closestSegment(point, "shelf",robot_position) * scale).astype(np.int32)
        disp_im = cv2.line(image.copy(), sexy[0, :], sexy[1, :], (0, 0, 255), 2)
        disp_im = cv2.drawMarker(disp_im, (robot_position*scale).astype(np.int32), (0,0,0), 15)
        cv2.imshow("Arena", disp_im)
    elif flags & 2:
        robot_position = np.array([x, y]) / scale
        disp_im = cv2.drawMarker(image.copy(), (robot_position*scale).astype(np.int32), (0,0,0), 15)
        cv2.imshow("Arena", disp_im)

robot_position = np.array([1.,1.])


if __name__ == "__main__":
    map = ArenaMap()
    image = map.draw_arena(512)

    cv2.imshow("Arena", image)
    cv2.setMouseCallback("Arena", onMouseClick)
    while(1):
        k = cv2.waitKey(0) & 0xFF
        if k == 27:
            break
        elif k == ord('q'):
            break
