from threading import Thread
from .Globals import *
import time

class PathProcess:

    thread = None
    running = True
    path = []
    seg = 0

    fwd = 0
    rot = 0
    fwdlen = None
    rotlen = None

    @classmethod
    def motorLoop(cls):
        tl = time.time()
        time.sleep(0.01)
        while cls.running:
            tn = time.time()
            delta = tn-tl
            tl = tn

            c_fwd, c_rot = cls.currentSpeed()
            if cls.fwdlen is not None:
                cls.fwdlen -= abs(c_fwd) * delta
            if cls.rotlen is not None:
                cls.rotlen -= abs(c_rot) * delta
            
            # Note: if the segments are very short and we move fast, we maybe should be skipping some segments but this will not happen in the current code
            # Todo if we go below 0, carry that over to the next segment and continue skipping any number of segments until the extra distance is accounted for
            if (cls.fwdlen is not None and cls.fwdlen <= 0) or (cls.rotlen is not None and cls.rotlen <= 0):
                cls.next_seg()
            
            Specific.set_velocity(cls.fwd, cls.rot)

            time.sleep(0.01) # ~ 100 Hz
            # print(f"Motorloop {1.0/delta:.2f} fps")

    @classmethod 
    def next_seg(cls, inc=True):
        if inc:
            cls.seg += 1

        if cls.seg < len(cls.path):
            cls.fwd, cls.rot, cls.fwdlen, cls.rotlen = cls.path[cls.seg]
        else:
            cls.fwd = 0
            cls.rot = 0
            cls.fwdlen = None
            cls.rotlen = None

    @classmethod
    def new_path(cls, newpath):
        cls.path = newpath.copy()
        cls.seg = 0
        cls.next_seg(False)


    @classmethod
    def currentSpeed(cls):
        # todo implement Specific.odometry()
        return cls.fwd, cls.rot

    @classmethod
    def Start(cls):
        cls.thread = Thread(target=cls.motorLoop)
        cls.thread.start()
    
    @classmethod
    def End(cls):
        cls.running = False
        if cls.thread is not None and cls.thread.is_alive():
            cls.thread.join()
    
    @classmethod
    def set_velocity(cls, fwd, rot):
        cls.new_path([(fwd, rot, None, None)])