import RPi.GPIO as GPIO
import time
from .RP2040 import I2C

class ItemCollectionModule:

    is_initialized = False


    @classmethod
    def stop_all(cls):
        cls.gripper_stop()
        cls.lifter_stop()

    @classmethod
    def gripper_close (cls,seconds=1):
        I2C.ServoWrite(4, 110)
        time.sleep(seconds)
        cls.gripper_stop()

    @classmethod
    def gripper_open (cls,seconds=1):
        I2C.ServoWrite(4, 70)
        time.sleep(seconds)
        cls.gripper_stop()

    @classmethod
    def gripper_hold (cls,seconds=1):
        I2C.ServoWrite(4, 100)
        time.sleep(seconds)
        cls.gripper_stop()

    @classmethod
    def gripper_stop(cls):#greater num = close less num = open
        I2C.ServoWrite(4, 90)

    @classmethod
    def lifter_up(cls,seconds=1):
        I2C.ServoWrite(2, 120)
        time.sleep(seconds)
        cls.lifter_stop()

    @classmethod
    def lifter_down(cls,seconds=1):
        I2C.ServoWrite(2, 60)
        time.sleep(seconds)
        cls.lifter_stop()

    @classmethod
    def lifter_stop(cls):
        I2C.ServoWrite(2, 90)

