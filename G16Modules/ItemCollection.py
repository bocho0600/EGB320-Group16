import RPi.GPIO as GPIO
import time
from RP2040 import I2C

I2C.init(bus_number=1, addr=0x08)
class ItemCollectionModule:
    is_initialized = False
    @classmethod
    def stop_all(cls):
        cls.gripper_stop()
        cls.lifter_stop()

    @classmethod
    def gripper_close(cls,seconds=1):
        I2C.ServoWrite(4, 110)
        time.sleep(seconds)
        cls.gripper_stop()

    @classmethod
    def gripper_open(cls,seconds=1):
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
        I2C.ServoWrite(2, 140)
        time.sleep(seconds)
        cls.lifter_stop()

    @classmethod
    def lifter_down(cls,seconds=1):
        I2C.ServoWrite(2, 40)
        time.sleep(seconds)
        cls.lifter_stop()

    @classmethod
    def lifter_stop(cls):
        I2C.ServoWrite(2, 90)


    prev_height = None
    ave_up_speed = None
    ave_down_speed = None

    @classmethod
    def lift_to_height(height):
        direction = height - prev_height
        if direction >= 0:
            seconds = direction/ ave_up_speed
            lifter_up(seconds)
        elif direction < 0:
            seconds = direction/ ave_down_speed
            lifter_down(seconds)


    @classmethod
    def collect(h):
        lift_to_height(h)
        gripper_open(1)
        gripper_close(20)





