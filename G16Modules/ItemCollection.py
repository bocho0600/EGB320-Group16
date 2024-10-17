import time
from .RP2040 import I2C

I2C.init(bus_number=1, addr=0x08)
class ItemCollectionModule:
    is_initialized = False

    lifter_servo = 3
    lifter_positions = [20, 85, 180]

    gripper_servo = 2



    @classmethod
    def stop_all(cls):
        cls.gripper_stop()
        cls.lifter_set(2)

    @classmethod
    def gripper_close(cls,seconds=1):
        # I2C.ServoWrite(4, 90)
        # time.sleep(seconds)
        # cls.gripper_stop()
        pass

    @classmethod
    def gripper_open(cls,seconds=1):
        # I2C.ServoWrite(4, 70)
        # time.sleep(seconds)
        # cls.gripper_stop()
        pass

    @classmethod
    def gripper_hold (cls,seconds=1):
        # I2C.ServoWrite(4, 100)
        # time.sleep(seconds)
        # cls.gripper_stop()
        pass

    @classmethod
    def gripper_stop(cls):#greater num = close less num = open
        # I2C.ServoWrite(4, 80)
        pass

    @classmethod
    def lifter_set(cls,level, seconds=1):
        I2C.ServoWrite(cls.lifter_servo, cls.lifter_positions[level])
        time.sleep(seconds)



    # @classmethod
    # def collect(cls,h):
    #     cls.lifter_set(h)
    #     cls.gripper_open(1)
    #     cls.gripper_close(20)





