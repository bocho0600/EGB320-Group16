from G16Modules.ItemCollection import ItemCollectionModule as ItemCollection
from G16Modules.RP2040 import I2C

if __name__ == "__main__":
    I2C.init()
    ItemCollection.lifter_down(10)
    




