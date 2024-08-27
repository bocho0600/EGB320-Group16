from smbus import SMBus
import time
# Made by Kelvin Le, QUT-EGB320-G16
class I2C:
    def __init__(self, bus_number=1, addr=0x08):
        self.addr = addr  # I2C address of the Arduino slave
        self.bus = SMBus(bus_number)  # I2C bus (1 for Raspberry Pi, might be different for other devices)

    def string_to_ascii_array(self, input_string):  # Function to convert a string to an array of ASCII values to load to I2C bus
        ascii_values = [ord(char) for char in input_string]  # List comprehension for conversion
        return ascii_values

    def LedWrite(self, number, state):
        # Validate inputs
        if number not in range(4):
            print("Invalid LED number. Please choose between 0 and 3.")
            return

        if state not in ["ON", "OFF"]:
            print("Invalid angle. Please choose between 0 and 180.")
            return

        # Prepare the command string
        command = f"L{number} {state}" #Convert number and state to string

        try:
            # Send the command to the Arduino
            ascii_array = self.string_to_ascii_array(command)
            self.bus.write_i2c_block_data(self.addr, 0, ascii_array)
            print(f"Sent command: {command}")
        except Exception as e:
            print(f"Error sending command: {e}")


    def ServoWrite(self, number, angle):
        # Validate inputs
        if number not in range(1, 5):  # Servo numbers start from 1 to 4
            print(f"Invalid Servo number: {number}. Please choose between 1 and 4.")
            return

        if angle < 0 or angle > 180:  # Allow any angle within the servo's range
            print(f"Invalid angle: {angle}. Please choose between 0 and 180.")
            return

        # Prepare the command string
        command = f"S{number} {angle}"
        print(f"Sending command to servo: {command}")

        try:
            # Send the command to the Arduino
            ascii_array = self.string_to_ascii_array(command)
            self.bus.write_i2c_block_data(self.addr, 0, ascii_array)
        except Exception as e:
            print(f"Error sending servo command: {e}")






# Example usage


