from smbus import SMBus
import time

# Made by Kelvin Le, QUT-EGB320-G16
class I2C:
    def __init__(self, bus_number=1, addr=0x08):
        self.addr = addr  # I2C address of the Arduino slave
        self.bus = SMBus(bus_number)  # I2C bus (1 for Raspberry Pi, might be different for other devices)
        TRUE_SPEED_MAX = 100
        SET_SPEED_MAX = 255
        self.prev_left_speed = 0
        self.prev_right_speed = 0
        self.prev_servo = [0, 0, 0, 0]  # Initialize prev_servo for 4 servos
        self.Levels = [180, 0, 90, 180]  # Default levels for the servos for each level

    # def convert_100_to_255_range(self,value):
    #     return round((value / TRUE_SPEED_MAX) * SET_SPEED_MAX)

    def string_to_ascii_array(self, input_string):  
        # Function to convert a string to an array of ASCII values to load to I2C bus
        ascii_values = [ord(char) for char in input_string]  # List comprehension for conversion
        return ascii_values

    def LedWrite(self, number, state):
        # Validate inputs
        if number not in range(4):  # LED numbers start from 0 to 3
            print("Invalid LED number. Please choose between 0 and 3.")
            return

        if state not in ["ON", "OFF"]:
            print("Invalid state. Please choose between 'ON' and 'OFF'.")
            return

        # Prepare the command string
        command = f"L{number} {state}"  # Convert number and state to string

        try:
            # Send the command to the Arduino
            ascii_array = self.string_to_ascii_array(command)  # Corrected variable name
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

    def DCWrite(self, number, direction, speed):
        # Validate inputs
        if number not in range(1, 3):  # DC motor numbers start from 1 to 2
            print(f"Invalid DC number: {number}. Please choose between 1 and 2.")
            return

        if speed < 0 or speed > 255:  # Speed range should be between 0 and 255
            print(f"Invalid speed: {speed}. Please choose between 0 and 255.")
            return

        if direction not in ["0", "1", "S"]:  # Accept "0", "1", and "S" for STOP
            print("Invalid direction. Please choose between '0', '1' or 'S' for STOP.")
            return

        # Prepare the command string
        command = f"M{number} {direction} {speed}"
        print(f"Sending command to DC motor: {command}")

        try:
            # Send the command to the Arduino
            ascii_array = self.string_to_ascii_array(command)
            self.bus.write_i2c_block_data(self.addr, 0, ascii_array)
        except Exception as e:
            print(f"Error sending DC motor command: {e}")

    def ServoConfig(self, ID, Angle):
        if ID == 1:
            if Angle != self.prev_servo[0]:  # Check if the angle is different from the previous angle
                self.ServoWrite(1, Angle)
                self.prev_servo[0] = Angle
        elif ID == 2:
            if Angle != self.prev_servo[1]:
                self.ServoWrite(2, Angle)
                self.prev_servo[1] = Angle
        elif ID == 3:
            if Angle != self.prev_servo[2]:
                self.ServoWrite(3, Angle)
                self.prev_servo[2] = Angle
        elif ID == 4:
            if Angle != self.prev_servo[3]:
                self.ServoWrite(4, Angle)
                self.prev_servo[3] = Angle
        else:
            print("Invalid Servo ID. Please choose between 1 and 4.")
            return

    def ServoSet(self, ID, Level):
        print(f"Servo {ID} set to level {Level}")
        if ID == 1:
            self.ServoConfig(1, self.Levels[Level])
        elif ID == 2:
            self.ServoConfig(2, self.Levels[Level])
        elif ID == 3:
            self.ServoConfig(3, self.Levels[Level])
        elif ID == 4:
            self.ServoConfig(4, self.Levels[Level])
        else:
            print("Invalid Servo ID. Please choose between 1 and 4.")
            return
