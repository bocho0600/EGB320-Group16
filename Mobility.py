
from __future__ import print_function
import sys
import os

DFRobot_module_directory = "/home/group16/units/egb320/EGB320-Group16"
sys.path.append(DFRobot_module_directory)

import time
from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board

board = Board(1, 0x10)    # RaspberryPi select bus 1, set address to 0x10

def board_detect():
  l = board.detecte()
  print("Board list conform:")
  print(l)

''' print last operate status, users can use this variable to determine the result of a function call. '''
def print_board_status():
  if board.last_operate_status == board.STA_OK:
    print("board status: everything ok")
  elif board.last_operate_status == board.STA_ERR:
    print("board status: unexpected error")
  elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
    print("board status: device not detected")
  elif board.last_operate_status == board.STA_ERR_PARAMETER:
    print("board status: parameter error, last operate no effective")
  elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
    print("board status: unsupport board framware version")


if __name__ == "__main__":

  board_detect()    # If you forget address you had set, use this to detected them, must have class instance

  while board.begin() != board.STA_OK:    # Board begin and check board status
    print_board_status()
    print("board begin faild")
    time.sleep(2)
  print("board begin success")

board.set_encoder_enable(board.ALL)                 # Set selected DC motor encoder enable
  #board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
board.set_encoder_reduction_ratio(board.ALL, 150)   # Set selected DC motor encoder reduction ratio, given on website motor reduction ratio is 100
board.set_moter_pwm_frequency(1000)                 # Set DC motor pwm frequency to 1000HZ

## Begin functions. IC facing forward.
# M1 is RIGHT wheel.  Has encoder and   CW forward
# M2 is LEFT wheel.   Has encodeer and  CCW forward
'''!
      @brief motor_movement
      @param id             Motor list, items in range 1 to 2, or id = self.ALL
      @param orientation    Motor orientation, self.CW (clockwise) or self.CCW (counterclockwise)
      @param speed         Motor pwm duty cycle, in range 0 to 100, otherwise no effective
    '''



def Forwards(speed): 
  print("Forwards")
  board.motor_movement([board.M1], board.CW, speed)
  board.motor_movement([board.M2], board.CCW, speed)
  ReadSpeed = board.get_encoder_speed(board.ALL)      # Use boadrd.all to get all encoders speed
  print("M1 encoder speed: %d rpm, M2 encoder speed %d rpm" %(ReadSpeed[0], ReadSpeed[1]))


def Backwards(speed): 
  print("Begin backing up")
  board.motor_movement([board.M1], board.CCW, speed)
  board.motor_movement([board.M2], board.CW, speed)
  ReadSpeed = board.get_encoder_speed(board.ALL)      # Use boadrd.all to get all encoders speed
  print("M1 encoder speed: %d rpm, M2 encoder speed %d rpm" %(ReadSpeed[0], ReadSpeed[1]))
  print("finito")
  
def TurnRight(speed):
  print("Begin right turn")
  board.motor_movement([board.M1], board.CW, speed)
  board.motor_movement([board.M2], board.CW, speed) 
  print("Finished right turn")
  
def TurnLeft(speed): 
  print("Begin left turn")
  board.motor_movement([board.M1], board.CCW, speed)
  board.motor_movement([board.M2], board.CCW, speed)
  print("Finished left turn")
  
def Turn360(speed): 
  print("Begin 360")
  board.motor_movement([board.M1], board.CW, speed)
  board.motor_movement([board.M2], board.CW, speed)
  print("Finished 360")
  
def Stop(): 
  board.motor_stop(board.ALL)                       # stop all DC motor
  print("Motors stopped")

    
# except KeyboardInterrupt:
#       # Handle keyboard interrupt to stop the robot
#       print('Keyboard interrupt received. Stopping the robot...')
#       board.motor_stop(board.ALL)
#       sys.exit(0)



def Move(linear_velocity, angular_velocity):
    '''
    @brief Move the robot based on linear and angular velocity
    @param linear_velocity    Speed of the robot's forward/backward motion (positive for forward, negative for backward)
    @param angular_velocity   Rate of rotation (positive for right turn, negative for left turn)
    '''
    # Convert velocities to motor speed ranges
    max_speed = 100  # 100 is the maximum speed
    left_motor_speed = linear_velocity - angular_velocity
    right_motor_speed = linear_velocity + angular_velocity

    # Ensure the speeds are within the allowable range
    left_motor_speed = max(min(left_motor_speed, max_speed), -max_speed)
    right_motor_speed = max(min(right_motor_speed, max_speed), -max_speed)
    print("The left motor speed is ",left_motor_speed )
    print("The right motor speed is ",right_motor_speed )
    # Determine the movement direction based on speed values
    if left_motor_speed > 0:
        board.motor_movement([board.M1], board.CW, abs(left_motor_speed))
    elif left_motor_speed < 0:
        board.motor_movement([board.M1], board.CW, abs(left_motor_speed))
    else:
        board.motor_stop([board.M1])
    
    if right_motor_speed > 0:
        board.motor_movement([board.M2], board.CCW, abs(right_motor_speed))
    elif right_motor_speed < 0:
        board.motor_movement([board.M2], board.CCW, abs(right_motor_speed))
    else:
        board.motor_stop([board.M2])
    
    # Print the status for debugging
    print(f"Left motor speed: {left_motor_speed}")
    print(f"Right motor speed: {right_motor_speed}")

# Example usage

#(100, 0)  # Move forward with a slight right turn
#speed = 100
#Move(10, 0)  # Move forward with a slight right turn
Turn360(55)
time.sleep(5)
Stop()

