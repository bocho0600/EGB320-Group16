##  EGB320 Mobility Functions 
# Revised         11 Sep 2024
# Version                   1
'''!
  @file DC_Motor_Demo.py
  @brief Connect board with raspberryPi.
  @n Make board power and motor connection correct.
  @n Run this demo.
  @n Motor 1 will move slow to fast, orientation clockwise, 
  @n motor 2 will move fast to slow, orientation count-clockwise, 
  @n then fast to stop. loop in few seconds.
  @n Motor speed will print on terminal
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license    The MIT License (MIT)
  @author     [tangjie](jie.tang@dfrobot.com)
  @version    V1.0.1
  @date       2022-04-19
  @url  https://github.com/DFRobot/DFRobot_RaspberryPi_Motor
'''
from __future__ import print_function
import sys
import os
sys.path.append("../")

import time

from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

if THIS_BOARD_TYPE:
  board = Board(1, 0x10)    # RaspberryPi select bus 1, set address to 0x10
else:
  board = Board(7, 0x10)    # RockPi select bus 7, set address to 0x10

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

  # Set board controler address, use it carefully, reboot module to make it effective
  '''
  board.set_addr(0x10)
  if board.last_operate_status != board.STA_OK:
    print("set board address faild")
  else:
    print("set board address success")
  '''

  while board.begin() != board.STA_OK:    # Board begin and check board status
    print_board_status()
    print("board begin faild")
    time.sleep(2)
  print("board begin success")

  # board.set_encoder_enable(board.ALL)                 # Set selected DC motor encoder enable
  board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
  board.set_encoder_reduction_ratio(board.ALL, 43)    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8
  # board.set_moter_pwm_frequency(1000)                 # Set DC motor pwm frequency to 1000HZ

## Begin functions
# M1 is LEFT wheel
# M2 is RIGHT wheel
'''!
      @brief motor_movement
      @param id             Motor list, items in range 1 to 2, or id = self.ALL
      @param orientation    Motor orientation, self.CW (clockwise) or self.CCW (counterclockwise)
      @param speed         Motor pwm duty cycle, in range 0 to 100, otherwise no effective
    '''

def Forwards(speed): 
  print("Forwards")
  board.motor_movement([board.M1], board.CW, speed)
  board.motor_movement([board.M2], board.CW, speed)
  ReadSpeed = board.get_encoder_speed(board.ALL)      # Use boadrd.all to get all encoders speed
  print("M1 encoder speed: %d rpm" %(ReadSpeed[0])) #M2 encoder speed %d rpm" %(ReadSpeed[0], ReadSpeed[1]))
  pass
  
def Backwards(speed): 
  print("Begin backing up")
  board.motor_movement([board.M1], board.CCW, speed)
  board.motor_movement([board.M2], board.CCW, speed)
  print("finito")
  
def TurnRight(speed):
  print("Begin right turn")
  board.motor_movement([board.M1], board.CCW, speed)
  board.motor_movement([board.M2], board.CW, speed) 
  print("Finished right turn")
  
def TurnLeft(speed): 
  print("Begin left turn")
  board.motor_movement([board.M1], board.CW, speed)
  board.motor_movement([board.M2], board.CCW, speed)
  print("Finished left turn")
  
def Turn360(speed): 
  print("Begin 360")
  board.motor_movement([board.M1], board.CW, speed)
  board.motor_movement([board.M2], board.CW, speed)
  print("Finished 360")
  
def Stop(): 
  board.motor_stop(board.ALL)                       # stop all DC motor
  #print("Motors stopped")