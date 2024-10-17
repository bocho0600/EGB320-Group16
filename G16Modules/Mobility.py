from __future__ import print_function
import math
from .RP2040 import I2C

class MobilityModule:


	@classmethod
	def Stop(cls): 
		I2C.StopAll()
		print("Motors stopped")

	@classmethod
	def Move(cls, linear_velocity, angular_velocity):
			'''
			@brief Move the robot based on linear and angular velocity
			@param linear_velocity    Speed of the robot's forward/backward motion (positive for forward, negative for backward)
			@param angular_velocity   Rate of rotation (positive for right turn, negative for left turn)
			'''
			# Convert velocities to motor speed ranges
			max_speed = 255  # 255 is the maximum speed (because DC write (pwm) is 255)
			min_speed = 24  # :D

			left_motor_speed = linear_velocity + angular_velocity
			right_motor_speed = linear_velocity - angular_velocity

			# Ensure the speeds are within the allowable range
			if left_motor_speed != 0 and abs(left_motor_speed) < min_speed:
				left_motor_speed = math.copysign(min_speed, left_motor_speed)
			left_motor_speed = max(min(left_motor_speed, max_speed), -max_speed)
			if right_motor_speed != 0 and abs(right_motor_speed) < min_speed:
				right_motor_speed = math.copysign(min_speed, right_motor_speed)
			right_motor_speed = max(min(right_motor_speed, max_speed), -max_speed)
			
			
			I2C.movement(left_motor_speed, right_motor_speed)
I2C.init(bus_number=1, addr=0x08)
MobilityModule.Stop()

