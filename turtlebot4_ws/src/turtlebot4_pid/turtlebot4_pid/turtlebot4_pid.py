# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from tcs_interface.msg	import ColorRGBC as  rgbtype

PI=3.1415926535897932384626433832795
class PID_controller:
	def __init__(self,kp,ki,kd,isat):
		self.kp = kp
		self.ki = ki
		self.kd = kd
		self.isat = isat
		self.old_error = 0
		self.sum_integral = 0

	# Must be called every 100ms
	def compute(self, dt, error):
		# Check if the sum and the error have a different sign. If so, reset the sum
		proportional = self.kp*error

		if (self.sum_integral*error < 0):
			#self.sum_integral = max(min(self.sum_integral + 4*error, self.isat), -self.isat)
			# self.sum_integral = round((self.sum_integral+error) / (20*dt), 1)
			#self.sum_integral = 0
			self.sum_integral = max(min(self.sum_integral + (4*error*dt), self.isat), -self.isat)
		else:
			# Clamp the error to an integral saturation value
			self.sum_integral = max(min(self.sum_integral + (error*dt), self.isat), -self.isat)
		integral = self.ki*self.sum_integral

		integral_sum_load = abs(self.sum_integral)/self.isat
		
		dErr = error - self.old_error
		derivative = self.kd*(dErr/dt)
		self.old_error = error

		pid = float(proportional + integral + derivative)
		#print(self.sum_integral)
		
		#integral_sum_load = float(self.sum_integral/self.isat)
		return (pid)
	
class MinimalPublisher(Node):

	def __init__(self):
		super().__init__('turtlebot_pid')
  
		self.publisher_ = self.create_publisher(Twist, 'lite_1/cmd_vel', 10)
		self.subscription = self.create_subscription(rgbtype,'tcs',self.get_RGBvalue,10)
		self.subscription
  
		timer_period = 0.5  # seconds
		self.timer = self.create_timer(timer_period, self.pid_send)
  
		self.i = 0
		self.valeurgb= rgbtype()#specific topicof the RGB sensor
		self.pidangle=PID_controller(3,2.0,0,60)#todo Change the values of the PID

	   
	def get_RGBvalue(self, msgsub):
		self.valeurgb=msgsub

	def pid_send(self):
		error=(self.valeurgb.r+self.valeurgb.g+self.valeurgb.b)/(3*2.55)#todo Change the value of the error and the consigne
		error=error-28#todo Change the value of the error and the consigne
		white=self.valeurgb.c    
		angle=self.pidangle.compute(0.1,error)#todo Change the value of the dt
	 
	 
		msg = Twist() 
		msg.linear.x=0.5#self.order.straight; #todo Change the value of the speed
		#msg.angular.z=self.order.rotate*PI# left or right
		msg.angular.z=angle#todo want a float
		self.publisher_.publish(msg)
		self.i += 1

def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()

	rclpy.spin(minimal_publisher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
