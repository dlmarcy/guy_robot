import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import BatteryState
import math
import random

class Teensy_Sim(Node):

	def __init__(self):
		super().__init__('teensy_sim')
		
		# robot state variables
		self.left_pos = 0.0
		self.left_vel = 0.0
		self.left_vel_k1 = 0.0
		self.left_vel_k2 = 0.0
		self.left_vel_k3 = 0.0
		self.right_pos = 0.0
		self.right_vel = 0.0
		self.right_vel_k1 = 0.0
		self.right_vel_k2 = 0.0
		self.right_vel_k3 = 0.0
		self.x = 0.0
		self.y = 0.0
		self.prev_v = 0.0
		self.v = 0.0
		self.a = 0.0
		self.theta = 0.0
		self.prev_w = 0.0
		self.w = 0.0

		# motor model and velocity setpoints
		self.a0 = 0
		self.a1 = 0.01170024
		self.a2 = -0.0067191
		self.a3 = -0.00234201
		self.b0 = 1
		self.b1 = -1.78637763
		self.b2 = 0.79534228
		self.b3 = -0.00632552
		self.setpoint_left = 0.0
		self.setpoint_left_k1 = 0.0
		self.setpoint_left_k2 = 0.0
		self.setpoint_left_k3 = 0.0
		self.setpoint_right = 0.0
		self.setpoint_right_k1 = 0.0
		self.setpoint_right_k2 = 0.0
		self.setpoint_right_k3 = 0.0

		# create timing variables for robot model timer
		self.TIMER_PERIOD = 10e-3
		self.TIMER_RATE = 1.0/self.TIMER_PERIOD
		self.robot_model_timer = self.create_timer(self.TIMER_PERIOD, self.robot_model_timer_cb)

		# robot constants
		self.TIRE_DIA = 0.152 # meter
		self.TIRE_SEP = 0.380 # meter
		self.TIRE_SCALE_SEP = self.TIRE_DIA/(2*self.TIRE_SEP)
		self.TIRE_SCALE_DIA = self.TIRE_DIA/4
		
		# set up power monitor
		self.battery_broadcaster = self.create_publisher(BatteryState, 'arduino/battery', 10)
		self.battery_timer = self.create_timer(1.0/1.0, self.battery_timer_cb)
		self.battery_msg = BatteryState()
		self.battery_msg.header.frame_id = "battery"
		self.battery_msg.present = True
		self.battery_msg.power_supply_technology = 4 #POWER_SUPPLY_TECHNOLOGY_SLA 
		self.battery_msg.power_supply_health = 1 # POWER_SUPPLY_HEALTH_GOOD 
		self.battery_msg.power_supply_status = 4 # POWER_SUPPLY_STATUS_FULL 
		self.battery_msg.design_capacity = 6.0

		# set up IMU
		self.imu_broadcaster = self.create_publisher(Imu, 'arduino/imu', 10)
		self.imu_timer = self.create_timer(1.0/20.0, self.imu_timer_cb)
		self.imu_msg = Imu()
		self.imu_msg.header.frame_id = "imu"
		self.imu_msg.orientation_covariance[0] = 0.01
		self.imu_msg.orientation_covariance[4] = 0.01
		self.imu_msg.orientation_covariance[8] = 0.01
		self.imu_msg.angular_velocity_covariance[0] = 0.00005
		self.imu_msg.angular_velocity_covariance[4] = 0.00005
		self.imu_msg.angular_velocity_covariance[8] = 0.00005
		self.imu_msg.linear_acceleration_covariance[0] = 0.002
		self.imu_msg.linear_acceleration_covariance[4] = 0.002
		self.imu_msg.linear_acceleration_covariance[8] = 0.002

		# set up encoders and broadcaster
		self.joint_state_broadcaster = self.create_publisher(JointState, 'arduino/joint_states', 10)
		self.joint_state_timer = self.create_timer(1.0/10.0, self.joint_state_timer_cb)
		self.joint_state_msg = JointState()
		self.joint_state_msg.name = ['gear_left_shaft', 'gear_right_shaft']
		self.joint_state_msg.position = [0.0, 0.0]
		self.joint_state_msg.velocity = [0.0, 0.0]
		
		# set up PID controller, motors, and commander
		self.joint_state_commander = self.create_subscription(JointState, 'arduino/commands', self.commander_cb, 10)
		self.joint_state_commander  # prevent unused variable warning
		self.command_msg = JointState()
	
	def battery_timer_cb(self):
		self.battery_msg.voltage = 12.0
		self.battery_msg.current = -0.7
		self.battery_msg.header.stamp = self.get_clock().now().to_msg()
		self.battery_broadcaster.publish(self.battery_msg)
		
	def imu_timer_cb(self):
		self.imu_msg.orientation.w = math.cos(0.5 * self.theta)
		self.imu_msg.orientation.z = math.sin(0.5 * self.theta)
		self.imu_msg.angular_velocity.z = self.w
		if self.w == 0.0:
			self.imu_msg.linear_acceleration.x = self.a
			self.imu_msg.linear_acceleration.y = 0.0
		else:
			R = self.v/self.w
			self.imu_msg.linear_acceleration.x = self.a + R * (self.w - self.prev_w) * self.TIMER_RATE
			if self.w < 0.0:
				self.imu_msg.linear_acceleration.y = -R * self.w * self.w
			else:
				self.imu_msg.linear_acceleration.y = R * self.w * self.w
		self.imu_msg.header.stamp = self.get_clock().now().to_msg()
		self.imu_broadcaster.publish(self.imu_msg)

	def joint_state_timer_cb(self):
		self.joint_state_msg.position[0] = self.left_pos
		self.joint_state_msg.velocity[0] = self.left_vel
		self.joint_state_msg.position[1] = self.right_pos
		self.joint_state_msg.velocity[1] = self.right_vel
		self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
		self.joint_state_broadcaster.publish(self.joint_state_msg)

	def commander_cb(self, cmd):
		self.setpoint_left = cmd.velocity[0]
		self.setpoint_right = cmd.velocity[1]

	def robot_model_timer_cb(self):
		self.left_vel_k3 = self.left_vel_k2
		self.left_vel_k2 = self.left_vel_k1
		self.left_vel_k1 = self.left_vel
		self.right_vel_k3 = self.right_vel_k2
		self.right_vel_k2 = self.right_vel_k1
		self.right_vel_k1 = self.right_vel
		self.setpoint_left_k3 = self.setpoint_left_k2
		self.setpoint_left_k2 = self.setpoint_left_k1
		self.setpoint_left_k1 = self.setpoint_left
		self.setpoint_right_k3 = self.setpoint_right_k2
		self.setpoint_right_k2 = self.setpoint_right_k1
		self.setpoint_right_k1 = self.setpoint_right
		self.left_vel = -self.b1*self.left_vel_k1 - self.b2*self.left_vel_k2 - self.b3*self.left_vel_k3
		self.left_vel += (self.a1*self.setpoint_left_k1 + self.a2*self.setpoint_left_k2+ self.a3*self.setpoint_left_k3)
		self.left_pos += (self.left_vel * self.TIMER_PERIOD)
		self.right_vel = -self.b1*self.right_vel_k1 - self.b2*self.right_vel_k2 - self.b3*self.right_vel_k3
		self.right_vel += (self.a1*self.setpoint_right_k1 + self.a2*self.setpoint_right_k2+ self.a3*self.setpoint_right_k3)
		self.right_pos += (self.right_vel * self.TIMER_PERIOD)
		slip_l = 1.0/random.paretovariate(100)
		slip_r = 1.0/random.paretovariate(150)
		self.prev_w = self.w
		self.w = (slip_r*self.right_vel - slip_l*self.left_vel) * self.TIRE_SCALE_SEP
		self.prev_v = self.v
		self.v = (slip_r*self.right_vel + slip_l*self.left_vel) * self.TIRE_SCALE_DIA
		self.a = (self.v - self.prev_v) * self.TIMER_RATE
		self.x += (self.v * self.TIMER_PERIOD * math.cos(self.theta))
		self.y += (self.v * self.TIMER_PERIOD * math.sin(self.theta))
		self.theta += (self.w * self.TIMER_PERIOD)
	
def main(args=None):
	try:
		rclpy.init(args=args)

		simulator = Teensy_Sim()

		# print('Starting Teensy simulation node')
		simulator.get_logger().info('Starting Teensy simulation node')

		rclpy.spin(simulator)
	
	except KeyboardInterrupt:
		pass

	except Exception as e:
		print(e)
    
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	simulator.destroy_node()
	#rclpy.shutdown()

if __name__ == '__main__':
    main()

