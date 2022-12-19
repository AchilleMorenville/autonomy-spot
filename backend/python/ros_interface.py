from autonomous_interfaces.srv import SlamSaveMap
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node

class RosInterface(Node):

	def __init__(self):
		super().__init__('ros_clients')

		self.client_slam_start = self.create_client(Trigger, "slam/start")
		self.client_slam_stop = self.create_client(Trigger, "slam/stop")
		self.client_slam_save_map = self.create_client(SlamSaveMap, "slam/save_map")
		self.client_slam_reset = self.create_client(Trigger, "slam/reset")
		self.client_spot_driver_odometry_connect = self.create_client(Trigger, "spot_driver/odometry/connect")

		self.robot_connected = False
		self.slam_started = False
		self.map_saved = False

		self.potree_path = 'None'

	def set_potree_path(self, path):
		self.potree_path = path

	def get_potree_path(self):
		return self.potree_path

	def send_request_slam_start(self):
		req = Trigger.Request()
		future = self.client_slam_start.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

		if future.result() is not None and future.result().success == True:
			self.slam_started = True

		return future.result()

	def send_request_slam_stop(self):
		req = Trigger.Request()
		future = self.client_slam_stop.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

		if future.result() is not None and future.result().success == True:
			self.slam_started = False

		return future.result()

	def send_request_slam_reset(self):
		req = Trigger.Request()
		future = self.client_slam_reset.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

		if future.result() is not None and future.result().success == True:
			self.map_saved = False

		return future.result()

	def send_request_slam_save_map(self, destination, resolution=0.1):
		req = SlamSaveMap.Request()
		req.destination = destination + "/map_go.pcd"
		req.resolution = resolution
		future = self.client_slam_save_map.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

		if future.result() is not None and future.result().success == True:
			self.map_saved = True

		return future.result()

	def send_request_spot_driver_connect(self):
		req = Trigger.Request()
		future = self.client_spot_driver_odometry_connect.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

		if future.result() is not None and future.result().success == True:
			self.robot_connected = True

		return future.result()

def test():
	rclpy.init()

	ros_interface = RosInterface()

	response = ros_interface.send_request_start();

	ros_interface.destroy_node()

	rclpy.shutdown()

if __name__ == "__main__":
	test()

