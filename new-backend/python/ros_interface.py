from aut_msgs.srv import SaveMap
from aut_msgs.srv import LoadMap
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node


class RosInterface(Node):

	def __init__(self):
		super().__init__('ros_clients')

		self.client_slam_start = self.create_client(Trigger, "aut_slam/start")
		self.client_slam_stop = self.create_client(Trigger, "aut_slam/stop")
		self.client_slam_save_map = self.create_client(SaveMap, "aut_slam/save_map")
		self.client_slam_reset = self.create_client(Trigger, "aut_slam/reset")

		self.client_localization_start = self.create_client(Trigger, "aut_localization/start")
		self.client_localization_stop = self.create_client(Trigger, "aut_localization/stop")
		self.client_localization_load_map = self.create_client(LoadMap, "aut_localization/load_map")

		self.client_navigation_load_map = self.create_client(LoadMap, "aut_global_planner/load_map")

		self.robot_connected = False
		self.slam_started = False
		self.localization_started = False
		self.localization_map_loaded = False
		self.navigation_map_loaded = False
		self.map_saved = False

		self.potree_path = None

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

	def send_request_slam_save_map(self, destination, resolution=0.01):
		req = SaveMap.Request()
		req.destination = destination
		req.resolution = resolution
		future = self.client_slam_save_map.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)

		if future.result() is not None and future.result().success == True:
			self.map_saved = True

		return future.result()

	def send_request_localization_start(self):
		req = Trigger.Request()
		future = self.client_localization_start.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

		if future.result() is not None and future.result().success == True:
			self.localization_started = True

		return future.result()

	def send_request_localization_stop(self):
		req = Trigger.Request()
		future = self.client_localization_stop.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

		if future.result() is not None and future.result().success == True:
			self.localization_started = False

		return future.result()

	def send_request_localization_load_map(self, destination):
		req = LoadMap.Request()
		req.map_directory_path = destination
		future = self.client_localization_load_map.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)

		if future.result() is not None and future.result().success == True:
			self.localization_map_loaded = True

		return future.result()

	def send_request_navigation_load_map(self, destination):
		req = LoadMap.Request()
		req.map_directory_path = destination
		future = self.client_navigation_load_map.call_async(req)
		rclpy.spin_until_future_complete(self, future, timeout_sec=120.0)

		if future.result() is not None and future.result().success == True:
			self.navigation_map_loaded = True

		return future.result()
	
	def get_timestamp(self):
		return self.get_clock().now().to_msg()

def test():
	rclpy.init()

	ros_interface = RosInterface()

	response = ros_interface.send_request_start();

	ros_interface.destroy_node()

	rclpy.shutdown()

if __name__ == "__main__":
	test()

