import time
import rclpy
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from uav_ugv_nav.uav.cf_driver import init_cf, get_scf
from cflib.utils.reset_estimator import reset_estimator
from rclpy.callback_groups import ReentrantCallbackGroup
from uav_ugv_nav.uav.optitrack_feed_node import OptiTrackFeedNode

waypoints = [
    [(0,1),(1,0)],
    [(0,1),(1,0)],
]

class CZFNode(Node):
    def __init__(self, mimic_communication=False, name="czf_node"):
        super().__init__(name)
        self.cf = init_cf()
        self.takoff_srv = self.create_service(Trigger, 'drone_takeoff', self.takeoff_cb)
        self.landed_client = self.create_client(Trigger, 'drone_landed')
        self.mimic_communication = mimic_communication
        self.timer = self.create_timer(1.0, self.step)
        self.cycle_idx = 0
    
    def step(self):
        self.get_logger().info("I'm alive")
    
    def takeoff_cb(self, request, response):
        self.get_logger().info("Received takeoff command")

        response.success = True
        response.message = "Takeoff command accepted."
        
        threading.Thread(target=self.run_cycle).start()

        self.get_logger().info("Returning takeoff response")
        return response
    
    def run_cycle(self):
        self.get_logger().info("run cycle")
        if self.cycle_idx == 0:
            reset_estimator(self.cf)
        self.get_logger().info("Starting flight sequence...")
        commander = self.cf.high_level_commander
        self.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        commander.takeoff(0.7, 3.0)
        time.sleep(3.0)

        for waypoint in waypoints[self.cycle_idx]:
            commander.go_to(waypoint[0], waypoint[1], 0.7, 0.0, 5.0)
            time.sleep(5.0)

        self.get_landing_target()


        self.get_logger().info(f"Got landing target: {self.landing_target}")
        time.sleep(1.0)
        commander.go_to(self.landing_target.x, self.landing_target.y, 0.7, 0.0, 5.0)
        time.sleep(5.0)
        commander.land(0.39, 5.0)
        time.sleep(5.0)

        print("Finished visiting waypoints")
        self.send_landed()
        self.cycle_idx += 1


    def send_landed(self):
        if self.mimic_communication == True:
            return
        future = self.landed_client.call_async(Trigger.Request())
        future.add_done_callback(self.send_landed_cb)
        self.get_logger().info(f"Send landed")
    
    def send_landed_cb(self, future):
        self.get_logger().info(f"Received landed callback")
        response = future.result()
        if response.success:
            self.get_logger().info(f"Tbot confirmed landed: {response.message}")
        else:
            self.get_logger().warn(f"Service call failed: {response.message}")
            exit()

    def landing_target_cb(self, msg):
        self.landing_target = msg.pose.position
        self.destroy_subscription(self.temp_sub)

    def get_landing_target(self):
        self.landing_target = None
        
        self.temp_sub = self.create_subscription(
            PoseStamped, '/Robot_2/pose', self.landing_target_cb, 1, callback_group=ReentrantCallbackGroup())
        while self.landing_target is None:
            time.sleep(0.1)
        
        
def main(args=None):
    rclpy.init(args=args)

    mocap_node = OptiTrackFeedNode("mocap_node")
    uav_node = CZFNode("uav_node")

    mocap_executor = SingleThreadedExecutor()
    mocap_executor.add_node(mocap_node)
    mocap_thread = threading.Thread(target=mocap_executor.spin)

    
    uav_executor = MultiThreadedExecutor(2)
    uav_executor.add_node(uav_node)
    uav_thread = threading.Thread(target=uav_executor.spin)

    mocap_thread.start()
    print("Waiting for mocap to work for 5s...")
    time.sleep(5)
    print("Starting UAV Thread")
    uav_thread.start()

    mocap_thread.join()
    uav_thread.join()

    mocap_executor.shutdown()
    uav_executor.shutdown()

    mocap_node.destroy_node()
    uav_node.destroy_node()

    rclpy.shutdown()

    get_scf().close_link()
    time.sleep(1)


if __name__ == '__main__':
    main()
