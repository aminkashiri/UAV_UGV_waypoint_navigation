import time
import rclpy
import threading
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from uav_ugv_nav.uav.cf_driver import init_cf, get_scf
from cflib.utils.reset_estimator import reset_estimator
from uav_ugv_nav.uav.optitrack_feed_node import OptiTrackFeedNode

class CZFNode(Node):
    def __init__(self, name="czf_node"):
        super().__init__(name)
        self.cf = init_cf()

    def run_sequence(self):
        reset_estimator(self.cf)
        print("Crazyflie initialized.")
        self.get_logger().info("Starting flight sequence...")
        commander = self.cf.high_level_commander
        self.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        commander.takeoff(0.5, 3.0)
        time.sleep(3.0)

        # commander.go_to(1.0, 0.0, 0.5, 0.0, 5.0)
        # time.sleep(5.0)

        # commander.go_to(0.0, 0.0, 0.5, 0.0, 5.0)
        # time.sleep(5.0)

        # commander.go_to(0.0, 1.0, 0.5, 0.0, 5.0)
        # time.sleep(5.0)
        # commander.go_to(1.0, 0.0, 0.5, 0.0, 5.0)
        # time.sleep(5.0)

        self.get_landing_target()
        print("Going to: ",self.landing_target.x, self.landing_target.y)
        time.sleep(1.0)
        commander.go_to(self.landing_target.x, self.landing_target.y, 0.5, 0.0, 5.0)
        time.sleep(5.0)

        commander.land(0.0, 5.0)
        time.sleep(5.0)



        commander.stop()
        self.get_logger().info("Sequence complete.")

        self.cf.platform.send_arming_request(False)
        time.sleep(1.0)

    def get_landing_target(self):
        self.landing_target = None
        
        def temp_callback(msg):
            self.landing_target = msg.pose.position
        
        temp_sub = self.create_subscription(
            PoseStamped, '/Robot_2/pose', temp_callback, 1)
        
        while self.landing_target is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        temp_sub.destroy()
        print(f"Got landing target: {self.landing_target}")
        
def main(args=None):
    rclpy.init(args=args)

    mocap_node = OptiTrackFeedNode("mocap_node")
    uav_node = CZFNode("uav_node")

    executor = SingleThreadedExecutor()
    executor.add_node(mocap_node)
    # executor.add_node(uav_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    
    print("Waiting for mocap to work for 5s...")
    time.sleep(5)

    uav_node.run_sequence()

    executor.shutdown()
    rclpy.shutdown()
    executor_thread.join()
    mocap_node.destroy_node()
    uav_node.destroy_node()

    get_scf().close_link()
    time.sleep(1)


if __name__ == '__main__':
    main()
