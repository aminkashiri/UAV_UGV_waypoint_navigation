import time
import rclpy
import threading
from rclpy.node import Node
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

        commander.land(0.0, 5.0)
        time.sleep(5.0)

        commander.stop()
        self.get_logger().info("Sequence complete.")

        self.cf.platform.send_arming_request(False)
        time.sleep(1.0)

def main(args=None):
    rclpy.init(args=args)

    mocap_node = OptiTrackFeedNode("mocap_node")
    mocap_thread = threading.Thread(target=rclpy.spin, args=(mocap_node,))
    mocap_thread.start()
    
    print("Waiting for mocap to work for 5s...")
    time.sleep(5)

    node = CZFNode("uav_node")
    node.run_sequence()

    rclpy.shutdown()
    mocap_thread.join()
    mocap_node.destroy_node()
    node.destroy_node()

    get_scf().close_link()
    time.sleep(1)


if __name__ == '__main__':
    main()
