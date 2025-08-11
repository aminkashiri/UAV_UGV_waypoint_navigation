import time
import rclpy
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from uav_ugv_nav.uav.cf_driver import init_cf, get_scf
from cflib.utils.reset_estimator import reset_estimator
from uav_ugv_nav.uav.optitrack_feed_node import OptiTrackFeedNode

class CZFNode(Node):
    def __init__(self, name="czf_node"):
        super().__init__(name)
        self.cf = init_cf()
        self.takoff_srv = self.create_service(Trigger, 'drone_takeoff', self.takeoff_cb)
        self.landed_client = self.create_client(Trigger, 'drone_landed')
    
    def takeoff_cb(self, request, response):
        self.get_logger().info("Received takeoff command")

        response.success = True
        response.message = "Takeoff command accepted."
        
        thread = threading.Thread(target=self.run_cycle)
        thread.start()

        print("Returning response")
        return response
    
    def visit_waypoints(self):
        reset_estimator(self.cf)
        print("Crazyflie initialized.")
        self.get_logger().info("Starting flight sequence...")
        commander = self.cf.high_level_commander
        self.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        commander.takeoff(0.7, 3.0, yaw=None)
        time.sleep(3.0)

        # commander.go_to(1.0, 0.0, 0.5, 0.0, 5.0)
        time.sleep(10.0)

        # commander.go_to(0.0, 0.0, 0.5, 0.0, 5.0)
        # time.sleep(5.0)

        # commander.go_to(0.0, 1.0, 0.5, 0.0, 5.0)
        # time.sleep(5.0)
        # commander.go_to(1.0, 0.0, 0.5, 0.0, 5.0)
        # time.sleep(5.0)

        self.get_landing_target()
        print("Going to: ",self.landing_target.x, self.landing_target.y)
        time.sleep(1.0)
        commander.go_to(self.landing_target.x, self.landing_target.y, 0.7, 0.0, 5.0)
        time.sleep(5.0)

        # commander.land(0.0, 5.0)
        commander.land(0.39, 5.0)
        time.sleep(5.0)

        # commander.stop()
        # self.get_logger().info("Sequence complete.")

        # self.cf.platform.send_arming_request(False)
        # time.sleep(1.0)

    def run_cycle(self, commuincation=True):
        self.visit_waypoints()

        if commuincation == False:
            return
        if not self.landed_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("TBOT 'landed' service not available!")
            return
        self.get_logger().info("Calling TBOT 'landed' service to notify landing...")

        req = Trigger.Request()
        future = self.landed_client.call_async(req)

        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done():
            result = future.result()
            if result.success:
                self.get_logger().info(f"TBOT confirmed landed: {result.message}")
            else:
                self.get_logger().warn(f"TBOT 'landed' service call failed: {result.message}")
        else:
            self.get_logger().error("Timed out waiting for TBOT 'landed' service response")

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

    mocap_executor = SingleThreadedExecutor()
    mocap_executor.add_node(mocap_node)
    mocap_thread = threading.Thread(target=mocap_executor.spin, daemon=True)

    
    uav_executor = SingleThreadedExecutor()
    uav_executor.add_node(uav_node)
    uav_thread = threading.Thread(target=uav_executor.spin, daemon=True)

    mocap_thread.start()
    print("Waiting for mocap to work for 5s...")
    time.sleep(5)
    uav_thread.start()

    uav_node.visit_waypoints()

    try:
        # Wait for threads to finish (Ctrl+C raises KeyboardInterrupt)
        while mocap_thread.is_alive() and uav_thread.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("Ctrl+C detected, shutting down...")

    mocap_executor.shutdown()
    uav_executor.shutdown()

    mocap_node.destroy_node()
    uav_node.destroy_node()

    rclpy.shutdown()

    get_scf().close_link()  # Crazyflie cleanup
    time.sleep(1)


if __name__ == '__main__':
    main()
