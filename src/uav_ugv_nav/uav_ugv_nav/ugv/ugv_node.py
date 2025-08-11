import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist

def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

release_points = [
    (0.5,-1.5),
    # (-1.5,-1.5),
    (0.5,0.0)
]
collect_points = [
    (0.5, -0.5),
    # (-1.5, -0.5),
    (0.5, 0.5)
]
cycles = (release_points, collect_points)

class TB4(Node):
    def __init__(self, mimic_commuincation):
        super().__init__('tb4_node')

        self.mimic_communication = mimic_commuincation
        self.landed_topic = "/landed"
        self.pose_topic = "/Robot_2/pose"
        self.cmd_vel_topic = "/tbot4/cmd_vel"

        self.wait_secs = 8.0

        self.max_v = 0.2
        self.max_w = 1.0
        self.k_v = 0.7
        self.k_w = 1.8
        self.xy_tol = 0.08
        self.mocap_stale_sec = 0.25
        self.loop_hz = 30.0

        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.srv = self.create_service(Trigger, 'drone_landed', self.landed_cb)

        self.current_xy = None
        self.current_yaw = None
        self.last_pose_time = None
        self.cycle_index = 0
        self.state = 0 # 0: going to release, 1: going to collect
        self.waiting_for_takeoff = False
        self.arrival_time = None
        self.drone_landed= False
         
        self.align_yaw_target = math.radians(0.0)
        self.align_yaw_tol = math.radians(7.0)
        self.align_yaw_k = 1.8
        self.align_yaw_max_w = 0.8
        self.align_yaw_timeout = 2.0
        self.aligning_yaw = False
        self.align_deadline = None


        self.timer = self.create_timer(1.0 / self.loop_hz, self.step)
        self.get_logger().info(f"[WaypointFollower] pose={self.pose_topic} cmd_vel={self.cmd_vel_topic}")

        if self.mimic_communication:
            return
        self.cli = self.create_client(Trigger, 'drone_takeoff')
        self.get_logger().info("Waiting for 'drone_takeoff' service...")
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("'drone_takeoff' service not available.")
            raise RuntimeError("Service not available")
        self.get_logger().info("'drone_takeoff' service is available")
        self.req = Trigger.Request()

    def landed_cb(self, request, response):
        self.drone_landed = True
        self.get_logger().info("CZF has landed. Continuing mission...")
        response.success = True
        response.message = "TBot received landing confirmation."
        return response


    def pose_cb(self, msg: PoseStamped):
        self.current_xy = np.array([msg.pose.position.x, msg.pose.position.y], dtype=float)
        self.current_yaw = yaw_from_quat(
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        )
        self.last_pose_time = msg.header.stamp

    def send_takeoff_request(self, now):
        self.get_logger().info("Sending takeoff request to CZF...")
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.done():
            response = future.result()
            self.get_logger().info(f"Takeoff response: success={response.success}, message='{response.message}'")
            success = response.success
        else:
            self.get_logger().error("Takeoff service call timed out.")
            success = False

        if success:
            self.get_logger().info("CZF accepted takeoff command.")
        else:
            self.get_logger().error("Failed to initiate takeoff. Stopping")
            exit()

        self.waiting_for_takeoff = True
        self.arrival_time = now

    def step(self):
        if self.cycle_index >= len(cycles):
            return

        if self.current_xy is None or self.current_yaw is None or self.last_pose_time is None:
            return

        # Mocap wait
        now_msg = self.get_clock().now().to_msg()
        now = Time(seconds=now_msg.sec, nanoseconds=now_msg.nanosec)
        last = Time(seconds=self.last_pose_time.sec, nanoseconds=self.last_pose_time.nanosec)
        age = (now - last).nanoseconds * 1e-9
        if age > self.mocap_stale_sec:
            return

        tw = Twist()
        gx, gy = cycles[self.state][self.cycle_index]
        dx, dy = gx - self.current_xy[0], gy - self.current_xy[1]
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        yaw_err = wrap_to_pi(desired_yaw - self.current_yaw)

        if dist <= self.xy_tol:
            if self.waiting_for_takeoff or self.state==1:
                print("Trying to align")
                if not self.aligning_yaw:
                    print("setting aliging to yaw to true")
                    self.aligning_yaw = True
                    self.align_deadline = now + Duration(seconds=self.align_yaw_timeout)
                print("self.current yaw: ", self.current_yaw)
                print("desired: ", self.align_yaw_target)
                yaw_err_align = wrap_to_pi(math.radians(self.align_yaw_target) - self.current_yaw)
                done_align = abs(yaw_err_align) <= self.align_yaw_tol or (now > self.align_deadline)
                print("Done align: ", done_align, "yaw err: ", yaw_err_align, now> self.align_deadline)
                if not done_align:
                    tw.linear.x = 0.0
                    w = self.align_yaw_k * yaw_err_align
                    tw.angular.z = max(-self.align_yaw_max_w, min(self.align_yaw_max_w, w))
                    self.arrival_time = now
                    self.cmd_pub.publish(tw)
                    return
                else:
                    self.aligning_yaw = False
                    self.align_deadline = None
            self.get_logger().info(
                f"Reached wp {self.cycle_index}/{len(cycles)} "
                f"({gx:.2f},{gy:.2f})."
                )
            if self.waiting_for_takeoff:
                elapsed = (now - self.arrival_time).nanoseconds * 1e-9
                if elapsed > self.wait_secs:
                    self.state = 1
                    self.waiting_for_takeoff = False
                return

            if self.state == 0:
                if self.mimic_communication:
                    self.waiting_for_takeoff = True
                    self.arrival_time = now
                else:
                    self.send_takeoff_request(now)
            else:
                if self.drone_landed or self.mimic_communication:
                    self.state = 0
                    self.cycle_index += 1
                    self.drone_landed = False
            return

        v = min(self.k_v * dist, self.max_v)
        w = max(-self.max_w, min(self.max_w, self.k_w * yaw_err))
        if abs(yaw_err) > math.radians(45):
            v = min(v, 0.07)

        tw.linear.x = max(-self.max_v, min(self.max_v, v))
        tw.angular.z = max(-self.max_w, min(self.max_w, w))
        self.cmd_pub.publish(tw)

def main():
    rclpy.init()
    node = TB4(mimic_commuincation=True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
