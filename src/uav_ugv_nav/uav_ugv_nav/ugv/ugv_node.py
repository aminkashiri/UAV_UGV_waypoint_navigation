import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Twist

def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

release_points = [
    (-1.5,-1.5),
    (-1.5,1.5)
]
collect_points = [
    (-1.5, -0.5),
    (0, 1.5)
]
cycles = (release_points, collect_points)

class TB4(Node):
    def __init__(self):
        super().__init__('tb4_node')

        self.landed_topic = "/landed"
        self.pose_topic = "/Robot_2/pose"
        self.cmd_vel_topic = "/tbot4/cmd_vel"

        self.wait_secs = 3.0

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
         


        self.timer = self.create_timer(1.0 / self.loop_hz, self.step)
        self.get_logger().info(f"[WaypointFollower] pose={self.pose_topic} cmd_vel={self.cmd_vel_topic}")

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
            if not self.waiting_for_takeoff:
                self.waiting_for_takeoff = True
                self.arrival_time = now
                self.get_logger().info(
                    f"[WaypointFollower] Reached wp {self.cycle_index+1}/{len(cycles)} "
                    f"({gx:.2f},{gy:.2f}). Waiting {self.wait_secs:.1f}s."
                )
            tw.linear.x = 0.0
            tw.angular.z = 0.0
            elapsed = (now - self.arrival_time).nanoseconds * 1e-9
            if self.state == 0:
                if elapsed >= self.wait_secs:
                    self.state = 1
                    self.waiting_for_takeoff = False
            else:
                if self.drone_landed:
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
    node = TB4()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
