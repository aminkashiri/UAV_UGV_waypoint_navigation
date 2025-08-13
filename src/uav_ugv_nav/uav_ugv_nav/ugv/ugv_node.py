import time
import math
import rclpy
import threading
import numpy as np
from enum import Enum
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import SingleThreadedExecutor

class TBotStates(Enum):
    MOVING_TO_RELEASE = 0
    SENDING_TAKEOFF = 1
    WAITING_FOR_TAKEOFF = 2
    MOVING_TO_COLLECT = 3
    WAITING_FOR_LANDING = 4

def yaw_from_quat(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw*qz + qx*qy)
    cosy_cosp = 1.0 - 2.0 * (qy*qy + qz*qz)
    return math.atan2(siny_cosp, cosy_cosp)

def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

release_points = [
    (0.7,-1.5),
    # (-1.5,-1.5),
    (0.7,0.0)
]
collect_points = [
    (0.7, -0.5),
    # (-1.5, -0.5),
    (0.7, 0.5)
]
cycles = {
    0:release_points, 3:collect_points
}

class TB4(Node):
    def __init__(self, mimic_communication):
        super().__init__('tb4_node')

        self.mimic_communication = mimic_communication
        self.landed_topic = "/landed"
        self.pose_topic = "/Robot_2/pose"
        self.cmd_vel_topic = "/tbot4/cmd_vel"

        self.wait_secs = 10.0

        self.max_v = 0.2
        self.max_w = 1.0
        self.k_v = 0.7
        self.k_w = 1.8
        self.xy_tol = 0.08
        self.mocap_stale_sec = 0.25
        self.loop_hz = 30.0

        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self.pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.landed_service = self.create_service(Trigger, 'drone_landed', self.landed_cb)
        self.takeoff_client = self.create_client(Trigger, 'drone_takeoff')

        self.current_xy = None
        self.current_yaw = None
        self.last_pose_time = None
        self.cycle_index = 0
        self.state = TBotStates.MOVING_TO_RELEASE
        self.sent_takeoff = False
        self.arrival_time = None
        self.drone_landed= False
         
        self.align_yaw_target = math.radians(0.0)
        self.align_yaw_tol = math.radians(7.0)
        self.align_yaw_k = 1.8
        self.align_yaw_max_w = 0.8
        self.align_yaw_timeout = 4.0
        self.align_deadline = None

        self.aligning_yaw = False

        self.timer = self.create_timer(1.0 / self.loop_hz, self.step)
        self.get_logger().info(f"[WaypointFollower] pose={self.pose_topic} cmd_vel={self.cmd_vel_topic}")

        if self.mimic_communication:
            return

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

    def send_takeoff_request(self):
        if self.mimic_communication:
            return

        self.state = TBotStates.SENDING_TAKEOFF
        self.get_logger().info(f"Sending takeoff request to CZF... sent_takeoff is {self.sent_takeoff}")
        future = self.takeoff_client.call_async(Trigger.Request())
        future.add_done_callback(self.send_takeoff_cb)

    def send_takeoff_cb(self, future):
        response = future.result()
        if response.success:
            self.get_logger().info("Takeoff response received!")
            self.sent_takeoff = True
            #TODO: 
            now_msg = self.get_clock().now().to_msg()
            self.arrival_time = Time(seconds=now_msg.sec, nanoseconds=now_msg.nanosec) 
            self.state = TBotStates.WAITING_FOR_TAKEOFF
        else:
            self.get_logger().error(f"Service call failed")


    def align_yaw(self, now):
        if not self.aligning_yaw:
            self.get_logger().info(
                f"Aliging yaw"
            )
            self.aligning_yaw = True
            self.align_deadline = now + Duration(seconds=self.align_yaw_timeout)
        yaw_err_align = wrap_to_pi(self.align_yaw_target - self.current_yaw)
        need_align = abs(yaw_err_align) > self.align_yaw_tol and (now < self.align_deadline)
        if need_align:
            tw = Twist()
            tw.linear.x = 0.0
            w = self.align_yaw_k * yaw_err_align
            tw.angular.z = max(-self.align_yaw_max_w, min(self.align_yaw_max_w, w))
            self.arrival_time = now
            self.cmd_pub.publish(tw)
            return False
        else:
            self.aligning_yaw = False
            self.align_deadline = None
            return True

    def step(self):
        if self.cycle_index >= len(cycles):
            return

        if self.current_xy is None or self.current_yaw is None or self.last_pose_time is None:
            return

        print("STEP")
        # Mocap wait
        now_msg = self.get_clock().now().to_msg()
        now = Time(seconds=now_msg.sec, nanoseconds=now_msg.nanosec)
        last = Time(seconds=self.last_pose_time.sec, nanoseconds=self.last_pose_time.nanosec)
        age = (now - last).nanoseconds * 1e-9
        if age > self.mocap_stale_sec:
            return

        if self.state == TBotStates.SENDING_TAKEOFF:
            return
        elif self.state == TBotStates.WAITING_FOR_TAKEOFF:
            elapsed = (now - self.arrival_time).nanoseconds * 1e-9
            if elapsed > self.wait_secs:
                self.state = TBotStates.MOVING_TO_COLLECT
            return
        elif self.state == TBotStates.WAITING_FOR_LANDING:
            if self.drone_landed or self.mimic_communication:
                self.get_logger().info("Drone is landed, moving to next release")
                self.state = TBotStates.MOVING_TO_RELEASE
                self.cycle_index += 1
                self.drone_landed = False
            return

        gx, gy = cycles[self.state.value][self.cycle_index]
        dx, dy = gx - self.current_xy[0], gy - self.current_xy[1]
        dist = math.hypot(dx, dy)
        if dist <= self.xy_tol:
            success = self.align_yaw(now)
            if success: 
                if self.state == TBotStates.MOVING_TO_RELEASE:
                    self.get_logger().info(
                        f"Reached release point {self.cycle_index} ({gx:.2f},{gy:.2f}). Waiting for takeoff."
                    )
                    self.send_takeoff_request()
                else:
                    self.get_logger().info(
                        f"Reached collect point {self.cycle_index} ({gx:.2f},{gy:.2f}). Waiting for landing."
                    )
                    self.state = TBotStates.WAITING_FOR_LANDING
            return


        desired_yaw = math.atan2(dy, dx)
        yaw_err = wrap_to_pi(desired_yaw - self.current_yaw)
        v = min(self.k_v * dist, self.max_v)
        w = max(-self.max_w, min(self.max_w, self.k_w * yaw_err))
        if abs(yaw_err) > math.radians(45):
            v = min(v, 0.07)

        tw = Twist()
        tw.linear.x = max(-self.max_v, min(self.max_v, v))
        tw.angular.z = max(-self.max_w, min(self.max_w, w))
        self.cmd_pub.publish(tw)

def main():
    rclpy.init()
    node = TB4(mimic_communication=False)
    ugv_executor = SingleThreadedExecutor()
    ugv_executor.add_node(node)
    ugv_thread = threading.Thread(target=ugv_executor.spin)
    ugv_thread.start()

    ugv_thread.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
