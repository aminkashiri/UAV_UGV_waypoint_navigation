from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from cflib.crazyflie.log import LogConfig
from  uav_ugv_nav.uav.cf_driver import init_cf

class OptiTrackFeedNode(Node):
    def __init__(self, name="optitrack_feed_node"):
        super().__init__(name)
        self.cf = init_cf()

        self.subscription = self.create_subscription(
            PoseStamped,
            '/Robot_1/pose',
            self.pose_callback,
            10
        )
        self.last_sent_time = self.get_clock().now()

        self.internal_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.setup_logging()

    def setup_logging(self):
        log_config = LogConfig(name='Pose', period_in_ms=100)
        for var in ['x', 'y', 'z', 'yaw']:
            log_config.add_variable(f'stateEstimate.{var}', 'float')

        def log_pose_callback(timestamp, data, logconf):
            for var in ['x', 'y', 'z', 'yaw']:
                self.internal_pose[var] = data[f'stateEstimate.{var}']

        log_config.data_received_cb.add_callback(log_pose_callback)
        self.cf.log.add_config(log_config)
        log_config.start()

    def pose_callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_sent_time).nanoseconds < 1e7:  # 100Hz
            return

        pos = msg.pose.position
        ori = msg.pose.orientation
        self.cf.extpos.send_extpose(pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)

        # Log external vs internal pose
        if (now - self.last_sent_time).nanoseconds < 1e9:  # 10ms = 100Hz
            return
        print(f"[External] x: {pos.x:.3f}, y: {pos.y:.3f}, z: {pos.z:.3f}")
        print(f"[Internal] x: {self.internal_pose['x']:.3f}, y: {self.internal_pose['y']:.3f}, z: {self.internal_pose['z']:.3f}, yaw: {self.internal_pose['yaw']:.2f}")
        self.last_sent_time = now
