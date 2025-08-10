from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    optitrack_feed_uav = Node(
        package='uav_ugv_nav',
        executable='optitrack_feed_node',
        name='optitrack_feed_uav',
    )

    uav_node = TimerAction(
        period=5.0,  # delay in seconds
        actions=[
            Node(
                package='uav_ugv_nav',
                executable='uav_node',
                name='uav_node',
            )
        ]
    )

    return LaunchDescription([
        optitrack_feed_uav,
        uav_node,
    ])
