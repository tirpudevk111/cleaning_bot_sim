from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    front = {
        "rgb": "/front_camera/image_raw",
        "depth": "/front_camera/depth/image_raw",
        "info": "/front_camera/camera_info",
        "frame": "front_camera_link"
    }
    left = {
        "rgb": "/left_camera/image_raw",
        "depth": "/left_camera/depth/image_raw",
        "info": "/left_camera/camera_info",
        "frame": "left_camera_link"
    }
    right = {
        "rgb": "/right_camera/image_raw",
        "depth": "/right_camera/depth/image_raw",
        "info": "/right_camera/camera_info",
        "frame": "right_camera_link"
    }

    return LaunchDescription([
        # RGBD Sync for front camera
        Node(
            package='rtabmap_ros',
            executable='rgbd_sync',
            name='rgbd_sync_front',
            remappings=[
                ('rgb/image', front["rgb"]),
                ('depth/image', front["depth"]),
                ('rgb/camera_info', front["info"])
            ],
            parameters=[{'frame_id': front["frame"], 'approx_sync': True}]
        ),

        # RGBD Sync for left camera
        Node(
            package='rtabmap_ros',
            executable='rgbd_sync',
            name='rgbd_sync_left',
            remappings=[
                ('rgb/image', left["rgb"]),
                ('depth/image', left["depth"]),
                ('rgb/camera_info', left["info"])
            ],
            parameters=[{'frame_id': left["frame"], 'approx_sync': True}]
        ),

        # RGBD Sync for right camera
        Node(
            package='rtabmap_ros',
            executable='rgbd_sync',
            name='rgbd_sync_right',
            remappings=[
                ('rgb/image', right["rgb"]),
                ('depth/image', right["depth"]),
                ('rgb/camera_info', right["info"])
            ],
            parameters=[{'frame_id': right["frame"], 'approx_sync': True}]
        ),

        # Main RTAB-Map Node (use front camera for now)
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            remappings=[
                ('rgb/image', front["rgb"]),
                ('depth/image', front["depth"]),
                ('rgb/camera_info', front["info"])
            ],
            parameters=[
                {'frame_id': front["frame"]},
                {'subscribe_depth': True},
                {'approx_sync': True},
                {'subscribe_rgb': True},
                {'visual_odometry': True},
                {'queue_size': 10}
            ]
        )
    ])
