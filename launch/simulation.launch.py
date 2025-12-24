import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_self_driving_bot'
    pkg_share = get_package_share_directory(pkg_name)
    # Ép Gazebo sử dụng phần mềm render nếu phần cứng lỗi
    # Hoặc ép sử dụng engine Ogre (thay vì Ogre2 nếu bị sập)
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    os.environ['GZ_RENDERING_ENGINE_GUESS'] = 'ogre' 
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    os.environ['GZ_SIM_RESOURCE_PATH'] = pkg_share # Đảm bảo tìm thấy tài nguyên

    # 1. Khai báo các đường dẫn
    world_file = os.path.join(pkg_share, 'worlds', 'track.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'vehicle.xacro')
    
    # 2. Xử lý Robot URDF
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # 3. Ép đường dẫn tìm tài nguyên (Mesh/World)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_share, 'worlds'),
            os.path.join(pkg_share, 'urdf'),
            os.path.join(pkg_share, 'mesh')
        ])
    )

    # 4. Khởi chạy Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # THÊM THAM SỐ --render-engine ogre trực tiếp vào gz_args
        launch_arguments={'gz_args': f'-r --render-engine ogre {world_file}'}.items(),
    )

    # 5. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': True}]
    )

    # 6. Bridge (Cầu nối ROS 2 - Gazebo)
    # 6. Bridge (Cập nhật thêm /clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/model/my_bot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/default/model/my_bot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # 7. Spawner xe - Đặt tại góc chéo (-15, -15)
    spawn_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-x', '47.0', 
            '-y', '-2.4',
            '-z', '0.08',  # Hạ thấp một chút để bánh chạm đường ngay
            '-Y', '2.5'   # Quay mặt về hướng km số 0
        ],
        output='screen'
    )
    # Trả về các hành động launch
    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        node_robot_state_publisher,
        bridge,
        # Đợi 5 giây để Gazebo load Map xong rồi mới thả xe
        TimerAction(period=5.0, actions=[spawn_cmd])
    ])