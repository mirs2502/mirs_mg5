import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # --- パッケージディレクトリの取得 ---
    mirs_pkg = get_package_share_directory('mirs')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    bt_pkg = get_package_share_directory('bt_pkg')
    cone_detector_pkg = get_package_share_directory('cone_detector')

    # --- 引数の定義 ---
    # 1. マップファイル (使わないがNav2の引数互換のため)
    default_map_path = os.path.join(mirs_pkg, 'maps', 'my_mirs_map.yaml')
    map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map file to load'
    )

    # 2. Behavior Tree XML (2コーン往復版)
    default_bt_xml_path = os.path.join(bt_pkg, 'behavior_tree', 'bt_two_cone.xml')
    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml_path',
        default_value=default_bt_xml_path,
        description='Path to the Behavior Tree XML file'
    )

    # 3. シミュレーション時刻
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated clock if true'
    )

    # 4. ログレベル
    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for Nav2 nodes'
    )

    # 5. ポート設定
    esp_port = DeclareLaunchArgument(
        'esp_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_44dcbf303dfbeb1193273ca4c6d924ec-if00-port0',
        description='ESP32 USB port'
    )
    lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_947ec7a9868124459631e474f6875e54-if00-port0',
        description='LiDAR USB port'
    )

    # --- 1. ハードウェア & 基本システム (mirs_odom_only.launch.py) ---
    mirs_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mirs_pkg, 'launch', 'mirs_odom_only.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'esp_port': LaunchConfiguration('esp_port'),
            'lidar_port': LaunchConfiguration('lidar_port')
        }.items()
    )

    # --- 2. ナビゲーション (Nav2) ---
    nav2_params_file = os.path.join(mirs_pkg, 'config', 'nav2_params.yaml')

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_file,
            'log_level': LaunchConfiguration('log_level'),
        }.items()
    )

    # --- 3. カメラドライバ (v4l2_camera) ---
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': '/dev/video2',
            'frame_id': 'camera',
            'io_method': 'mmap',
            'pixel_format': 'YUYV',
            'camera_info_url': 'file://' + os.path.join(cone_detector_pkg, 'config', 'camera.yaml')
        }],
        remappings=[
            ('image_raw', '/camera/color/image_raw'),
            ('camera_info', '/camera/color/camera_info')
        ]
    )

    # --- 4. 2コーン往復ミッション (two_cone_mission.launch.py) ---
    two_cone_mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bt_pkg, 'launch', 'two_cone_mission.launch.py')
        ),
        launch_arguments={
            'bt_xml_path': LaunchConfiguration('bt_xml_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_lidar_only': 'false'  # カメラフュージョンを有効化
        }.items()
    )

    # --- 5. Static TF (map -> odom) ---
    # オドメトリのみで動かすため、mapとodomを一致させる
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # --- 5.5 Trajectory Publisher ---
    trajectory_publisher = Node(
        package='mirs',
        executable='trajectory_publisher.py',
        name='trajectory_publisher',
        output='screen'
    )

    # --- 6. RViz2 ---
    rviz_config_file = os.path.join(mirs_pkg, 'rviz', 'system_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # --- 7. Groot (v1) ---
    launch_groot_arg = DeclareLaunchArgument(
        'launch_groot',
        default_value='true',
        description='Launch Groot for behavior tree visualization'
    )

    groot_process = ExecuteProcess(
        cmd=[
            '/home/yzksy/mirs_ws/install/groot/lib/groot/Groot',
            '--mode', 'monitor',
            '--publisher_port', '2666',
            '--server_port', '2667',
            '--autoconnect'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_groot'))
    )

    return LaunchDescription([
        map_yaml_file,
        bt_xml_arg,
        use_sim_time,
        log_level,
        esp_port,
        lidar_port,
        launch_groot_arg,

        mirs_hardware_launch,
        nav2_bringup_launch,
        camera_node,
        two_cone_mission_launch,
        static_tf_map_odom,
        trajectory_publisher,
        rviz_node,
        groot_process
    ])
