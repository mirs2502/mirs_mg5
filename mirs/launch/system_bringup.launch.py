import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- パッケージディレクトリの取得 ---
    mirs_pkg = get_package_share_directory('mirs')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    bt_pkg = get_package_share_directory('bt_pkg')
    cone_detector_pkg = get_package_share_directory('cone_detector')

    # --- 引数の定義 ---
    # 1. マップファイル
    default_map_path = os.path.join(mirs_pkg, 'maps', 'my_mirs_map.yaml')
    map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map file to load'
    )

    # 2. Behavior Tree XML
    default_bt_xml_path = os.path.join(bt_pkg, 'behavior_tree', 'bt_mission.xml')
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

    # 5. 経路生成モード (Zigzag vs Rectangle)
    use_rectangle_path = DeclareLaunchArgument(
        'use_rectangle_path',
        default_value='true',
        description='If true, use rectangle generator (Oomawari) instead of zigzag generator'
    )

    # 6. ポート設定
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

    # --- 1. ハードウェア & 基本システム (mirs.launch.py) ---
    # 含まれるもの: Micro-ROS, LiDAR, EKF(Local/Global), URDF(Robot State Publisher)
    mirs_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mirs_pkg, 'launch', 'mirs.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'esp_port': LaunchConfiguration('esp_port'),
            'lidar_port': LaunchConfiguration('lidar_port')
        }.items()
    )

    # --- 2. ナビゲーション (Nav2) ---
    nav2_params_file = os.path.join(mirs_pkg, 'config', 'nav2_params.yaml')
    
    # Nav2 Bringup (Navigation only - No Map Server / AMCL)
    # マップを使わないため、navigation_launch.py を使用する
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
    # real_mission.launch.pyには含まれていないためここで起動
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
            ('image_raw', '/camera/color/image_raw'), # real_missionに合わせてリマップ
            ('camera_info', '/camera/color/camera_info')
        ]
    )

    # --- 4. 認識・計画・BT (real_mission.launch.py) ---
    # 含まれるもの: ScanToPCL, Clustering, Fusion, Area(Polygon), Zigzag, BT Executor
    # 注意: real_mission.launch.py内のCone Color Detector等は、
    # 上記カメラドライバからのトピック(/camera/color/image_raw)を期待している
    real_mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bt_pkg, 'launch', 'real_mission.launch.py')
        ),
        launch_arguments={
            'bt_xml_path': LaunchConfiguration('bt_xml_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_rectangle_path': LaunchConfiguration('use_rectangle_path')
        }.items()
    )

    # --- 5. 自己位置補正 (Landmark Localizer) ---
    # Nav2の地図座標系(map)と、認識したコーン配置を照合して補正
    # COMMENTED OUT: landmark_localizerに設計上の問題があるため一時的に無効化
    landmark_localizer_node = Node(
         package='cone_detector',
         executable='landmark_localizer',
         name='landmark_localizer',
         output='screen',
         parameters=[{'min_shared_landmarks': 1}]
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

    # Delay Landmark Localizer to wait for TF tree to be ready
    # COMMENTED OUT: landmark_localizerを無効化
    # delayed_landmark_localizer_node = TimerAction(
    #     period=5.0,
    #     actions=[landmark_localizer_node]
    # )

    # --- 7. Groot (v1) ---
    launch_groot_arg = DeclareLaunchArgument(
        'launch_groot',
        default_value='true',
        description='Launch Groot for behavior tree visualization'
    )

    groot_prefix = get_package_prefix('groot')
    groot_executable = os.path.join(groot_prefix, 'lib', 'groot', 'Groot')

    groot_process = ExecuteProcess(
        cmd=[
            groot_executable,
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
        use_rectangle_path,
        esp_port,
        lidar_port,
        launch_groot_arg,

        mirs_hardware_launch,
        nav2_bringup_launch,
        camera_node,
        real_mission_launch,
        # delayed_landmark_localizer_node,  # COMMENTED OUT: landmark_localizer無効化
        rviz_node,
        groot_process
    ])
