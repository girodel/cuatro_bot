from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

# ================== ENVIRONMENT SETUP =================== #

    #CHANGE THESE TO BE RELEVANT TO THE SPECIFIC PACKAGE
    robot_description_path = get_package_share_directory('my_robot_description')  
    robot_package = FindPackageShare('my_robot_description') 
    robot_name = 'tetrabot' 
    robot_urdf_file_name = 'my_robot.urdf.xacro'
    rviz_config_file_name = 'my_robo_maps.rviz'
    custom_world_file_name = 'my_prueba_world_h.sdf'

    parent_of_share_path = os.path.dirname(robot_description_path)

    # --- Set GZ_SIM_RESOURCE_PATH  ---
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            os.path.pathsep, 
            parent_of_share_path 
        ]
    )

    # --- Set GAZEBO_MODEL_PATH ---
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[
            os.environ.get('GAZEBO_MODEL_PATH', ''),
            os.path.pathsep,
            os.path.join(robot_description_path, 'models') 
        ]
    )

    # --- Use sim time setup ---
    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')


# ========================================================= #

# ======================== RVIZ ========================== #

    # Declare arguments
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=PathJoinSubstitution([
            robot_package,
            'urdf',
            robot_urdf_file_name
        ]),
        description='Path to the URDF file for the robot description.'
    )

    rviz_config_path_arg = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=PathJoinSubstitution([
            robot_package,
            'rviz',
            rviz_config_file_name
        ]),
        description='Path to the RViz configuration file.'
    )

    # Get the robot description from the URDF file
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('urdf_path')]),
        value_type=str
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
            'frame_prefix': robot_name + '/' 
        }]
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_path')],
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    #Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

# ========================================================= #


# ============== GAZEBO - SETUP AND LAUNCH ================ #
    
    # Include the Gazebo Sim launch file (using gz_sim.launch.py)
    gz_sim_launch_file = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])


    # Define the path to your custom world file
    custom_world_path = PathJoinSubstitution([
        robot_package,
        'worlds',
        custom_world_file_name
    ])

    # Construct the gz_args to the custom world as a single string
    gz_args_value = ['-r ', custom_world_path] #the '-r' makes the server run

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch_file]),
        launch_arguments={'gz_args': gz_args_value, 
                          'use_sim_time': use_sim_time,
                          'on_exit_shutdown': 'True' 
                          }.items()
    )

    # Reads the robot_description from the parameter server and spawns it.
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name, 
            '-topic', 'robot_description', 
            '-x', '0', 
            '-y', '0',
            '-z', '0.5'
        ],
        output='screen'
    )

# ========================================================= #


# ================= GAZEBO BRIDGES & SENSOR SETUP =================== #

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            f'/model/{robot_name}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # --- PUENTES PARA ULTRASONICOS ---
            # El formato es: /TOPICO@ROS_MSG_TYPE[GZ_MSG_TYPE
            '/ultrasonico/der@sensor_msgs/msg/Range[ignition.msgs.EntityRange',
            '/ultrasonico/iz@sensor_msgs/msg/Range[ignition.msgs.EntityRange',
            '/ultrasonico/tra@sensor_msgs/msg/Range[ignition.msgs.EntityRange'
        ],
        # 🔥 EL REMAPEO DE TF AQUÍ:
        remappings=[
            (f'/model/{robot_name}/tf', '/tf'),
        ]
    )

    # 🔥 EL ESLABÓN PERDIDO PARA CONECTAR EL ROBOT A SLAM:
    footprint_to_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='footprint_to_base_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 
                   f'{robot_name}/base_footprint', 
                   f'{robot_name}/base_link']
    )

    lidar_tf_publisher_node = Node(   
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_gpu_frame_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', '1', 
                   f'{robot_name}/lidar_link', 
                   f'{robot_name}/base_footprint/gpu_lidar'] 
    )


# ========================================================= #

    return LaunchDescription([
        urdf_path_arg,
        rviz_config_path_arg,
        use_sim_time_declare,
        set_gz_sim_resource_path, 
        set_gazebo_model_path, 
        footprint_to_base_node,       # <--- Agregado aquí
        lidar_tf_publisher_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        gazebo_launch,
        spawn_entity_node,
        ros_gz_bridge,
        rviz2_node
        ])