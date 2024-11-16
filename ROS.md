# Comprehensive ROS2 Interview Cheatsheet

## Core Concepts and Architecture

### 1. ROS2 vs ROS1 Key Differences
- **DDS (Data Distribution Service)**
  - Reliable peer-to-peer communication
  - Built-in discovery service
  - Configurable Quality of Service (QoS)
  - No single point of failure
- **Improved Security**
  - SROS2 (Secure ROS2)
  - Authentication and encryption
  - Access control policies
- **Real-time Capabilities**
  - Deterministic behavior
  - Real-time operating system support
  - Priority inheritance
- **Multi-robot Support**
  - Built-in discovery across networks
  - Domain ID separation
  - Improved namespace handling
- **Modern Features**
  - Python 3 support
  - Modern C++ (14, 17) features
  - Improved build system (colcon)

### 2. Communication Patterns

#### Topics (Many-to-Many)
```python
# Publisher
self.publisher_ = self.create_publisher(
    msg_type=String,
    topic='topic_name',
    qos_profile=10
)

# Subscriber
self.subscription = self.create_subscription(
    msg_type=String,
    topic='topic_name',
    callback=self.listener_callback,
    qos_profile=10
)
```

#### Services (One-to-One)
```python
# Service Server
self.srv = self.create_service(
    srv_type=AddTwoInts,
    srv_name='add_two_ints',
    callback=self.add_two_ints_callback
)

# Service Client
self.client = self.create_client(
    srv_type=AddTwoInts,
    srv_name='add_two_ints'
)
```

#### Actions (Long-running tasks)
```python
# Action Server
self._action_server = ActionServer(
    self,
    Fibonacci,
    'fibonacci',
    self.execute_callback
)

# Action Client
self._action_client = ActionClient(
    self,
    Fibonacci,
    'fibonacci'
)
```

#### Parameters
```python
# Declare parameters
self.declare_parameter('my_parameter', 'default_value')

# Get parameter
my_param = self.get_parameter('my_parameter').value

# Set parameter
self.set_parameters([Parameter('my_parameter', value='new_value')])
```

### 3. Quality of Service (QoS) Profiles

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Custom QoS Profile
custom_qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# Common QoS Profiles
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_services_default
from rclpy.qos import qos_profile_parameters
```

## Advanced Commands and Operations

### Workspace Management
```bash
# Create and build workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install  # Symlink for Python packages
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-select my_package --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Clean build
colcon clean packages
rm -rf build install log
```

### Package Management
```bash
# Create packages
ros2 pkg create --build-type ament_cmake --node-name my_node my_cpp_pkg
ros2 pkg create --build-type ament_python --node-name my_node my_py_pkg

# List dependencies
ros2 pkg list --depends-on rclcpp
ros2 pkg xml my_package  # Show package.xml

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Launch File Examples

#### Basic Launch File (Python)
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='custom_node_name',
            parameters=[{'param1': 42}],
            remappings=[('/old_topic', '/new_topic')],
            output='screen'
        )
    ])
```

#### Advanced Launch File
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='robot1',
            description='Namespace for the robot'
        ),
        
        # Group nodes under namespace
        GroupAction([
            PushRosNamespace(namespace),
            
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
            ),
            
            Node(
                package='my_package',
                executable='my_node',
                parameters=[{
                    'param1': 42,
                    'param2': 'value'
                }]
            )
        ])
    ])
```

### Advanced Code Examples

#### Lifecycle Node
```python
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleNode(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Initialize hardware, set up publishers/subscribers
        return TransitionCallbackReturn.SUCCESS
        
    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Start processing, enable outputs
        return TransitionCallbackReturn.SUCCESS
        
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Stop processing, disable outputs
        return TransitionCallbackReturn.SUCCESS
        
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # Clean up resources
        return TransitionCallbackReturn.SUCCESS
        
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        # Clean up resources
        return TransitionCallbackReturn.SUCCESS
```

#### Component Nodes
```python
from rclpy.node import Node
from rclpy.component.component import ComponentFactory

class MyComponent(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        
def main(args=None):
    rclpy.init(args=args)
    component = MyComponent('my_component')
    rclpy.spin(component)
    component.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Testing in ROS2

#### Unit Tests
```python
import unittest
from my_package.my_node import MyNode

class TestMyNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = MyNode()
        
    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        
    def test_something(self):
        # Your test code here
        pass
```

#### Launch Testing
```python
import unittest
import launch_testing
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_test_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
        ),
        launch_testing.actions.ReadyToTest()
    ])

class TestMyNode(unittest.TestCase):
    def test_node_output(self, proc_output):
        proc_output.assertWaitFor('Expected output')
```

## Important Interview Topics

### 1. ROS2 Navigation Stack (Nav2)
- BehaviorTrees for task execution
- Costmap2D for obstacle avoidance
- Global and local planners
- Recovery behaviors
- SLAM integration

### 2. TF2 (Transform Library)
```python
# Broadcasting transforms
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

tf = TransformStamped()
tf.header.stamp = self.get_clock().now().to_msg()
tf.header.frame_id = 'world'
tf.child_frame_id = 'base_link'
self.tf_broadcaster.sendTransform(tf)

# Listening to transforms
from tf2_ros import Buffer, TransformListener
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)
transform = self.tf_buffer.lookup_transform(
    'world',
    'base_link',
    rclpy.time.Time()
)
```

### 3. URDF and Robot Description
```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.2 0.3 0.1"/>
            </geometry>
        </visual>
    </link>
    
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
    </joint>
</robot>
```

### 4. Custom Message/Service Definitions
```
# Custom message definition (my_msgs/msg/CustomMsg.msg)
string data
int32 count
geometry_msgs/Pose pose

# Custom service definition (my_msgs/srv/CustomSrv.srv)
string request_data
---
bool success
string response_data
```

### 5. ROS2 Tools
- **rqt_graph**: Visualize node connections
- **rviz2**: 3D visualization
- **ros2 doctor**: System diagnostics
- **ros2 bag**: Record and replay data
- **ros2 topic hz**: Message frequency
- **ros2 topic bw**: Bandwidth usage

## Common Interview Questions and Answers

1. **How does ROS2 handle real-time systems better than ROS1?**
   - DDS middleware with configurable QoS
   - No central master reduces latency
   - Better support for real-time operating systems
   - Priority inheritance for handling priority inversion

2. **Explain the ROS2 node discovery process**
   - DDS Discovery and Data-Distribution Protocol (DDS-RTPS)
   - Nodes announce presence using multicast
   - Exchange QoS profiles and topic information
   - Establish peer-to-peer connections

3. **What are the different QoS policies in ROS2?**
   - Reliability (RELIABLE vs BEST_EFFORT)
   - History (KEEP_LAST vs KEEP_ALL)
   - Durability (VOLATILE vs TRANSIENT_LOCAL)
   - Deadline
   - Liveliness

4. **How do you handle large message transfers in ROS2?**
   - Use appropriate QoS settings
   - Consider message fragmentation
   - Use services for large one-time transfers
   - Implement streaming for continuous large data

5. **Explain the lifecycle node concept**
   - Managed nodes with defined states
   - Configurable, cleanable, activatable
   - Better resource management
   - Improved error handling

## Best Practices

1. **Code Organization**
   - One node per file
   - Clear separation of concerns
   - Use composition when appropriate
   - Follow ROS2 naming conventions

2. **Resource Management**
   - Use RAII principles
   - Properly handle node cleanup
   - Monitor system resources
   - Use lifecycle nodes for complex systems

3. **Error Handling**
   - Use try-except blocks appropriately
   - Implement proper logging
   - Handle timeout scenarios
   - Implement recovery behaviors

4. **Performance Optimization**
   - Choose appropriate QoS profiles
   - Monitor message latency
   - Use efficient data structures
   - Implement message filtering when needed

Remember to always relate these concepts to practical experiences in your interviews!

## Advanced Navigation Stack (Nav2)

### 1. Key Components
```python
# Basic Navigation Node
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    navigator.setInitialPose(initial_pose)
    
    # Wait for Nav2
    navigator.waitUntilNav2Active()
    
    # Navigate to pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 2.0
    navigator.goToPose(goal_pose)

```

### 2. Navigation Parameters
```yaml
# nav2_params.yaml
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_slow: 0.001
    recovery_alpha_fast: 0.1
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    plugin_lib_names:
    - nav2_compute_path_to_pose_action_bt_node
    - nav2_follow_path_action_bt_node
    - nav2_back_up_action_bt_node
    - nav2_spin_action_bt_node
    - nav2_wait_action_bt_node
    - nav2_clear_costmap_service_bt_node
    - nav2_is_stuck_condition_bt_node
    - nav2_goal_reached_condition_bt_node
    - nav2_initial_pose_received_condition_bt_node
    - nav2_reinitialize_global_localization_service_bt_node
    - nav2_rate_controller_bt_node
    - nav2_distance_controller_bt_node
    - nav2_speed_controller_bt_node
    - nav2_recovery_node_bt_node
    - nav2_pipeline_sequence_bt_node
    - nav2_round_robin_node_bt_node
    - nav2_transform_available_condition_bt_node
    - nav2_time_expired_condition_bt_node
    - nav2_distance_traveled_condition_bt_node

controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
```

### 3. SLAM Integration

#### 1. Using SLAM Toolbox
```bash
# Launch SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# Save map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "name:
    data: 'my_map'"
```

#### 2. Custom SLAM Node
```python
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np

class SimpleSLAM(Node):
    def __init__(self):
        super().__init__('simple_slam')
        
        # Create map publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            'map',
            10
        )
        
        # Subscribe to laser scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Initialize map
        self.map = OccupancyGrid()
        self.map.info.resolution = 0.05  # 5cm per pixel
        self.map.info.width = 1000
        self.map.info.height = 1000
        self.map.data = [-1] * (self.map.info.width * self.map.info.height)
```

## MoveIt2 Integration

### 1. Basic Setup
```python
from moveit2 import (
    MoveGroupInterface,
    PlanningSceneInterface,
    RobotCommander
)

class RobotManipulation(Node):
    def __init__(self):
        super().__init__('robot_manipulation')
        
        # Initialize MoveIt2 interfaces
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.move_group = MoveGroupInterface("arm")
        
        # Set planning parameters
        self.move_group.set_planning_time(5.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.1)
```

### 2. Planning Scene
```python
# Add box obstacle
box_pose = geometry_msgs.msg.PoseStamped()
box_pose.header.frame_id = "world"
box_pose.pose.position.x = 1.0
scene.add_box("obstacle", box_pose, (0.5, 0.5, 0.5))

# Add collision object
collision_object = moveit_msgs.msg.CollisionObject()
collision_object.header.frame_id = "world"
collision_object.id = "collision_object"
# ... set geometry, pose, etc.
scene.add_object(collision_object)
```

### 3. Motion Planning
```python
# Plan to joint target
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 1.57  # 90 degrees
success = move_group.go(joint_goal, wait=True)

# Plan to pose target
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.5
pose_goal.position.y = 0.0
pose_goal.position.z = 0.5
move_group.set_pose_target(pose_goal)
success = move_group.go(wait=True)
```

## Multi-Robot Coordination

### 1. Namespace Management
```python
class RobotNode(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_node')
        
        # Use namespaced topics
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            f'/{robot_name}/pose',
            self.pose_callback,
            10
        )
```

### 2. Fleet Management
```python
class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        
        self.robots = {}
        self.robot_states = {}
        
        # Discovery service
        self.create_service(
            RegisterRobot,
            'register_robot',
            self.register_robot_callback
        )
        
    def register_robot_callback(self, request, response):
        robot_name = request.robot_name
        self.robots[robot_name] = {
            'status': 'idle',
            'position': None,
            'task_queue': []
        }
        response.success = True
        return response
```

## Security (SROS2)

### 1. Key Management
```bash
# Generate keystore
ros2 security create_keystore ~/ros2_security

# Generate keys for node
ros2 security create_key ~/ros2_security /namespace/node_name

# Create permissions
ros2 security create_permission ~/ros2_security /namespace/node_name policies/policy.xml
```

### 2. Policy Definition
```xml
<?xml version="1.0" encoding="UTF-8"?>
<policy version="0.2.0">
  <enclaves>
    <enclave path="/">
      <profiles>
        <profile node="/**/my_node">
          <topics publish="ALLOW" subscribe="ALLOW">
            <topic>cmd_vel</topic>
            <topic>scan</topic>
          </topics>
          <services reply="ALLOW" request="ALLOW">
            <service>set_pose</service>
          </services>
        </profile>
      </profiles>
    </enclave>
  </enclaves>
</policy>
```

## Advanced Networking

### 1. DDS Configuration
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>CustomUDPTransport</transport_id>
                <type>UDPv4</type>
                <maxMessageSize>65000</maxMessageSize>
                <non_blocking_send>true</non_blocking_send>
            </transport_descriptor>
        </transport_descriptors>
        
        <participant profile_name="CustomParticipant">
            <rtps>
                <userTransports>
                    <transport_id>CustomUDPTransport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

### 2. Network Bridge
```python
from ros2_network_bridge import NetworkBridge

class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')
        
        self.bridge = NetworkBridge(
            local_ip="192.168.1.100",
            remote_ip="192.168.1.200",
            topics=[
                ("/cmd_vel", "geometry_msgs/Twist"),
                ("/scan", "sensor_msgs/LaserScan")
            ]
        )
```

## Performance Optimization

### 1. Message Filtering
```python
from message_filters import Subscriber, TimeSynchronizer

class SynchronizedNode(Node):
    def __init__(self):
        super().__init__('synchronized_node')
        
        # Create subscribers with message filters
        self.image_sub = Subscriber(self, Image, 'image')
        self.depth_sub = Subscriber(self, Image, 'depth')
        
        # Synchronize messages
        self.ts = TimeSynchronizer(
            [self.image_sub, self.depth_sub],
            queue_size=10
        )
        self.ts.registerCallback(self.synchronized_callback)
```

### 2. Memory Management
```python
class OptimizedNode(Node):
    def __init__(self):
        super().__init__('optimized_node')
        
        # Pre-allocate messages
        self.msg_pool = [Twist() for _ in range(10)]
        self.pool_index = 0
        
    def get_message(self):
        msg = self.msg_pool[self.pool_index]
        self.pool_index = (self.pool_index + 1) % len(self.msg_pool)
        return msg
```

### 3. Performance Monitoring
```python
import psutil
import time

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        self.create_timer(1.0, self.monitor_callback)
        
    def monitor_callback(self):
        # CPU usage
        cpu_percent = psutil.cpu_percent()
        
        # Memory usage
        memory = psutil.Process().memory_info()
        
        # Message statistics
        topic_info = self.get_publisher_names_and_types_by_node(
            self.get_name(),
            ''
        )
        
        self.get_logger().info(
            f'CPU: {cpu_percent}%, Memory: {memory.rss / 1024 / 1024}MB'
        )
        
        # Monitor message latency
        self._monitor_latency()
        
        # Monitor network bandwidth
        self._monitor_bandwidth()
        
    def _monitor_latency(self):
        start_time = self.get_clock().now()
        
        # Create test message
        test_msg = String()
        test_msg.data = 'latency_test'
        
        # Publish and wait for echo
        self.test_pub.publish(test_msg)
        
        try:
            msg = self.test_sub.receive_message(timeout=1.0)
            end_time = self.get_clock().now()
            latency = (end_time - start_time).nanoseconds / 1e6  # Convert to ms
            self.get_logger().info(f'Message latency: {latency}ms')
        except TimeoutError:
            self.get_logger().warning('Message timeout')
            
    def _monitor_bandwidth(self):
        # Get network stats
        net_io = psutil.net_io_counters()
        current_time = time.time()
        
        if hasattr(self, '_last_net_io'):
            time_delta = current_time - self._last_time
            bytes_sent_delta = net_io.bytes_sent - self._last_net_io.bytes_sent
            bytes_recv_delta = net_io.bytes_recv - self._last_net_io.bytes_recv
            
            # Calculate bandwidth in MB/s
            send_rate = bytes_sent_delta / time_delta / 1024 / 1024
            recv_rate = bytes_recv_delta / time_delta / 1024 / 1024
            
            self.get_logger().info(
                f'Network bandwidth - Send: {send_rate:.2f}MB/s, Recv: {recv_rate:.2f}MB/s'
            )
            
        self._last_net_io = net_io
        self._last_time = current_time

## Vision Integration

### 1. OpenCV Integration
```python
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        
        # Image subscriber
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # Processed image publisher
        self.processed_pub = self.create_publisher(
            Image,
            'processed_image',
            10
        )
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Process image
            processed = self.process_image(cv_image)
            
            # Convert back to ROS Image and publish
            processed_msg = self.bridge.cv2_to_imgmsg(processed, 'bgr8')
            self.processed_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
            
    def process_image(self, image):
        # Example processing pipeline
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 1.5)
        edges = cv2.Canny(blurred, 100, 200)
        return edges
```

### 2. Point Cloud Processing
```python
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

class PointCloudProcessor(Node):
    def __init__(self):
        super().__init__('pointcloud_processor')
        
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            'input_cloud',
            self.cloud_callback,
            10
        )
        
        self.processed_pub = self.create_publisher(
            PointCloud2,
            'processed_cloud',
            10
        )
        
    def cloud_callback(self, cloud_msg):
        # Convert to numpy array
        points = np.array(list(pc2.read_points(cloud_msg, skip_nans=True)))
        
        # Process point cloud
        processed_points = self.process_points(points)
        
        # Convert back to PointCloud2
        processed_msg = pc2.create_cloud_xyz32(
            cloud_msg.header,
            processed_points
        )
        self.processed_pub.publish(processed_msg)
        
    def process_points(self, points):
        # Example: Remove points beyond certain distance
        distances = np.linalg.norm(points[:, :3], axis=1)
        mask = distances < 10.0  # 10 meters
        return points[mask]
```

## Custom Sensor Integration

### 1. Custom Message Definition
```
# MyCustomSensor.msg
Header header
float64 sensor_value
float64[] raw_data
string status
```

### 2. Sensor Driver Node
```python
from my_interfaces.msg import MyCustomSensor
import serial

class SensorDriver(Node):
    def __init__(self):
        super().__init__('sensor_driver')
        
        # Serial connection
        self.serial_port = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=1.0
        )
        
        # Publisher
        self.sensor_pub = self.create_publisher(
            MyCustomSensor,
            'sensor_data',
            10
        )
        
        # Timer for reading sensor
        self.timer = self.create_timer(0.1, self.read_sensor)
        
    def read_sensor(self):
        try:
            # Read from serial
            data = self.serial_port.readline().decode().strip()
            
            # Parse data
            values = [float(x) for x in data.split(',')]
            
            # Create and publish message
            msg = MyCustomSensor()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.sensor_value = values[0]
            msg.raw_data = values[1:]
            msg.status = 'OK'
            
            self.sensor_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error reading sensor: {str(e)}')
```

## Docker Integration

### 1. Dockerfile Example
```dockerfile
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
COPY . /ros2_ws/src/my_package/

# Build workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Source workspace in bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["ros2", "launch", "my_package", "my_launch.py"]
```

### 2. Docker Compose
```yaml
version: '3'
services:
  ros2_node:
    build: .
    network_mode: host
    environment:
      - DISPLAY=$DISPLAY
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - ~/.ros:/root/.ros
    devices:
      - /dev/video0:/dev/video0
```

## Testing Best Practices

### 1. Component Tests
```python
from unittest import TestCase
import rclpy
from rclpy.node import Node
from my_package.my_component import MyComponent

class TestMyComponent(TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_node')
        cls.component = MyComponent(cls.node)
        
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        
    def test_component_initialization(self):
        self.assertIsNotNone(self.component)
        self.assertEqual(self.component.get_parameter('param').value, 'default')
        
    def test_component_callback(self):
        test_msg = String()
        test_msg.data = 'test'
        self.component.callback(test_msg)
        # Assert expected behavior
```

### 2. Integration Tests
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
import launch_testing.markers
import pytest

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            parameters=[{'param': 'test_value'}]
        ),
        ReadyToTest()
    ])
    
class TestNodeIntegration:
    def test_node_output(self, proc_output):
        proc_output.assertWaitFor('Node started', timeout=10)
```

## Error Handling and Recovery

### 1. Error Handling Patterns
```python
class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.retry_count = 0
        self.max_retries = 3
        
    def handle_operation(self):
        try:
            # Attempt operation
            result = self.perform_operation()
            self.retry_count = 0
            return result
            
        except TimeoutError:
            self.get_logger().warn('Operation timeout')
            if self.retry_count < self.max_retries:
                self.retry_count += 1
                return self.handle_operation()
            else:
                self.get_logger().error('Max retries exceeded')
                raise
                
        except Exception as e:
            self.get_logger().error(f'Unexpected error: {str(e)}')
            self.enter_failure_mode()
            raise
            
    def enter_failure_mode(self):
        # Implement safety measures
        self.publish_error_status()
        self.stop_motors()
        self.activate_safety_mode()
```

### 2. Recovery Behaviors
```python
class RecoveryManager(Node):
    def __init__(self):
        super().__init__('recovery_manager')
        self.recovery_steps = [
            self.clear_costmaps,
            self.reinit_localization,
            self.restart_navigation,
            self.emergency_stop
        ]
        
    async def handle_failure(self):
        for step in self.recovery_steps:
            try:
                await step()
                if self.check_recovery():
                    self.get_logger().info('Recovery successful')
                    return True
            except Exception as e:
                self.get_logger().error(f'Recovery step failed: {str(e)}')
                continue
        return False
```
