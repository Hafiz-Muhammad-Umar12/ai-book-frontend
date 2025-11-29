## Chapter 1: Introduction to ROS 2 for Humanoid Robotics

Robot Operating System 2 (ROS 2) provides a flexible framework for writing robot software. For humanoid robotics, ROS 2 offers a robust set of tools, libraries, and conventions that facilitate the development, testing, and deployment of complex robotic applications. It enables modularity, allowing different functionalities (e.g., perception, planning, control, and human-robot interaction) to be developed and run as independent components, which then communicate seamlessly. Given the complexity of humanoid platforms, ROS 2's distributed architecture and real-time capabilities are invaluable for managing multiple sensors, actuators, and advanced algorithms efficiently, even under the memory constraints of platforms like the Jetson Orin Nano.

### 1.1 ROS 2 Concepts: Nodes, Topics, and Services

Understanding the core communication mechanisms in ROS 2 is fundamental to building any robotic system.

#### 1.1.1 Nodes

In ROS 2, a **Node** is an executable process that performs a specific task. Each node should ideally be responsible for a single, well-defined function. For instance, in a humanoid robot, you might have separate nodes for:
*   Reading joint encoder data.
*   Controlling a specific motor.
*   Processing camera feeds to detect objects.
*   Planning a walking gait.
*   Managing the robot's overall state.

Nodes can be written in various programming languages, with Python (using `rclpy`) and C++ (using `rclcpp`) being the most common. The modularity of nodes significantly enhances system robustness and maintainability, as failures in one node are less likely to bring down the entire robot system.

#### 1.1.2 Topics

**Topics** are the primary mechanism for asynchronous, one-way data streaming in ROS 2. They implement a publisher/subscriber model, where:
*   A **Publisher** node sends messages to a specific topic.
*   One or more **Subscriber** nodes can listen to that topic and receive the messages.

This communication is non-blocking, meaning publishers send data without waiting for confirmation from subscribers. Topics are ideal for continuous data streams such as sensor readings (e.g., IMU data, joint positions), control commands (e.g., desired joint velocities), or processed perception data (eg., detected object locations). Each topic has a defined **message type**, which dictates the structure of the data being transmitted (e.g., `sensor_msgs/msg/JointState`, `geometry_msgs/msg/Twist`).

#### 1.1.3 Services

**Services** provide a synchronous request/response communication model between nodes. Unlike topics, where data flows one-way, services involve a distinct interaction:
*   A **Server** node offers a particular service.
*   A **Client** node sends a request to the server and waits for a response.

This pattern is suitable for operations that require a specific action to be performed and a result to be returned, such as:
*   Triggering a robot to perform a specific action and confirm its completion (e.g., "move to position X").
*   Querying the robot's current status (e.g., "get battery level").
*   Requesting a complex calculation (e.g., inverse kinematics for a target pose).

Each service has a defined **service type**, which specifies the structure of both the request and the response messages.

### 1.2 "Hello Robot" Node Example with `rclpy`

Below is a simple Python node using `rclpy` that demonstrates the creation of a node and a publisher. This node will periodically publish a "Hello, Robot!" message to a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard message type for strings
import time

class HelloRobotPublisher(Node):
    """
    A simple ROS 2 node that publishes 'Hello, Robot!' messages to a topic.
    """

    def __init__(self) -> None:
        """
        Initializes the HelloRobotPublisher node.
        """
        super().__init__('hello_robot_publisher')
        self.publisher_ = self.create_publisher(String, 'hello_robot', 10)
        self.timer_ = self.create_timer(1.0, self.publish_hello_message) # Publish every 1 second
        self.get_logger().info('Hello Robot Publisher Node started and ready to publish.')

    def publish_hello_message(self) -> None:
        """
        Publishes a 'Hello, Robot!' message to the 'hello_robot' topic.
        """
        msg: String = String()
        msg.data = 'Hello, Robot! Current time: %f' % time.time()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args: list[str] | None = None) -> None:
    """
    Main function to initialize and spin the ROS 2 node.
    """
    rclpy.init(args=args)
    hello_robot_publisher_node: HelloRobotPublisher = HelloRobotPublisher()
    rclpy.spin(hello_robot_publisher_node)
    hello_robot_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

:::tip
**Sourcing Setup Script:** Before running any ROS 2 executable, including Python nodes, you **must** source the `setup.bash` file from your ROS 2 installation. This script sets up the necessary environment variables, allowing ROS 2 packages and commands to be found. Failing to do so will result in "command not found" errors or issues with node discovery. For example, in a terminal, you would typically run: `source /opt/ros/humble/setup.bash` (Linux) or `source install/setup.bash` from your workspace root.
:::