"""Simple Node to allow users of crisp_py (https://github.com/utiasDSL/crisp_py) to use the Robotiq 2F-85 gripper."""

from time import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class RobotiqGripperClient:
    """Client for controlling the Robotiq 2F-85 gripper via GripperCommand action."""

    # Robotiq 2F-85 position limits (in radians for the knuckle joint)
    POSITION_OPEN = 0.0
    POSITION_CLOSED = 0.8

    def __init__(
        self,
        node: Node,
        gripper_namespace: str = "gripper",
        controller_name: str = "robotiq_gripper_controller",
    ):
        """Initialize the robotiq gripper client.

        Args:
            node: The ROS2 node to use for communication.
            gripper_namespace: The namespace for the gripper (default: "gripper").
            controller_name: The name of the gripper controller (default: "robotiq_gripper_controller").
        """
        self._node = node
        self._gripper_namespace = gripper_namespace
        self._controller_name = controller_name

        # Action client for gripper commands
        action_topic = f"/{gripper_namespace}/{controller_name}/gripper_cmd"
        self._gripper_action_client = ActionClient(
            node,
            GripperCommand,
            action_topic,
            callback_group=ReentrantCallbackGroup(),
        )
        node.get_logger().info(f"Gripper action client created for: {action_topic}")

        # Subscribe to joint states to get gripper position
        joint_state_topic = f"/{gripper_namespace}/joint_states"
        self._joint_state_subscriber = node.create_subscription(
            JointState,
            joint_state_topic,
            self._joint_state_callback,
            qos_profile_system_default,
        )
        node.get_logger().info(f"Subscribed to joint states: {joint_state_topic}")

        self._position = None
        self._goal_handle = None

    @property
    def position(self) -> float | None:
        """Returns the current position of the gripper or None if not initialized."""
        return self._position

    @property
    def width(self) -> float | None:
        """Returns the current width of the gripper (normalized 0-1) or None if not initialized.

        0 = fully closed, 1 = fully open (compatible with crisp_py interface)
        """
        if self._position is None:
            return None
        # Convert from joint position to normalized width
        # Position 0.0 = open (width 1.0), Position 0.8 = closed (width 0.0)
        return 1.0 - (self._position / self.POSITION_CLOSED)

    def is_open(self, open_threshold: float = 0.7) -> bool:
        """Returns True if the gripper is open."""
        return self.width is not None and self.width > open_threshold

    def is_ready(self) -> bool:
        """Returns True if the gripper is fully ready to operate."""
        return self._position is not None and self._gripper_action_client.server_is_ready()

    def wait_until_ready(self, timeout_sec: float = 10.0):
        """Waits until the gripper is fully ready to operate."""
        time_start = time()

        # Wait for action server
        self._node.get_logger().info("Waiting for gripper action server...")
        while not self._gripper_action_client.wait_for_server(timeout_sec=1.0):
            if time() - time_start > timeout_sec:
                raise TimeoutError("Gripper action server not available after timeout.")
            self._node.get_logger().info("Still waiting for gripper action server...")

        # Wait for joint state
        self._node.get_logger().info("Waiting for gripper joint state...")
        while self._position is None:
            rclpy.spin_once(self._node, timeout_sec=1.0)
            if time() - time_start > timeout_sec:
                raise TimeoutError("Gripper joint state not received after timeout.")

        self._node.get_logger().info("Gripper is ready!")

    def _joint_state_callback(self, msg: JointState):
        """Updates the gripper position using the current joint state."""
        # Look for the robotiq knuckle joint
        for i, name in enumerate(msg.name):
            if "knuckle" in name.lower() or "finger" in name.lower():
                self._position = msg.position[i]
                return

        # Fallback: use first joint position
        if len(msg.position) > 0:
            self._position = msg.position[0]

    def send_gripper_command(
        self,
        position: float,
        max_effort: float = 50.0,
        block: bool = False,
    ):
        """Send a gripper command.

        Args:
            position: Target position (0.0 = open, 0.8 = closed).
            max_effort: Maximum effort/force to apply.
            block: Whether to wait for the action to complete.
        """
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        future = self._gripper_action_client.send_goal_async(goal)

        if block:
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
            goal_handle = future.result()
            if goal_handle is not None and goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self._node, result_future, timeout_sec=10.0)

    def open(self, max_effort: float = 50.0, block: bool = False):
        """Open the gripper."""
        self.send_gripper_command(self.POSITION_OPEN, max_effort, block)

    def close(self, max_effort: float = 50.0, block: bool = False):
        """Close the gripper."""
        self.send_gripper_command(self.POSITION_CLOSED, max_effort, block)

    def move_to_width(self, width: float, max_effort: float = 50.0, block: bool = False):
        """Move gripper to a normalized width (0 = closed, 1 = open)."""
        # Convert normalized width to position
        position = (1.0 - width) * self.POSITION_CLOSED
        self.send_gripper_command(position, max_effort, block)

    def toggle(self, max_effort: float = 50.0, block: bool = False):
        """Toggle the gripper between open and closed."""
        if self.is_open():
            self.close(max_effort, block)
        else:
            self.open(max_effort, block)


class CrispPyRobotiqAdapter(Node):
    """Adapter node to allow crisp_py to control the Robotiq 2F-85 gripper."""

    def __init__(self):
        super().__init__("crisp_py_robotiq_adapter")

        # Declare parameters
        self.declare_parameter("gripper_namespace", "gripper")
        self.declare_parameter("controller_name", "robotiq_gripper_controller")
        self.declare_parameter("command_topic", "gripper/gripper_position_controller/commands")
        self.declare_parameter("joint_state_topic", "gripper/gripper_state")
        self.declare_parameter("joint_state_freq", 50)

        gripper_namespace = self.get_parameter("gripper_namespace").value
        controller_name = self.get_parameter("controller_name").value
        self.command_topic = self.get_parameter("command_topic").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.joint_state_freq = self.get_parameter("joint_state_freq").value

        self.gripper_client = RobotiqGripperClient(
            self,
            gripper_namespace=gripper_namespace,
            controller_name=controller_name,
        )
        self.gripper_client.wait_until_ready()

        self.gripper_client.open()
        self.is_closing = False

        self.create_subscription(
            Float64MultiArray,
            self.command_topic,
            self.callback_command,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        self.joint_state_publisher = self.create_publisher(
            JointState,
            self.joint_state_topic,
            qos_profile_system_default,
            callback_group=ReentrantCallbackGroup(),
        )

        self.create_timer(1 / self.joint_state_freq, self.callback_publish_joint_state)
        self.get_logger().info("The crisp_py robotiq adapter started.")

    def callback_publish_joint_state(self):
        """Publish gripper joint state for crisp_py."""
        if self.gripper_client.width is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper_joint"]
        msg.position = [self.gripper_client.width]  # Normalized 0-1
        msg.effort = [0.0]

        self.joint_state_publisher.publish(msg)

    def callback_command(self, msg: Float64MultiArray):
        """Callback to handle gripper commands from crisp_py.

        Expects a value between 0 and 1:
        - 0 = close gripper
        - 1 = open gripper
        """
        if len(msg.data) == 0:
            return

        gripper_command = msg.data[0]

        if (
            gripper_command <= 0.5
            and self.gripper_client.is_open()
            and not self.is_closing
        ):
            self.gripper_client.close()
            self.is_closing = True
        elif (
            gripper_command > 0.5
            and not self.gripper_client.is_open()
            and self.is_closing
        ):
            self.gripper_client.open()
            self.is_closing = False


def main():
    rclpy.init()
    adapter = CrispPyRobotiqAdapter()
    rclpy.spin(adapter)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
