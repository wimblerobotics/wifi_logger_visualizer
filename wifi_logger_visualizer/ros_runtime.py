"""Shared ROS entry-point lifecycle for the Wi-Fi nodes.

Ctrl+C and a context shut down by ROS launch are expected termination paths.
Node-specific destroy_node methods still release databases and worker pools;
try_shutdown avoids a second-shutdown error after ROS's signal handler runs.
Unexpected initialization, callback, and cleanup errors remain visible.
"""
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.exceptions import InvalidHandle
from rclpy.impl.implementation_singleton import rclpy_implementation


def run_node(node_factory, args=None):
    """Construct and spin one node, then clean up on normal exit or failure."""
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        # Avoid lazily constructing rclpy's global executor on the first spin.
        # Own its lifetime so callbacks stop before node resources are released.
        executor = SingleThreadedExecutor()
        node = node_factory()
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (rclpy_implementation.RCLError, InvalidHandle):
        # A launch SIGTERM may invalidate the context between the executor's
        # context.ok() check and native wait-set creation. Only suppress that
        # teardown race; an RCL error with a live context is still a failure.
        if rclpy.ok():
            raise
    finally:
        try:
            if executor is not None:
                executor.shutdown()
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                rclpy.try_shutdown()
