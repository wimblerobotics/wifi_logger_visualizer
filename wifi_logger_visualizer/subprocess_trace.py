"""Log subprocess argument vectors immediately before execution."""
import shlex


def trace_command(command, trace=None):
    """Return arguments unchanged after optionally logging a shell-quoted command."""
    if trace is not None:
        trace(shlex.join(command))
    return command


def node_subprocess_trace(node):
    """Create a callback that honors the node's current tracing parameter."""
    def trace(command):
        if node.get_parameter("trace_subprocess_runs").value:
            node.get_logger().info(f"subprocess: {command}")
    return trace
