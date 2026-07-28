"""Back-compat shim. The wire protocol moved into the `nav` library
(`nav/stream_protocol.py`) so the pip-installed ROS2 package can share it.
Import from `nav.stream_protocol` in new code; this re-export keeps older
imports (`from nodes.stream_protocol import ...`) working."""

from nav.stream_protocol import *  # noqa: F401,F403
from nav.stream_protocol import __all__  # noqa: F401
