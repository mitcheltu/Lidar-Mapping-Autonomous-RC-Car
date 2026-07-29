"""motion_enable_node: the laptop GO/HOLD button for testing mode.

Publishes a latched std_msgs/Bool on /motion_enable that gates
motion_controller_node. Default HOLD -- the planner still computes and shows the
path (/cmd_path in RViz, /drive_intended shows the command), but the car does not
move until you press GO.

Run in its own terminal (it reads keystrokes):
    ros2 run autonomous_rc_car_ros motion_enable_node

Keys:  SPACE or g = GO (drive)   h = HOLD (stop)   q = quit
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('motion_enable_node')
    latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(Bool, '/motion_enable', latched)

    enabled = False
    pub.publish(Bool(data=enabled))
    sys.stdout.write('[motion_enable] HOLD. Keys: SPACE/g=GO  h=HOLD  q=quit\r\n')
    sys.stdout.flush()

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                continue
            c = sys.stdin.read(1)
            if c == ' ':
                enabled = not enabled
            elif c in ('g', 'G'):
                enabled = True
            elif c in ('h', 'H'):
                enabled = False
            elif c in ('q', 'Q', '\x03'):   # q or Ctrl-C
                break
            else:
                continue
            pub.publish(Bool(data=enabled))
            state = 'GO -- DRIVING' if enabled else 'HOLD -- stopped'
            sys.stdout.write(f'[motion_enable] {state}\r\n')
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        # Safety: command HOLD on exit so a quit never leaves the car enabled.
        try:
            pub.publish(Bool(data=False))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
