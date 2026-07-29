"""motion_enable_node: laptop control + status console for testing.

One terminal to run the whole test loop by hand:
  - request a single plan (the planner computes one /cmd_path, shown in RViz),
  - enable / disable driving (GO / HOLD),
  - watch the planned path size and the live drive command.

Run in its own terminal (it reads keystrokes):
    ros2 run autonomous_rc_car_ros motion_enable_node

Keys:  p = plan now    g = GO (drive)    h or SPACE = HOLD (stop)    q = quit
Default is HOLD; the car never moves until you press g.
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Path
from std_msgs.msg import Bool, Empty, String


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('motion_enable_node')

    latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    enable_pub = node.create_publisher(Bool, '/motion_enable', latched)
    plan_pub = node.create_publisher(Empty, '/plan_trigger', 10)

    state = {'enabled': False, 'waypoints': None, 'cmd': '-'}
    node.create_subscription(Path, '/cmd_path',
                             lambda m: state.update(waypoints=len(m.poses)), latched)
    node.create_subscription(String, '/drive_intended',
                             lambda m: state.update(cmd=m.data), 10)

    enable_pub.publish(Bool(data=False))

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    def show():
        st = 'GO  ' if state['enabled'] else 'HOLD'
        wp = f"{state['waypoints']} wp" if state['waypoints'] is not None else 'no path'
        sys.stdout.write(
            f"\r[{st}] path: {wp:>8} | drive: {state['cmd']:>10}   "
            f"(p=plan  g=go  h/SPACE=hold  q=quit)   ")
        sys.stdout.flush()

    try:
        tty.setraw(fd)
        show()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([sys.stdin], [], [], 0.15)
            if ready:
                c = sys.stdin.read(1)
                if c == 'p':
                    plan_pub.publish(Empty())
                elif c in ('g', 'G'):
                    state['enabled'] = True
                    enable_pub.publish(Bool(data=True))
                elif c in ('h', 'H', ' '):
                    state['enabled'] = False
                    enable_pub.publish(Bool(data=False))
                elif c in ('q', 'Q', '\x03'):
                    break
            show()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        try:
            enable_pub.publish(Bool(data=False))   # never leave the car enabled on exit
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
