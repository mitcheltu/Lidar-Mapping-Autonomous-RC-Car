"""motion_enable_node: laptop control + status console for testing.

One terminal to run the whole test loop by hand:
  - calibrate the car against the phone's pose (c),
  - request a single plan (the planner computes one /cmd_path, shown in RViz),
  - enable / disable driving (GO / HOLD),
  - watch the car link, the planned path size and the live drive command.

Run in its own terminal (it reads keystrokes):
    ros2 run autonomous_rc_car_ros motion_enable_node

Keys:  c = calibrate   p = plan now   g = GO (drive)   h or SPACE = HOLD/abort
       q = quit
Default is HOLD; the car never moves until you press g or c.
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
    calibrate_pub = node.create_publisher(Empty, '/calibrate_trigger', 10)

    state = {'enabled': False, 'waypoints': None, 'cmd': '-',
             'link': '?', 'cal': '', 'calibrating': False}
    node.create_subscription(Path, '/cmd_path',
                             lambda m: state.update(waypoints=len(m.poses)), latched)
    node.create_subscription(String, '/drive_intended',
                             lambda m: state.update(cmd=m.data), 10)
    node.create_subscription(String, '/car_link',
                             lambda m: state.update(link=m.data), latched)
    node.create_subscription(String, '/calibration_status',
                             lambda m: state.update(cal=m.data), latched)
    node.create_subscription(Bool, '/calibration_active',
                             lambda m: state.update(calibrating=m.data), latched)

    enable_pub.publish(Bool(data=False))

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    def show():
        if state['calibrating']:
            # During calibration the stage text is the only thing worth seeing.
            line = f"\r[CAL ] {state['cal'][:96]:<96} (h/SPACE=abort)   "
        else:
            st = 'GO  ' if state['enabled'] else 'HOLD'
            wp = f"{state['waypoints']} wp" if state['waypoints'] is not None else 'no path'
            link = state['link'].split()[0] if state['link'] else '?'
            line = (f"\r[{st}] car: {link:<12} | path: {wp:>8} | drive: {state['cmd']:>10}   "
                    f"(c=calibrate  p=plan  g=go  h/SPACE=hold  q=quit)   ")
        sys.stdout.write(line)
        sys.stdout.flush()

    try:
        tty.setraw(fd)
        show()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            ready, _, _ = select.select([sys.stdin], [], [], 0.15)
            if ready:
                c = sys.stdin.read(1)
                if c == 'p' and not state['calibrating']:
                    plan_pub.publish(Empty())
                elif c in ('c', 'C') and not state['calibrating']:
                    # The car drives itself from here; HOLD first so the motion
                    # controller is not also commanding it.
                    state['enabled'] = False
                    enable_pub.publish(Bool(data=False))
                    calibrate_pub.publish(Empty())
                elif c in ('g', 'G') and not state['calibrating']:
                    state['enabled'] = True
                    enable_pub.publish(Bool(data=True))
                elif c in ('h', 'H', ' '):
                    state['enabled'] = False
                    enable_pub.publish(Bool(data=False))   # also aborts calibration
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
