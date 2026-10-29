import sys, time, statistics, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60
rclpy.init()
n = Node('hzstat')
t = []
n.create_subscription(Image, '/hk_camera/rgb', lambda m: t.append(time.monotonic()), qos_profile_sensor_data)
start = None
while rclpy.ok():
    rclpy.spin_once(n, timeout_sec=0.1)
    if t and start is None: start = t[0]
    if start and time.monotonic() - start > dur: break
d = [b - a for a, b in zip(t, t[1:])]
print(f"msgs={len(t)} span={t[-1]-t[0]:.1f}s avg_rate={(len(t)-1)/(t[-1]-t[0]):.2f}Hz min={min(d):.3f} max={max(d):.3f} std={statistics.pstdev(d):.4f} gaps>0.1s={sum(x>0.1 for x in d)} gaps>0.3s={sum(x>0.3 for x in d)}")
print("largest gaps:", sorted(round(x,3) for x in d)[-8:])
