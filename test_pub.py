import rclpy
import time
from rclpy.node import Node
from gz.transport13 import Node as GzNode
from gz.msgs10.entity_wrench_pb2 import EntityWrench

gz_node = GzNode()
pub = gz_node.advertise('/world/default/wrench', EntityWrench)
time.sleep(1)

msg = EntityWrench()
msg.entity.name = 'ustrov_0'
msg.entity.type = 2 # model
msg.wrench.force.x = 100.0
# send a huge force just to test
for _ in range(10):
    pub.publish(msg)
    time.sleep(0.1)
print("Sent model wrench.")

msg.entity.name = 'base_link'
msg.entity.type = 1 # link
for _ in range(10):
    pub.publish(msg)
    time.sleep(0.1)
print("Sent link wrench.")
