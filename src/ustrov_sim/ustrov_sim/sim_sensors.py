import rclpy
from rclpy.node import Node
import math
import random

from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

class SimSensorsNode(Node):
    def __init__(self):
        super().__init__('sim_sensors_node')
        
        self.pub_imu = self.create_publisher(Imu, '/sensor/imu', 10)
        self.pub_dvl = self.create_publisher(TwistWithCovarianceStamped, '/sensor/dvl', 10)
        self.pub_depth = self.create_publisher(PoseWithCovarianceStamped, '/sensor/depth', 10)
        
        self.get_logger().info("启动 Gazebo 完美状态监听并注入噪声...")
        
        self._gz_node = GzNode()
        self._gz_node.subscribe(Pose_V, '/world/default/dynamic_pose/info', self._pose_cb)
        
        self.last_time_sec = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_z = 0.0
        
        self.dvl_bias_vx = random.uniform(0.01, 0.03)  
        self.dvl_bias_vy = random.uniform(-0.03, -0.01)

    def euler_from_quaternion(self, w, x, y, z):
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    def _pose_cb(self, msg: Pose_V):
        if not msg.pose:
            return
            
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        
        dt = current_time_sec - self.last_time_sec
        if dt < 0.05:  # 限制到 20Hz
            return
            
        for pose in msg.pose:
            if pose.name != 'ustrov_0':
                continue
                
            x_ned = pose.position.x
            y_ned = -pose.position.y
            z_ned = -pose.position.z
            
            w, x, y, z = pose.orientation.w, pose.orientation.x, -pose.orientation.y, -pose.orientation.z
            roll, pitch, yaw = self.euler_from_quaternion(w, x, y, z)
            
            if self.last_time_sec > 0.0:
                vx_world = (x_ned - self.last_x) / dt
                vy_world = (y_ned - self.last_y) / dt
                vz_world = (z_ned - self.last_z) / dt
            else:
                vx_world, vy_world, vz_world = 0.0, 0.0, 0.0
                
            self.last_time_sec = current_time_sec
            self.last_x, self.last_y, self.last_z = x_ned, y_ned, z_ned
            
            ros_time_msg = self.get_clock().now().to_msg()
            
            noisy_depth = z_ned + random.gauss(0, 0.1)
            depth_msg = PoseWithCovarianceStamped()
            depth_msg.header.stamp = ros_time_msg
            depth_msg.header.frame_id = 'odom'
            depth_msg.pose.pose.position.z = noisy_depth
            depth_msg.pose.covariance[14] = 0.01  
            self.pub_depth.publish(depth_msg)
            
            vx_body = math.cos(-yaw) * vx_world - math.sin(-yaw) * vy_world
            vy_body = math.sin(-yaw) * vx_world + math.cos(-yaw) * vy_world
            vz_body = vz_world
            
            noisy_vx = vx_body + random.gauss(0, 0.05) + self.dvl_bias_vx
            noisy_vy = vy_body + random.gauss(0, 0.05) + self.dvl_bias_vy
            noisy_vz = vz_body + random.gauss(0, 0.05) 
            
            dvl_msg = TwistWithCovarianceStamped()
            dvl_msg.header.stamp = ros_time_msg
            dvl_msg.header.frame_id = 'base_link'
            dvl_msg.twist.twist.linear.x = noisy_vx
            dvl_msg.twist.twist.linear.y = noisy_vy
            dvl_msg.twist.twist.linear.z = noisy_vz
            dvl_msg.twist.covariance[0] = 0.0025  
            dvl_msg.twist.covariance[7] = 0.0025  
            dvl_msg.twist.covariance[14] = 0.0025 
            self.pub_dvl.publish(dvl_msg)
            
            nx = x + random.gauss(0, 0.01)
            ny = y + random.gauss(0, 0.01)
            nz = z + random.gauss(0, 0.01)
            nw = w
            norm = math.sqrt(nx*nx + ny*ny + nz*nz + nw*nw)
            
            imu_msg = Imu()
            imu_msg.header.stamp = ros_time_msg
            imu_msg.header.frame_id = 'base_link'
            imu_msg.orientation.w = nw / norm
            imu_msg.orientation.x = nx / norm
            imu_msg.orientation.y = ny / norm
            imu_msg.orientation.z = nz / norm
            
            imu_msg.orientation_covariance[0] = 0.001
            imu_msg.orientation_covariance[4] = 0.001
            imu_msg.orientation_covariance[8] = 0.001
            self.pub_imu.publish(imu_msg)
            
            break

def main(args=None):
    rclpy.init(args=args)
    node = SimSensorsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
