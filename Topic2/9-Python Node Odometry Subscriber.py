import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odometry_listener')
        self.subscription = self.create_subscription(Odometry,'/model/vehicle_blue/odometry',
            self.listener_callback,10) #Message/Topic Name/CallBack Function/Buffer

    #Each Time A message is received the callback function is triggered
    def listener_callback(self, msg):
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        ## Extract quaternion
        # q = msg.pose.pose.orientation
        # yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))

        # Velocities
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        print(f'Pose: x={x:.2f}, y={y:.2f}, yaw={math.degrees(yaw):.1f}° | v={v:.2f}, w={w:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
