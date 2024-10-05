import rclpy
import rclpy.node
from std_msgs.msg import String

rclpy.init()
topic_name="my_topic"

sub_node=rclpy.create_node("Sub_node")

def callback(msg):
    sub_node.get_logger().info("I heard Info: '%s'" %msg.data)
    sub_node.get_logger().warn("I heard Warn: '%s'" %msg.data)
    sub_node.get_logger().error("I heard Error: '%s'" %msg.data)

def main():
    subscriber=sub_node.create_subscription(String,topic_name,callback,10)
    rclpy.spin(sub_node)