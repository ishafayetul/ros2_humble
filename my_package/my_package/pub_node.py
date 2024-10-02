import rclpy
from std_msgs.msg import String

rclpy.init()
topic_name="my_topic"

pub_node=rclpy.create_node("Publisher_node")
publisher=pub_node.create_publisher(String,topic_name,10)

msg=String()
msg.data="Hello ROS2!"
def main(): 
    while(1):
        pub_node.get_logger().info("Talking: '%s'"%msg.data)
        publisher.publish(msg) 
        #rclpy.spin(pub_node)