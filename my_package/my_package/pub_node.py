import rclpy
from std_msgs.msg import String

rclpy.init()
topic_name="my_topic"

def timer_callback():
    msg=String()
    msg.data="Hello ROS2!"
    pub_node.get_logger().info("Talking: '%s'"%msg.data)
    publisher.publish(msg)

pub_node=rclpy.create_node("Publisher_node")
publisher=pub_node.create_publisher(String,topic_name,10)
timer_=pub_node.create_timer(0.5,timer_callback)
    
def main(): 
    rclpy.spin(pub_node)
    