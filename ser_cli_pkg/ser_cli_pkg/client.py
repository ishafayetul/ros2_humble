from msg_srv_pkg.srv import AddTwoInt

import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    
    cli_node=rclpy.create_node("Client_node")
    client=cli_node.create_client(AddTwoInt,"TestSrv")
    
    #waiting for the service to become available
    while not client.wait_for_service(timeout_sec=1.0):
        cli_node.get_logger().warn("Waiting for service...")
    
    #create a request
    request=AddTwoInt.Request()
    request.a=3
    request.b=7
    future=client.call_async(request)
    rclpy.spin_once(cli_node)
    
    response=future.result()
    cli_node.get_logger().info(f'Result is {response.sum}')