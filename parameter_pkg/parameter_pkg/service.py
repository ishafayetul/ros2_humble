import rclpy
from rclpy.node import Node
from msg_srv_pkg.srv import AddTwoInt

def handle_request(req,resp):
    print(req)
    print(resp)
    resp.sum=req.a+req.b
    print(f'Request: {req.a} + {req.b} = {resp.sum}')
    return resp

def main():
    rclpy.init()
    print()
    srv_node=rclpy.create_node("Service_node")
    service=srv_node.create_service(AddTwoInt,"TestSrv",handle_request)
    
    rclpy.spin(srv_node)