import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("Simple_Subscriber")
        self.sub= self.create_subscription(String,"Chatter",self.msgCallback,10)

    def msgCallback(self,msg):
        self.get_logger().info("I heard is %s" %msg.data)




def main():
    rclpy.init()
    simplesub=SimpleSubscriber()
    rclpy.spin(simplesub)
    simplesub.destroy_node()
    rclpy.shutdown()


if __name__ =='__main__':
    main()
