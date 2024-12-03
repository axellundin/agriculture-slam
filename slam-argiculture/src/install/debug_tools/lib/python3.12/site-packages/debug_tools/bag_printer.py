import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the message type you need

class BagPrinter(Node):

    def __init__(self):
        super().__init__('bag_printer')

        # Create a subscription to the topic you want to monitor
        # Replace '/topic_name' and String with your actual topic and message type
        self.subscription = self.create_subscription(
            String,
            '/odometry/odometry',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(f"Received message on topic {self.subscription.topic_name}")
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    print("in main of bag printer")
    rclpy.init(args=args)

    bag_printer = BagPrinter()

    rclpy.spin(bag_printer)

    # Destroy the node explicitly
    bag_printer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()