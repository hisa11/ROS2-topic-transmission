import os
import rclpy
from hello_interfaces.msg import MyString
from rclpy.node import Node
import serial
import time

# ROS_DOMAIN_IDを設定
os.environ['ROS_DOMAIN_ID'] = '1'

class MySubscriber(Node):
    def __init__(self):
        super().__init__('serial_node')
        
        # シリアルポートを自動的に選択
        self.serial_port = self.detect_serial_port()

        if not self.serial_port or not self.serial_port.isOpen():
            self.get_logger().error('Failed to open the serial port')
            exit(1)

        self.subscription = self.create_subscription(
            MyString, "chatter", self.listener_callback, 10
        )
        self.msg = None
        self.last_send_time = time.time()

    def detect_serial_port(self):
        # 使用可能なシリアルポートを定義
        possible_ports = ['/dev/ttyACM0', '/dev/ttyACM1']
        serial_port = None

        # ポートを1つずつ試して、成功したポートを使う
        for port in possible_ports:
            try:
                serial_port = serial.Serial(port, 250000, timeout=70)
                self.get_logger().info(f"Using serial port: {port}")
                return serial_port
            except serial.SerialException:
                self.get_logger().warning(f"Failed to open {port}")

        return None

    def listener_callback(self, msg):
        self.get_logger().info(f"Subscribe {msg.data}")
        self.msg = msg
        self.send_serial_data()  # メッセージを受け取った直後にデータを送信
        self.last_send_time = time.time()

    def send_serial_data(self):
        if self.msg is not None:
            data_to_send = self.msg.data.replace('\n', '').replace('\r', '') + '|'
            self.serial_port.write(data_to_send.encode())

def main(args=None):
    rclpy.init(args=args)
    my_subscriber = MySubscriber()

    try:
        rclpy.spin(my_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        if my_subscriber.serial_port:
            my_subscriber.serial_port.close()
        my_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
