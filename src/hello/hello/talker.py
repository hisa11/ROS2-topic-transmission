import rclpy
from hello_interfaces.msg import MyString
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from pyPS4Controller.controller import Controller
import os
import subprocess

class MyController(Controller, Node):
  def __init__(self, **kwargs):
    Controller.__init__(self, **kwargs)
    Node.__init__(self, 'ps4_controller_node')

    qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    self.publisher_ = self.create_publisher(MyString, 'chatter', qos_profile)
    self.timer = self.create_timer(0.02, self.timer_callback)  # 20msごとにコールバック

  def timer_callback(self):
    self.listen(timeout=5)

  def on_playstation_button_press(self):
    msg = MyString()
    msg.data = "psON"
    self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_playstation_button_release(self):
    msg = MyString()
    msg.data = "psOFF"
    self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)
    

  def on_right_arrow_press(self):
    msg = MyString()
    msg.data = "finish"
    self.publisher_.publish(msg)
    msg = MyString()
    msg.data = "nari"
    self.publisher_.publish(msg)
    subprocess.run(
        "export ROS_DOMAIN_ID=1 && ros2 run hello talker", shell=True)

  def on_up_arrow_press(self):
    msg = MyString()
    msg.data = "finish"
    self.publisher_.publish(msg)
    msg = MyString()
    msg.data = "nari"
    self.publisher_.publish(msg)
    subprocess.run(
        "export ROS_DOMAIN_ID=2 && ros2 run hello talker", shell=True)

  def on_left_arrow_press(self):
    msg = MyString()
    msg.data = "finish"
    self.publisher_.publish(msg)
    msg = MyString()
    msg.data = "nari"
    self.publisher_.publish(msg)
    subprocess.run(
        "export ROS_DOMAIN_ID=3 && ros2 run hello talker", shell=True)

  def on_circle_press(self):
    msg = MyString()
    msg.data = "circle"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_x_release(self):
    msg = MyString()
    msg.data = "croOFF"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_square_press(self):
    msg = MyString()
    msg.data = "square"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_circle_release(self):
    msg = MyString()
    msg.data = "circOFF"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_x_press(self):
    msg = MyString()
    msg.data = "cross"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_triangle_press(self):
    msg = MyString()
    msg.data = "triangle"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  # ジャンプ機構
  def on_L1_press(self):
    msg = MyString()
    msg.data = "L1ON"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_L1_release(self):
    msg = MyString()
    msg.data = "L1OFF"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_R1_press(self):
    msg = MyString()
    msg.data = "R1ON"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_R1_release(self):
    msg = MyString()
    msg.data = "R1OFF"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_L3_press(self):
    msg = MyString()
    msg.data = "L3ON"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  def on_L3_release(self):
    msg = MyString()
    msg.data = "L3OFF"
    for _ in range(3):
      self.publisher_.publish(msg)
    self.get_logger().info("Published: " + msg.data)

  # 三輪オムニの制御
  def on_R3_left(self, value):
    if -3000 < value < 3000:
      value = 0
    msg = MyString()
    msg.data = f"R3_x: {value}"
    self.publisher_.publish(msg)
    self.get_logger().info("Published: R3_x: " + str(value))

  def on_R3_right(self, value):
    if -3000 < value < 3000:
      value = 0
    msg = MyString()
    msg.data = f"R3_x: {value}"
    self.publisher_.publish(msg)
    self.get_logger().info("Published: R3_x: " + str(value))

  def on_R3_up(self, value):
    if -3000 < value < 3000:
      value = 0
    msg = MyString()
    msg.data = f"R3_y: {value}"
    self.publisher_.publish(msg)
    self.get_logger().info("Published: R3_y: " + str(value))

  def on_R3_down(self, value):
    if -3000 < value < 3000:
      value = 0
    msg = MyString()
    msg.data = f"R3_y: {value}"
    self.publisher_.publish(msg)
    self.get_logger().info("Published: R3_y: " + str(value))

  def on_L3_left(self, value):
    if -3000 < value < 3000:
      value = 0
    msg = MyString()
    msg.data = f"L3_x: {value}"
    self.publisher_.publish(msg)
    self.get_logger().info("Published: L3_x: " + str(value))

  def on_L3_right(self, value):
    if -3000 < value < 3000:
      value = 0
    msg = MyString()
    msg.data = f"L3_x: {value}"
    self.publisher_.publish(msg)
    self.get_logger().info("Published: L3_x: " + str(value))

  def on_L3_up(self, value):
    if -3000 < value < 3000:
      value = 0
    msg = MyString()
    msg.data = f"L3_y: {-value}"
    self.publisher_.publish(msg)
    self.get_logger().info("Published: L3_y: " + str(-value))

  def on_L3_down(self, value):
    if -3000 < value < 3000:
      value = 0
    msg = MyString()
    msg.data = f"L3_y: {-value}"
    self.publisher_.publish(msg)
    self.get_logger().info("Published: L3_y: " + str(-value))

def main(args=None):
  rclpy.init(args=args)
  controller = MyController(interface="/dev/input/js0",
                            connecting_using_ds4drv=False)
  rclpy.spin(controller)  # コントローラーとROS2ノードを同時に実行

  # コントローラが停止したら、ノードを破棄してROS通信をシャットダウンする
  controller.destroy_node()
  rclpy.shutdown()

if __name__ == "__main__":
  main()
