import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from hello_interfaces.msg import MyString  # カスタムメッセージタイプ
from ultralytics import YOLO
import cv2
import torch
import threading
import time

# ROS_DOMAIN_ID を設定
os.environ['ROS_DOMAIN_ID'] = '1'

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        qos_profile = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher_ = self.create_publisher(
            MyString, 'chatter', qos_profile)

        # YOLO モデルの読み込み
        self.model = YOLO('/home/hisa/強化学習/best.pt')
        self.conf_threshold = 0.6  # 信頼度のしきい値

        # GPU を使用するかどうか確認
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info('CUDAが使用可能です。GPUを使用します。')
        else:
            self.get_logger().info('CUDAが使用不可です。CPUを使用します。')

        # カメラのオープン
        self.cap = cv2.VideoCapture(0)  # カメラのオープン
        if not self.cap.isOpened():
            self.get_logger().error('カメラのオープンに失敗しました')
            raise RuntimeError('カメラにアクセスできません')

        # フレームレートと解像度の設定
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        # スレッドで画像処理ループを開始
        self.processing = True
        self.image_processing_thread = threading.Thread(
            target=self.image_processing_loop)
        self.image_processing_thread.daemon = True
        self.image_processing_thread.start()

        # ボール認識状態の初期化
        self.last_red_detection_time = time.time()
        self.last_blue_detection_time = time.time()

    def send_message(self, message):
        msg = MyString()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

    def image_processing_loop(self):
        while rclpy.ok():
            if not self.processing:
                continue

            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('画像の取得に失敗しました')
                rclpy.shutdown()
                break

            # YOLOによる推論
            results = self.model(frame)

            # 最大サイズの赤と青のオブジェクトを見つける
            largest_red_object = self.find_largest_object(results, 'red')
            largest_blue_object = self.find_largest_object(results, 'blue')

            current_time = time.time()

            # 赤オブジェクトの結果を送信
            if largest_red_object:
                x_center = (largest_red_object[1] + largest_red_object[3]) / 2.0
                self.send_message(f'red: {x_center:.2f}')
                self.draw_bounding_box(
                    frame, largest_red_object, (0, 0, 255), 'red')
                self.last_red_detection_time = current_time
            elif current_time - self.last_red_detection_time > 0.4:
                self.send_message('red: nothing')

            # 青オブジェクトの結果を送信（赤のメッセージ送信後に実行）
            if largest_blue_object:
                x_center = (
                    largest_blue_object[1] + largest_blue_object[3]) / 2.0
                self.send_message(f'blue: {x_center:.2f}')
                self.draw_bounding_box(
                    frame, largest_blue_object, (255, 0, 0), 'blue')
                self.last_blue_detection_time = current_time
            elif current_time - self.last_blue_detection_time > 0.4:
                self.send_message('blue: nothing')

            # 画像表示
            cv2.imshow('Object Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
                break

    def find_largest_object(self, results, target_class):
        max_avg_size = 0
        largest_object = None
        for result in results:
            for box in result.boxes:
                if box.conf >= self.conf_threshold:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    width = x2 - x1
                    height = y2 - y1
                    avg_size = (width + height) / 2.0
                    class_id = int(box.cls)
                    class_name = self.model.names[class_id]

                    if class_name == target_class and avg_size > max_avg_size:
                        max_avg_size = avg_size
                        largest_object = (box, x1, y1, x2, y2)
        return largest_object

    def draw_bounding_box(self, frame, object_info, color, label):
        box, x1, y1, x2, y2 = object_info
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        x_center = (x1 + x2) / 2.0
        cv2.putText(frame, f'{label} {x_center:.2f}', (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)


def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetectionNode()

    try:
        rclpy.spin(object_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        object_detection_node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
