import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import json
from minio import Minio
import pymysql
import numpy as np

# ====================== 病害检测模型配置 ======================
MODEL_PATH = "/home/sunrise/code/road_detection-main/newModel.onnx"
CLASS_NAMES = ["D00", "D10", "D20", "D30"]
CONF_THRESHOLD = 0.25
IOU_THRESHOLD = 0.45
INPUT_SIZE = (320, 320)

# ====================== 小车运动控制配置 ======================
CRUISE_SPEED = 0.2
PARK_SECONDS = 3.0

# ========== Web平台同步配置 ==========
MINIO_ENDPOINT = "192.168.137.45:9000"
MINIO_ACCESS_KEY = "admin"
MINIO_SECRET_KEY = "Lyy430520"
MINIO_BUCKET = "storage"

MYSQL_HOST = "127.0.0.1"
MYSQL_PORT = 3306
MYSQL_USER = "django"
MYSQL_PASSWORD = "road123456"
MYSQL_DB = "road_detection"


def letterbox(img, new_shape=(320, 320), color=(114, 114, 114)):
    shape = img.shape[:2]
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, r, dw, dh


def postprocess(output, orig_h, orig_w, r, dw, dh, conf_thres, iou_thres, num_classes):
    pred = output[0].transpose(1, 0)

    boxes = []
    scores = []
    class_ids = []

    for det in pred:
        cls_scores = det[4:]
        class_id = np.argmax(cls_scores)
        score = cls_scores[class_id]
        if score < conf_thres:
            continue

        cx, cy, w, h = det[:4]
        x1 = (cx - w/2 - dw) / r
        y1 = (cy - h/2 - dh) / r
        x2 = (cx + w/2 - dw) / r
        y2 = (cy + h/2 - dh) / r

        x1 = max(0, min(x1, orig_w))
        y1 = max(0, min(y1, orig_h))
        x2 = max(0, min(x2, orig_w))
        y2 = max(0, min(y2, orig_h))

        if x2 - x1 > 1 and y2 - y1 > 1:
            boxes.append([x1, y1, x2, y2])
            scores.append(float(score))
            class_ids.append(int(class_id))

    if len(boxes) == 0:
        return [], [], []

    indices = cv2.dnn.NMSBoxes(boxes, scores, conf_thres, iou_thres)

    final_boxes = []
    final_scores = []
    final_class_ids = []
    for i in indices:
        final_boxes.append(boxes[i])
        final_scores.append(scores[i])
        final_class_ids.append(class_ids[i])

    return final_boxes, final_scores, final_class_ids


class RoadDefectDetector(Node):
    def __init__(self):
        super().__init__('road_defect_detector')
        self.bridge = CvBridge()
        self.running = True
        self.frame_count = 0

        self.net = cv2.dnn.readNetFromONNX(MODEL_PATH)
        self.get_logger().info("✅ 自定义病害模型加载成功（CPU模式）")

        self.subscription = self.create_subscription(
            Image,
            '/aurora/rgb/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("📷 已订阅相机话题，道路病害巡检系统启动")

        self.result_pub = self.create_publisher(
            Image,
            '/defect_detection/image_result',
            10
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.defect_continuous_count = 0
        self.is_parking = False
        self.parking_start_time = 0.0
        self.parking_duration = PARK_SECONDS
        self.cooldown_time = 3.0
        self.last_parking_end_time = -10.0  # 初始冷却时间设为很久以前，开机直接巡航

        try:
            self.minio_client = Minio(
                MINIO_ENDPOINT,
                access_key=MINIO_ACCESS_KEY,
                secret_key=MINIO_SECRET_KEY,
                secure=False
            )
            self.mysql_conn = pymysql.connect(
                host=MYSQL_HOST,
                port=MYSQL_PORT,
                user=MYSQL_USER,
                password=MYSQL_PASSWORD,
                database=MYSQL_DB,
                charset='utf8mb4'
            )
            self.get_logger().info("✅ 已连接Web平台，检测结果将同步到历史记录")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Web平台连接失败: {e}")
            self.mysql_conn = None

        # 节点启动立刻发送前进速度
        start_twist = Twist()
        start_twist.linear.x = CRUISE_SPEED
        self.cmd_pub.publish(start_twist)
        self.get_logger().info("🚗 小车启动，开始自动巡航巡检")

    def image_callback(self, msg):
        self.frame_count += 1
        log_flag = (self.frame_count % 10 == 0)

        if not self.running:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        orig_h, orig_w = cv_image.shape[:2]

        try:
            img, r, dw, dh = letterbox(cv_image, INPUT_SIZE)
            blob = cv2.dnn.blobFromImage(img, 1/255.0, INPUT_SIZE, swapRB=True, crop=False)
            self.net.setInput(blob)
            outputs = self.net.forward()

            final_boxes, final_scores, final_class_ids = postprocess(
                outputs, orig_h, orig_w, r, dw, dh,
                CONF_THRESHOLD, IOU_THRESHOLD, len(CLASS_NAMES)
            )

            annotated_frame = cv_image.copy()
            for box, score, class_id in zip(final_boxes, final_scores, final_class_ids):
                x1, y1, x2, y2 = map(int, box)
                label = f"{CLASS_NAMES[class_id]} {score:.2f}"
                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated_frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            if log_flag:
                self.get_logger().info(f"正在处理第{self.frame_count}帧图像推理")
        except Exception as e:
            self.get_logger().error(f"推理失败: {e}")
            return

        defect_count = len(final_boxes)
        current_time = self.get_clock().now().nanoseconds / 1e9

        # 停车状态处理
        if self.is_parking:
            if current_time - self.parking_start_time >= self.parking_duration:
                self.is_parking = False
                self.last_parking_end_time = current_time
                move_msg = Twist()
                move_msg.linear.x = CRUISE_SPEED
                self.cmd_pub.publish(move_msg)
                self.get_logger().info("✅ 停车3秒完成，小车恢复巡检")
        else:
            # 冷却判断
            in_cooldown = (current_time - self.last_parking_end_time) < self.cooldown_time
            if in_cooldown:
                self.defect_continuous_count = 0
            else:
                if defect_count > 0:
                    self.defect_continuous_count += 1
                    # 连续3帧病害触发停车
                    if self.defect_continuous_count >= 3:
                        self.is_parking = True
                        self.parking_start_time = current_time
                        stop_msg = Twist()
                        stop_msg.linear.x = 0.0
                        self.cmd_pub.publish(stop_msg)
                        self.get_logger().info("🚨 连续检测到病害，小车自动停车3秒记录")

                        save_name = f"/tmp/defect_{self.get_clock().now().nanoseconds}.jpg"
                        cv2.imwrite(save_name, annotated_frame)

                        if self.mysql_conn:
                            try:
                                object_name = f"ros_defect_{self.get_clock().now().nanoseconds}.jpg"
                                self.minio_client.fput_object(
                                    MINIO_BUCKET, object_name, save_name,
                                    content_type="image/jpeg"
                                )
                                img_url = f"http://{MINIO_ENDPOINT}/{MINIO_BUCKET}/{object_name}"
                                result_list = []
                                for box, score, class_id in zip(final_boxes, final_scores, final_class_ids):
                                    result_list.append({
                                        "class": CLASS_NAMES[class_id],
                                        "confidence": round(float(score), 4),
                                        "bbox": box
                                    })
                                result_json = json.dumps(result_list, ensure_ascii=False)
                                cursor = self.mysql_conn.cursor()
                                sql = "INSERT INTO detection_result (img_url, result) VALUES (%s, %s)"
                                cursor.execute(sql, (img_url, result_json))
                                self.mysql_conn.commit()
                                cursor.close()
                                self.get_logger().info("☁️ 病害图片与检测结果已同步Web历史记录")
                            except Exception as e:
                                self.get_logger().error(f"同步失败: {e}")
                                if self.mysql_conn:
                                    self.mysql_conn.rollback()
                else:
                    # 无病害，重置计数
                    self.defect_continuous_count = 0

        # 发布标注画面
        try:
            result_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            result_msg.header = msg.header
            self.result_pub.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f"结果画面发布失败: {e}")

    def stop(self):
        self.running = False
        try:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            self.cmd_pub.publish(stop_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    detector = RoadDefectDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info("程序收到终止信号，正在退出...")
    except Exception as e:
        print(f"程序异常退出: {e}")
    finally:
        detector.stop()
        detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)


if __name__ == '__main__':
    main()