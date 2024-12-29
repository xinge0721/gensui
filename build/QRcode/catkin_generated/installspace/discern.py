import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from std_msgs.msg import String

def recognize_qrcode_from_image(frame):
    """
    识别图像中的二维码并打印信息。
    :param frame: 输入的 OpenCV 图像(numpy.ndarray 格式）
    :return: None
    """
    # 使用 pyzbar 解码二维码
    decoded_objects = decode(frame)

    if decoded_objects:
        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8').replace("\r", "").replace("\n", "")
            pub_qrcode_data.publish(qr_data)
            print(f"QR Code detected: {qr_data}")
    else:
        print("No QR Code detected.")

def image_callback(msg):
    """
    ROS 回调函数，处理接收到的图像消息并进行二维码识别。
    :param msg: ROS 图像消息
    :return: None
    """
    try:
        # 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 图像格式
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")

        # 调用二维码识别函数
        recognize_qrcode_from_image(frame)
    except CvBridgeError as e:
        rospy.logerr(f"cv_bridge 错误: {e}")

def main():
    # 初始化 ROS 节点
    rospy.init_node('discern', anonymous=True)
    global pub_qrcode_data

    pub_qrcode_data = rospy.Publisher('qrcode_data', String, queue_size=10)

    # 订阅图像话题
    rospy.Subscriber("processed_image", Image, image_callback)  # 假设图像话题为 "/processed_image"

    # 保持节点运行，等待回调
    rospy.spin()

if __name__ == '__main__':
    main()
