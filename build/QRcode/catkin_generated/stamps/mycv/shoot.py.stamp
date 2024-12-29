import cv2

import rospy
from sensor_msgs.msg import Image  # ROS 中定义的图像消息类型，用于在 ROS 中发布图像。
from cv_bridge import CvBridge  # 用于将 OpenCV 图像（即 numpy.ndarray 格式的图像）转换为 ROS 的图像消息



def capture_image():
    # 打开默认摄像头
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        return None

    # 捕获一帧图像
    ret, frame = cap.read()

    if not ret:
        cap.release()
        return None

    # 释放摄像头资源
    cap.release()

    # 返回捕获的图像
    return frame

def main():
    # 初始化 ROS 节点
    rospy.init_node('shoot', anonymous=True)

    # 订阅图像话题
    pub = rospy.Publisher('processed_image', Image, queue_size=10)

    bridge = CvBridge()

    # 设置日志级别为 WARNING，减少不必要的日志输出
    rospy.logwarn("Node started: Image publishing will begin.")  # 只在程序开始时打印一次信息

    while not rospy.is_shutdown():
        # 捕获图像
        captured_image = capture_image()

        # 如果捕获到图像则进入判断
        if captured_image is not None:
            try:
                # 将 OpenCV 图像转换为 ROS 图像消息
                ros_image = bridge.cv2_to_imgmsg(captured_image, "bgr8")

                # 发布图像到 'processed_image' 话题
                pub.publish(ros_image)

                # 控制打印频率，这里每发布5次图像打印一次
                if rospy.get_time() % 5 < 1:
                    rospy.loginfo("Publishing image")  # 限制日志输出频率

                # 延迟一会儿，确保消息被处理
                rospy.sleep(1)
            except Exception as e:
                rospy.logerr(f"Failed to convert and publish image: {str(e)}")
                rospy.sleep(1)

if __name__ == '__main__':
    main()
