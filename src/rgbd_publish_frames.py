import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node("publish_images", anonymous=True)
rgb_pub=rospy.Publisher('/camera/rgb/image_raw',Image,queue_size=1)
depth_pub=rospy.Publisher('/camera/depth_registered/image_raw',Image,queue_size=1)
rgb_path="/home/nitesh/catkin_ws/src/orb_slam_2_ros/videos/orb_slam_1_6/front_cam_rgb.mp4"
depth_path="/home/nitesh/catkin_ws/src/orb_slam_2_ros/videos/orb_slam_1_6/front_cam_depth.mp4"
# path=0
cap_rgb = cv2.VideoCapture(rgb_path)
cap_depth = cv2.VideoCapture(depth_path)
rate=rospy.Rate(15)
bridge=CvBridge()
while True or not rospy.is_shutdown():

    ret, frame_rgb = cap_rgb.read()
    ret, frame_depth = cap_depth.read()

    img_message_rgb=bridge.cv2_to_imgmsg(frame_rgb,encoding="passthrough")
    img_message_depth=bridge.cv2_to_imgmsg(frame_depth,encoding="passthrough")
    rgb_pub.publish(img_message_rgb)
    depth_pub.publish(img_message_depth)

    rate.sleep()
    
