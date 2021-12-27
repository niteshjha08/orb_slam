import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

rospy.init_node("publish_images", anonymous=True)
pub=rospy.Publisher('/camera/image_raw',Image,queue_size=1)

path="/home/nitesh/catkin_ws/src/orb_slam_2_ros/videos/front_cam.avi"
# path=0
cap = cv2.VideoCapture(path)
rate=rospy.Rate(1)
bridge=CvBridge()
while True:

    ret, frame = cap.read()
    img_message=bridge.cv2_to_imgmsg(frame,encoding="passthrough")
    pub.publish(img_message)
    rate.sleep()
    
