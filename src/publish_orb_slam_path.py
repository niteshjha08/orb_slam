import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

path = Path()


def pose_cb(data):
    global path
    path.header = data.header
    pose = PoseStamped()
    pose.header = data.header
    pose.pose = data.pose
    path.poses.append(pose)
    path_pub.publish(path)
    print("ORB Pose:",path.poses[-1].pose.position.x,", ",path.poses[-1].pose.position.y,", ",path.poses[-1].pose.position.z)

rospy.init_node('path_node')

path_pub = rospy.Publisher('/path', Path, queue_size=10)

if __name__ == '__main__':
    x, y, z = 0, 0, 0
    pose_sub=rospy.Subscriber('/orb_slam2_rgbd/pose',PoseStamped,pose_cb)
    # while True or not rospy.is_shutdown():
    #     x+=1
    #     y+=1
    #     z+=1
    #     pose = PoseStamped()
    #     pose.header = data.header
    #     pose.pose = data.pose.pose
    #     path.poses.append(pose)
    #     path_pub.publish(path)
    rospy.spin()