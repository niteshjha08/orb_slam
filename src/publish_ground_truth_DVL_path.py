import rospy

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np


path = Path()

TIME_INTERVAL = 1./15 # 15: rate of publishing frames
counter=0
file=open("/home/nitesh/catkin_ws/src/orb_slam_2_ros/data/position_values_1_6.txt","r")
lines=file.readlines()

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return qx, qy, qz, qw


def dvl_pose_cb(data):
    global counter
    line=lines[counter]
    counter+=4

    error_transient_x = np.random.normal(0, 0.0001)
    error_random_x = np.random.normal(0, 0.1)

    error_transient_y = np.random.normal(0, 0.0001)
    error_random_y = np.random.normal(0, 0.1)

    error_transient_z = np.random.normal(0, 0.0001)
    error_random_z = np.random.normal(0, 0.1)

    x=float(line.split(',')[0]) + (error_transient_x + error_random_x) * TIME_INTERVAL
    y=float(line.split(',')[1]) + (error_transient_y + error_random_y) * TIME_INTERVAL
    z=float(line.split(',')[2]) + (error_transient_z + error_random_z) * TIME_INTERVAL
    rx=float(line.split(',')[3])
    ry=float(line.split(',')[4])
    rz=float(line.split(',')[5])

    qx,qy,qz,qw=get_quaternion_from_euler(rx,ry,rz)
    
    global path
    path.header = data.header
    path.header.frame_id="final_map_rotated"
    pose = PoseStamped()
    pose.header = data.header
    pose.header.frame_id="final_map_rotated"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    path.poses.append(pose)
    path_pub.publish(path)
    print("GT with DVL Pose:",x,", ",y,", ",z)

rospy.init_node('gt_dvl_path_node')

path_pub = rospy.Publisher('/gt_dvl_path', Path, queue_size=10)

if __name__ == '__main__':
    x, y, z = 0, 0, 0
    pose_sub=rospy.Subscriber('/orb_slam2_rgbd/pose',PoseStamped,dvl_pose_cb)
    rospy.spin()
    