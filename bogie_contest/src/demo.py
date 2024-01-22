#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image
from pyzbar import pyzbar
from std_msgs.msg import String
import tf
import geometry_msgs
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time

global qr_data
qr_data=None

def get_qr_date(msg):
    
    global qr_data
    qr_data=msg.data
    #rospy.loginfo(qr_data) 

def moving(wp):
    global client

    rospy.loginfo("x=%.2f, y=%.2f, yaw=%.2f", wp[0].target_pose.pose.position.x, wp[0].target_pose.pose.position.y,wp[1][2])
    client.send_goal(wp[0])
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        if client.get_result():
            rospy.loginfo("Goal execution done!")
def send_waypoints():
    global qr_data,client

    # Initialize the ROS node

    # Set the header information
    header = rospy.Header(stamp=rospy.Time.now(), frame_id='map')

    # Retrieve the waypoints parameter from the launch file
    waypoints_param = rospy.get_param('~waypoints', '[[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]')
    loop = rospy.get_param('~loop', "false")

    rospy.loginfo("loop %s" , loop) 

    waypoints = eval(waypoints_param)  # Convert string representation to list
    # rospy.loginfo(waypoints) 
    
    # Define the waypoints

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    
    pose_stamped_waypoints = []
    orint=[]
    for waypoint in waypoints:
        quat=[]
        x, y, yaw = waypoint

        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        
        pose_stamped = MoveBaseGoal()
        pose_stamped.target_pose.header=header
        pose_stamped.target_pose.pose.position.x=x
        pose_stamped.target_pose.pose.position.y=y
        pose_stamped.target_pose.pose.position.z=0
        pose_stamped.target_pose.pose.orientation.x= quaternion[0]
        pose_stamped.target_pose.pose.orientation.y= quaternion[1]
        pose_stamped.target_pose.pose.orientation.z= quaternion[2]
        pose_stamped.target_pose.pose.orientation.w= quaternion[3]
        pose_stamped_waypoints.append(pose_stamped)
        quat = [
        pose_stamped.target_pose.pose.orientation.x,
        pose_stamped.target_pose.pose.orientation.y,
        pose_stamped.target_pose.pose.orientation.z,
        pose_stamped.target_pose.pose.orientation.w
        ]

        orint.append(tf.transformations.euler_from_quaternion(quat))

    while True:
        # quaternion = [
        #         [pose_stamped_waypoints[0].target_pose.pose.orientation.x,
        #         pose_stamped_waypoints[0].target_pose.pose.orientation.y,
        #         pose_stamped_waypoints[0].target_pose.pose.orientation.z,
        #         pose_stamped_waypoints[0].target_pose.pose.orientation.w],

        #         [pose_stamped_waypoints[1].target_pose.pose.orientation.x,
        #         pose_stamped_waypoints[1].target_pose.pose.orientation.y,
        #         pose_stamped_waypoints[1].target_pose.pose.orientation.z,
        #         pose_stamped_waypoints[1].target_pose.pose.orientation.w],

        #         [pose_stamped_waypoints[2].target_pose.pose.orientation.x,
        #         pose_stamped_waypoints[2].target_pose.pose.orientation.y,
        #         pose_stamped_waypoints[2].target_pose.pose.orientation.z,
        #         pose_stamped_waypoints[2].target_pose.pose.orientation.w]
        #         ]
        
        # orint = [tf.transformations.euler_from_quaternion(quaternion[0]),
        #         tf.transformations.euler_from_quaternion(quaternion[1]),
        #         tf.transformations.euler_from_quaternion(quaternion[2])]


        start_point=[pose_stamped_waypoints[0],orint[0]]
        table_point=[pose_stamped_waypoints[1],orint[1]]
        prep_point=[pose_stamped_waypoints[2],orint[2]]
        
        # Go to start point
        rospy.loginfo("Start")
        time.sleep(1)
        moving(start_point)
        
        rospy.loginfo("Move to table")
        time.sleep(1)
        moving(table_point)

        # Go to food 1
        rospy.loginfo("Get order")
        rospy.loginfo("Wait for qr")
        while qr_data is None:
            continue
        rospy.loginfo(qr_data)
        qr_data=None

        rospy.loginfo("Get Food")
        time.sleep(1)
        moving(prep_point)
        time.sleep(5)

        rospy.loginfo("Serve")
        time.sleep(1)
        moving(table_point)
        time.sleep(3)

        # Go to food 2
        rospy.loginfo("Get order")
        rospy.loginfo("Wait for qr")
        while qr_data is None:
            continue
        rospy.loginfo(qr_data)
        qr_data=None
        

        rospy.loginfo("Get Food")
        time.sleep(1)
        moving(prep_point)
        time.sleep(5)

        rospy.loginfo("Serve")
        time.sleep(1)
        moving(table_point)
        time.sleep(3)
        
        # Finished
        rospy.loginfo("Get order")
        rospy.loginfo("Wait for qr")
        while qr_data is None:
            continue
        rospy.loginfo(qr_data)
        qr_data=None

        rospy.loginfo("End")
        time.sleep(1)
        moving(start_point)

        if loop is not True:
            rospy.loginfo("Finished job")
            break
        
def image_callback(msg):
    global qr_data
    bridge = CvBridge()
    qr_data=None
    try:
        # Convert the ROS image message to OpenCV image
        publisher_qr = rospy.Publisher('/usb_cam/qr', Image, queue_size=10)
        publisher_qr_data = rospy.Publisher('/qr_data', String, queue_size=10)


        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert the OpenCV image to a ROS image message
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect QR codes in the image
        qr_codes = pyzbar.decode(gray)


        # Process each detected QR code
        for qr_code in qr_codes:
            # Extract the QR code's data
            qr_data = qr_code.data.decode("utf-8")
            

            points = qr_code.polygon
            if len(points) > 4 :
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else :
                hull = points
            #print(hull[0][0])
            # Number of points in the convex hull
            n = len(hull)
        
            # Draw the convext hull
            for j in range(0,n):
                cv2.line(cv_image, hull[j], hull[ (j+1) % n], (255,0,0), 3)
            
            cv2.putText(cv_image,'qr_data',(hull[0][0],hull[0][1]),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255), 2,cv2.LINE_AA, False)

            
            #rospy.loginfo("QR code detected: %s", qr_data)
        
        
        # Publish the ROS image message to a topic if needed
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        publisher_qr.publish(ros_image)
        publisher_qr_data.publish(qr_data)
    except CvBridgeError as e:
        print(e)
        return
    

def image_listener():
    rospy.init_node('contest')
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    # rospy.Subscriber('/qr_data', String, get_qr_date)


    send_waypoints  ()
    rospy.spin()

if __name__ == '__main__':
    try:
        image_listener()
    except rospy.ROSInterruptException:
        pass
