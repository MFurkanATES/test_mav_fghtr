import cv2
import math
import numpy as np
import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped,Vector3,TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mavros_msgs.msg import PositionTarget



#--vision-----
window_name2 = "Hue histogram back projection" 

old_sec = 0
new_sec = 0

pixel_yatay = 640
pixel_dikey = 480


#---orientation-----
X = 0.0
Y = 0.0
Z = 0.0
roll_degree = 0.0
pitch_degree = 0.0
yaw_degree = 0.0
quaternion_X = 0.0
quaternion_Y = 0.0
quaternion_Z = 0.0
quaternion_W = 0.0


#---yaw_pid ------
pre_hata_yaw_pix = 0
hata_yaw_pix = 0

"""if(new_sec != 0):

    # create window by name (note flags for resizable or not)

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.namedWindow(window_name2, cv2.WINDOW_NORMAL)
    cv2.namedWindow(window_nameSelection, cv2.WINDOW_NORMAL)

    # set sliders for HSV selection thresholds

    s_lower = 60
    cv2.createTrackbar("s lower", window_name2, s_lower, 255, nothing)
    s_upper = 255
    cv2.createTrackbar("s upper", window_name2, s_upper, 255, nothing)
    v_lower = 32
    cv2.createTrackbar("v lower", window_name2, v_lower, 255, nothing)
    v_upper = 255
    cv2.createTrackbar("v upper", window_name2, v_upper, 255, nothing)


    # Setup the termination criteria for search, either 10 iteration or
    # move by at least 1 pixel pos. difference
    term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)

def track_object(image,x,y,area): 

        start_t = cv2.getTickCount()
        s_lower = cv2.getTrackbarPos("s lower", window_name2)
        s_upper = cv2.getTrackbarPos("s upper", window_name2)
        v_lower = cv2.getTrackbarPos("v lower", window_name2)
        v_upper = cv2.getTrackbarPos("v upper", window_name2)"""




def track_yaw(curr_yaw,target_yaw_pix):
        global pre_hata_yaw_pix 
        global hata_yaw_pix
        p_yaw = 0.2
        d_yaw = 0.01
        hata_yaw_pix = 320 - target_yaw_pix
        print(hata_yaw_pix)
        if (hata_yaw_pix <= 0):#saat yonu
        #yaw hareketi saat yonunde olacaktir        
                calc_yaw =  ((p_yaw * hata_yaw_pix) + (d_yaw *(hata_yaw_pix - pre_hata_yaw_pix)))
                publish_yaw(calc_yaw)
                pre_hata_yaw_pix = hata_yaw_pix
                print("calc_yaw ",calc_yaw)
                
               
        if (hata_yaw_pix > 0): #saat yonu tersi
                calc_yaw =((p_yaw * hata_yaw_pix) + (d_yaw *(hata_yaw_pix - pre_hata_yaw_pix)))
                publish_yaw(calc_yaw)
                pre_hata_yaw_pix = hata_yaw_pix
                print("calc_yaw",calc_yaw)




def publish_yaw(new_yaw):
    pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    data = TwistStamped()
    #print(data)
    
    data.twist.angular.x = 0
    data.twist.angular.y = 0
    data.twist.angular.z = math.radians(new_yaw)
    pub.publish(data)    
    #print(data)
    #rospy.loginfo(data)

        

kalman = cv2.KalmanFilter(4, 2)
kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 0]], np.float32)

kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                    [0, 1, 0, 1],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]], np.float32)

kalman.processNoiseCov = np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]], np.float32) * 0.03

measurement = np.array((2, 1), np.float32)
prediction = np.zeros((2, 1), np.float32)

def center(points):
    x = np.float32(
        (points[0][0] +
         points[1][0] +
         points[2][0] +
         points[3][0]) /
        4.0)
    y = np.float32(
        (points[0][1] +
         points[1][1] +
         points[2][1] +
         points[3][1]) /
        4.0)
    return np.array([np.float32(x), np.float32(y)], np.float32)

def target_line_on_screen(image):

        #cv2.line(image,((pixel_yatay/2)-10,pixel_dikey/2),((pixel_yatay/2)-30,pixel_dikey/2),(150,255,50),2)
        #cv2.line(image,((pixel_yatay/2)+10,pixel_dikey/2),((pixel_yatay/2)+30,pixel_dikey/2),(150,255,50),2)
        cv2.line(image,(pixel_yatay/2 + 10,(pixel_dikey/2)-10),(pixel_yatay/2+10,(pixel_dikey/2)-30),(150,255,50),2)
        cv2.line(image,(pixel_yatay/2 + 10,(pixel_dikey/2)+10),(pixel_yatay/2+10,(pixel_dikey/2)+30),(150,255,50),2)
        cv2.line(image,(pixel_yatay/2 - 10,(pixel_dikey/2)-10),(pixel_yatay/2-10,(pixel_dikey/2)-30),(150,255,50),2)
        cv2.line(image,(pixel_yatay/2 - 10,(pixel_dikey/2)+10),(pixel_yatay/2-10,(pixel_dikey/2)+30),(150,255,50),2)

        cv2.line(image,(0,240),(640,240),(150,255,50),2)

def callback(msg):
        global new_sec
        global old_sec
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        new_sec = msg.header.seq
        target_line_on_screen(cv_image)
        if (new_sec > old_sec):
                old_sec = new_sec
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                ret,thresh = cv2.threshold(gray_image,235,255,cv2.THRESH_BINARY)

                cnts=cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
                center=None
	        if len(cnts)>0:
                        c=max(cnts,key=cv2.contourArea)
                        ((x,y),radius)=cv2.minEnclosingCircle(c)
                                              
                        #cv2.circle(cv_image,(int(x),int(y)),int(radius),(0,255,0))
                        track_yaw(yaw_degree,x)

                cv2.imshow("frame", cv_image)
        

       

       
        if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.loginfo("finished.")




def autopilot_imu_orientation(imu_orientation):  
        global X
        global Y
        global Z
        global roll_degree
        global pitch_degree
        global yaw_degree
        global quaternion_X
        global quaternion_Y
        global quaternion_Z
        global quaternion_W

        quaternion_X = imu_orientation.orientation.x
        quaternion_Y = imu_orientation.orientation.y
        quaternion_Z = imu_orientation.orientation.z
        quaternion_W = imu_orientation.orientation.w

        quaternion =(imu_orientation.orientation.x,imu_orientation.orientation.y,imu_orientation.orientation.z,imu_orientation.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        #radian
        X = euler[0]
        Y = euler[1]
        Z = euler[2]
        #degree
        roll_degree = int(abs(math.degrees(X)))
        pitch_degree = int(abs(math.degrees(Y)))
        yaw_degree = int(abs(math.degrees(Z)))




def image_sub():
        rospy.init_node('cv_stream', anonymous=False)
        sub = rospy.Subscriber('line', Image, callback)
        #rospy.Subscriber('mavros/imu/data',Imu,autopilot_imu_orientation)
        
        rospy.spin()
	
        cv2.destoryAllWindows()

if __name__ == '__main__':
        image_sub()
       