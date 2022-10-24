import rospy
from std_msgs.msg import Int64,Int32
from time_manager import TimeHelper
from encoder_manager import Encoder
from motor_pid import MotorPID

timer = TimeHelper()
encoder = Encoder()
pub = rospy.Publisher('controlled_rpm', Int32, queue_size=10)
rospy.init_node('motor_controller', anonymous=True)



def target_rpm_recieved(data):
    global motorPID
    Kp = 5.0;
    Ki = 4.0;
    Kd = 0.1;
    motorPID = MotorPID(data.data,Kp,Ki,Kd,500)

def encoder_count_changed(data):
    global encoder
    global timer
    global motorPID
    
    count = data.data
    dt = timer.get_dt()
    rpm = encoder.count_to_rpm(count,dt)
    if(dt > 0):
        controlled_rpm = motorPID.get_controlled_rpm(rpm,dt)
        pub.publish(controlled_rpm)
        

rospy.Subscriber("target_rpm", Int32, target_rpm_recieved)

rospy.Subscriber("encoder", Int64, encoder_count_changed)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()
