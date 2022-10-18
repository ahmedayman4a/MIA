import rospy
from std_msgs.msg import Float32

previous_time = 0
def get_dt():
    global previous_time
    # if this is the first reading there is no dt
    if previous_time == 0.:
        previous_time = rospy.get_time()
        return 0
    
    current_time = rospy.get_time()
    dt = (current_time - previous_time); # Calculate delta time
    previous_time = current_time
    return dt


class Kalman :
    def __init__(self):
        # We will set the variables like so, these can also be tuned by the user
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

        self.angle = 0.0; # Reset the angle
        self.bias = 0.0; # Reset bias
        
        # Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
        self.P = ([0.,0.],
                  [0.,0.])

        # The angle should be in degrees and the delta time in seconds
    def getKalmanAngle(self,newAngle,dt):
        # KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
        # See this blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it


        # Update estimation error covariance - Project the error covariance ahead
        # Step 1
        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
        # Calculate Kalman gain - Compute the Kalman gain
        # Step 4
        S = self.P[0][0] + self.R_measure; # Estimate error
        # Step 5
        K =[0,0] # Kalman gain - This is a 2x1 vector
        K[0] = self.P[0][0] / S
        K[1] = self.P[1][0] / S

        # Calculate angle and bias - Update estimate with measurement zk (newAngle)
        # Step 3
        y = newAngle - self.angle; # Angle difference
        # Step 6
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Calculate estimation error covariance - Update the error covariance
        # Step 7
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle




pub = rospy.Publisher('filtered_yaw', Float32, queue_size=10)
rospy.init_node('yaw_filterer', anonymous=True)


kalman = Kalman()

def yaw_angle_recieved(data):
    global kalman
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    dt = get_dt()
    if dt > 0:
        filtered_yaw = kalman.getKalmanAngle(data.data) # Calculate the angle using a Kalman filter
        rospy.loginfo("filtered yaw : "+filtered_yaw)
        pub.publish(filtered_yaw)
    

rospy.Subscriber("yaw_angle", Float32, yaw_angle_recieved)

# spin() simply keeps python from exiting until this node is stopped
rospy.spin()