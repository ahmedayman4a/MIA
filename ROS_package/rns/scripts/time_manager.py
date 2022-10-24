import rospy

class TimeHelper:

    def __init__(self):
        self._previous_time = 0
        # self._coefficient = 1
    
    def get_dt(self):
        # if this is the first reading there is no dt
        if self._previous_time == 0.:
            self._previous_time = rospy.get_time()
            return 0
        
        current_time = rospy.get_time()
        dt = (current_time - self._previous_time); # Calculate delta time
        self._previous_time = current_time
        return dt