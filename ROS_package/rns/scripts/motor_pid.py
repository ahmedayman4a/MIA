class MotorPID:
    def __init__(self,target_rpm,Kp,Ki,Kd,rpm_clamp_limit = 0):
        self.target_rpm = target_rpm
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self._rpm_clamp_limit = rpm_clamp_limit
        

    def clamp_saturation(self,controlled_rpm):
        # the rpm at which the integrator is turned off to prevent integral wind-up
        # for explanation, refer to : https://www.youtube.com/watch?v=NVLXCwc8HzM
        if self._rpm_clamp_limit > 0 and abs(controlled_rpm) > self._rpm_clamp_limit:
            return self._rpm_clamp_limit
        else:
            return controlled_rpm

    def get_controlled_rpm(self,rpm,dt):
        stop_integral_update = False
        
        error = self.target_rpm - rpm
        newIntegral = self._eIntegral + error
        eDerivative = -(rpm - self._last_rpm) / dt
        
        controlled_rpm = self.Kp * error + self.Ki * newIntegral  + self.Kd * eDerivative
        
        
        clamped_rpm = self.clamp_saturation(controlled_rpm)
        # if rpm has been clamped and integrator is still trying to go past saturation, then turn off integrator
        if(clamped_rpm != controlled_rpm and error * controlled_rpm > 0):
            stop_integral_update = True
            
        
        # if integral isn't turned off update eIntegral
        if not stop_integral_update:
            self._eIntegral = newIntegral
        self._last_rpm = rpm
        return clamped_rpm