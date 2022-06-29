import time
import math
from typing import Union

class PID(object):
    def __init__(self, p_gain: float, i_gain: float, d_gain: float, i_max: float, i_min: float):
        self.set_gains(p_gain, i_gain, d_gain, i_max, i_min)
        self.reset()

    def reset(self):
        """  Reset the state of this PID controller """
        self.p_error_last = 0.0 # Save position state for derivative
        self.p_error = 0.0 # Proportional error.
        self.d_error = 0.0 # Derivative error.
        self.i_error = 0.0 # Integator error.
        self.cmd = 0.0 # Command to send.
        self.last_time = None # Used for automatic calculation of dt.
        
    def set_gains(self, p_gain: float, i_gain: float, d_gain: float, i_max: float, i_min: float): 
        """ Set PID gains for the controller. 
         Parameters:
          p_gain     The proportional gain.
          i_gain     The integral gain.
          d_gain     The derivative gain.
          i_max      The integral upper limit.
          i_min      The integral lower limit. 
        """ 
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.i_max = i_max
        self.i_min = i_min
        
    def update(self, p_error: float, dt: Union[float, None] = None) -> float:
        """  Update the Pid loop with nonuniform time step size.

        Parameters:
          p_error  Error since last call (p_state - p_target)
          dt       Change in time since last call, in seconds, or None. 
                   If dt is None, then the system clock will be used to 
                   calculate the time since the last update. 
        """
        if dt == None:
            cur_time = time.time()
            if self.last_time is None:
                self.last_time = cur_time 
            dt = cur_time - self.last_time
            self.last_time = cur_time
            
        self.p_error = p_error # this is pError = pState-pTarget
        if dt == 0 or math.isnan(dt) or math.isinf(dt):
            return 0.0

        # Calculate proportional contribution to command
        p_term = self.p_gain * self.p_error

        # Calculate the integral error
        self.i_error += dt * self.p_error
        
        # Calculate integral contribution to command
        i_term = self.i_gain * self.i_error
        
        # Limit i_term so that the limit is meaningful in the output
        if i_term > self.i_max and self.i_gain != 0:
            i_term = self.i_max
            self.i_error = i_term / self.i_gain
        elif i_term < self.i_min and self.i_gain != 0:
            i_term = self.i_min
            self.i_error = i_term / self.i_gain
            
        # Calculate the derivative error
        self.d_error = (self.p_error - self.p_error_last) / dt
        self.p_error_last = self.p_error
        
        # Calculate derivative contribution to command 
        d_term = self.d_gain * self.d_error
        
        self.cmd = p_term + i_term + d_term

        return self.cmd