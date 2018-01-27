

class Statemachine():
    def idle(self):
        #state that runs by default
        pass
    def estop(self, rexarm):
        #sets the torque of all motors to 0, in case of emergency
        rexarm.max_torque = [0.0, 0.0, 0.0, 0.0]
        rexarm.cmd_publish()
        
    
