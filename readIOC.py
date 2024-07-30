from cothread.catools import caget, caput, camonitor
import time

class test:
    def __init__(self):
        self.start = time.time()
        self.end = 0
        self.count = 0

    def counter(self,*args):
        self.count += 1
        print(self.count)
    
    def printf(self,*args):
        print(f'{time.time()},')

test = test()

#camonitor("LA84R-DI-DCAM-01:CAM:Acquire",test.counter)
#camonitor("mecaRobot:Checkpoint",test.counter)
#camonitor("LA84R-DI-DCAM-01:CAM:Acquire",print)
#camonitor("mecaRobot:Checkpoint",test.freq)
camonitor("mecaRobot:Checkpoint",test.printf)