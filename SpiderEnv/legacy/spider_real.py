from ServotorAPI import *
from gym_spider import *
from time import time

s = ServotorAPI(degree_type="std")
s.center_servos()

t = time()
for i in range(100):
    n = np.sin(i/5)*90
    for servo in (5, 6, 7, 9, 10, 11, 13, 14, 15, 16, 17, 18, 20, 21, 22, 24, 25, 26, 31):
        s.move_servo(servo, n)