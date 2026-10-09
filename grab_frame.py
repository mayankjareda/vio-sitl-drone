import time
import numpy as np
import cv2
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

got = {}                                       # yahan latest image rakhenge

def cb(msg):                                   # Gazebo har image par ye function chalata hai
    got['m'] = msg

node = Node()                                  # gz-transport ka node (ROS ka replacement)
node.subscribe(Image, '/vio/down_camera', cb)  # topic par subscribe

for _ in range(100):                           # max 10 second tak image ka wait
    if 'm' in got:
        break
    time.sleep(0.1)

m = got['m']
print("width", m.width, "height", m.height, "format", m.pixel_format_type, "bytes", len(m.data))
img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)   # raw bytes -> image array
cv2.imwrite('frame.png', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))        # OpenCV BGR mangta hai
print("saved frame.png")
