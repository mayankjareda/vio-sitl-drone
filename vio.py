#!/usr/bin/env python3
import sys, time, threading, csv, queue
import numpy as np
import cv2
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

CAM_TOPIC = sys.argv[1] if len(sys.argv) > 1 else '/vio/down_camera'  # camera topic
HFOV = 1.047                      # SDF ke horizontal_fov se SAME hona chahiye (radian)

state = {'att': (0.0, 0.0, 0.0), 'h': 0.0, 'gps': None}   # MAVLink se latest values
lock = threading.Lock()           # do threads ek hi dict use karte hain, isliye lock
frames = queue.Queue(maxsize=2)   # image callback se main loop tak images ka pipe

def mav_thread():                 # background mein chalta hai
    m = mavutil.mavlink_connection('udpin:127.0.0.1:14561')   # MAVProxy ka doosra output
    m.wait_heartbeat()
    while True:
        msg = m.recv_match(type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'GPS_RAW_INT'],
                           blocking=True)
        t = msg.get_type()
        with lock:
            if t == 'ATTITUDE':
                state['att'] = (msg.roll, msg.pitch, msg.yaw)        # radian
            elif t == 'GLOBAL_POSITION_INT':
                state['h'] = msg.relative_alt / 1000.0               # mm -> m
            elif msg.fix_type >= 3:                                  # GPS_RAW_INT, 3D fix
                state['gps'] = (msg.lat * 1e-7, msg.lon * 1e-7)      # degree

def on_image(msg):                # Gazebo har image par ye chalata hai
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)    # tracking grayscale par hoti hai
    with lock:
        snap = (state['att'], state['h'], state['gps'])   # image ke time ki attitude/height
    try:
        frames.put_nowait((gray, *snap))
    except queue.Full:
        pass                      # main loop slow ho to purani frame chhod do

def R_ned_body(roll, pitch, yaw):  # body (FRD) -> NED rotation matrix
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([[cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy],
                     [cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy],
                     [-sp,   sr*cp,            cr*cp]])

# camera axes body mein: image right = body right, image down = body back,
# viewing direction = body down. Ye drone mein camera ki mounting se aata hai.
R_BC = np.array([[0, -1, 0],
                 [1,  0, 0],
                 [0,  0, 1]], dtype=float)

def ground_vecs(pts, R, h, Kinv):
    uv1 = np.hstack([pts, np.ones((len(pts), 1))])      # (u,v) -> (u,v,1)
    rays = (R @ R_BC @ Kinv @ uv1.T).T                  # pixel -> camera ray -> NED ray
    ok = rays[:, 2] > 0.2                               # sirf neeche ki taraf jaati rays
    t = np.where(ok, h / np.maximum(rays[:, 2], 1e-6), 0.0)   # ray ko ground tak badhao
    return rays[:, :2] * t[:, None], ok                 # ground point ka (North, East)

def latlon_to_ne(lat, lon, lat0, lon0):                 # GPS degree -> metre
    n = (lat - lat0) * 111320.0
    e = (lon - lon0) * 111320.0 * np.cos(np.radians(lat0))
    return n, e

node = Node()
node.subscribe(Image, CAM_TOPIC, on_image)              # bina ROS ke camera subscribe
threading.Thread(target=mav_thread, daemon=True).start()

Kinv = None
prev_gray = prev_pts = prev_R = prev_h = None
pos = np.zeros(2)                 # VIO position (North, East), start par (0,0)
gps0 = None                       # GPS ka origin
n = e = 0.0
last_print = 0.0
log = open('vio_log.csv', 'w', newline='')
wr = csv.writer(log)
wr.writerow(['t', 'vio_n', 'vio_e', 'gps_n', 'gps_e', 'h'])
t0 = time.time()
print("Waiting for images... takeoff karo")

try:
    while True:
        gray, att, h, gps = frames.get()
        if Kinv is None:                                 # pehli image par intrinsics banao
            H, W = gray.shape
            fx = (W / 2) / np.tan(HFOV / 2)              # focal length (pixel)
            Kinv = np.linalg.inv(np.array([[fx, 0, W/2], [0, fx, H/2], [0, 0, 1]]))
        if h < 1.0 or gps is None:                       # zameen par ho to tracking nahi
            prev_gray = None
            continue
        R = R_ned_body(*att)
        if gps0 is None:
            gps0 = gps                                   # VIO aur GPS ek hi point se shuru
        if prev_gray is None or prev_pts is None or len(prev_pts) < 50:
            pts = cv2.goodFeaturesToTrack(gray, 300, 0.01, 10)
            if pts is not None:
                prev_gray, prev_pts, prev_R, prev_h = gray, pts, R, h
            continue
        cur, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None,
                                              winSize=(21, 21), maxLevel=3)
        good = st.ravel() == 1                           # jo features track hue
        p0 = prev_pts[good].reshape(-1, 2)
        p1 = cur[good].reshape(-1, 2)
        vp, okp = ground_vecs(p0, prev_R, prev_h, Kinv)
        vc, okc = ground_vecs(p1, R, h, Kinv)
        ok = okp & okc
        if ok.sum() >= 20:
            pos += np.median(vp[ok] - vc[ok], axis=0)    # robust displacement, add karo
        n, e = latlon_to_ne(*gps, *gps0)
        now = time.time() - t0
        wr.writerow([f'{now:.3f}', pos[0], pos[1], n, e, h])
        log.flush()
        if now - last_print > 1.0:                       # har second ek status line
            last_print = now
            print(f"t={now:6.1f}  VIO N={pos[0]:6.2f} E={pos[1]:6.2f} | "
                  f"GPS N={n:6.2f} E={e:6.2f} | h={h:5.1f} feats={int(ok.sum())}")
        prev_gray, prev_R, prev_h = gray, R, h
        prev_pts = p1[ok].reshape(-1, 1, 2).astype(np.float32)
        if len(prev_pts) < 150:                          # features kam ho to naye dhundo
            extra = cv2.goodFeaturesToTrack(gray, 300, 0.01, 10)
            if extra is not None:
                prev_pts = extra
except KeyboardInterrupt:
    print("\nstopped, log saved in vio_log.csv")
finally:
    log.close()
