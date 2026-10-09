from pymavlink import mavutil
import time

ALT = 10.0      # takeoff altitude (metres)
SIDE = 10.0     # square ki side (metres)

# udpin = is port par SUNO. MAVProxy yahan data bhej raha hai.
master = mavutil.mavlink_connection('udpin:127.0.0.1:14560')
master.wait_heartbeat()          # jab tak drone ka heartbeat na aaye, ruko
print("Connected to system", master.target_system)

def set_mode(name):
    master.set_mode(master.mode_mapping()[name])   # mode ka naam -> number (GUIDED, LAND)
    time.sleep(1)

def arm():
    while not master.motors_armed():               # jab tak armed na ho, baar baar try
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)                # param1 = 1 matlab ARM
        master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        time.sleep(1)

def takeoff(alt):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)                  # last value = target altitude

def rel_alt():
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg.relative_alt / 1000.0               # mm se metre

def local_pos():
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    return msg.x, msg.y, msg.z                     # North, East, Down

def goto_ned(n, e, d):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0x0DF8,                                    # sirf position use karo, baaki ignore
        n, e, d, 0, 0, 0, 0, 0, 0, 0, 0)
    while True:                                    # pahunchne tak wait
        x, y, z = local_pos()
        if abs(x - n) < 1 and abs(y - e) < 1 and abs(z - d) < 1:
            break

set_mode('GUIDED')
arm()
takeoff(ALT)
while rel_alt() < ALT * 0.95:                      # 95% height tak wait
    time.sleep(0.5)
print("Reached altitude")

for n, e in [(SIDE, 0), (SIDE, SIDE), (0, SIDE), (0, 0)]:   # square flight
    goto_ned(n, e, -ALT)                           # NED mein upar = negative z
    print("Reached waypoint", n, e)

set_mode('LAND')
while rel_alt() > 0.2:
    time.sleep(0.5)
print("Landed")
