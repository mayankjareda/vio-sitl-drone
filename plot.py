import numpy as np
import matplotlib
matplotlib.use('Agg')            # window ki zaroorat nahi, seedha file mein save
import matplotlib.pyplot as plt

d = np.genfromtxt('vio_log.csv', delimiter=',', names=True)   # CSV padho, columns naam se
t = d['t']
vio = np.c_[d['vio_n'], d['vio_e']]       # VIO trajectory (N x 2)
gps = np.c_[d['gps_n'], d['gps_e']]       # GPS trajectory (N x 2)

err = np.linalg.norm(vio - gps, axis=1)   # har waqt dono ke beech ki doori (m)
rmse = np.sqrt(np.mean(err ** 2))         # RMSE: poori flight ki ek accuracy number
path = np.sum(np.linalg.norm(np.diff(gps, axis=0), axis=1))   # GPS path ki lambai
print(f"samples      : {len(t)}")
print(f"RMSE         : {rmse:.2f} m")
print(f"max error    : {err.max():.2f} m")
print(f"final error  : {err[-1]:.2f} m")
print(f"GPS path     : {path:.1f} m  -> drift = {100 * err[-1] / max(path, 1e-6):.1f} % of distance")

fig, ax = plt.subplots(1, 3, figsize=(17, 5))

ax[0].plot(gps[:, 1], gps[:, 0], label='GPS')          # x = East, y = North (map view)
ax[0].plot(vio[:, 1], vio[:, 0], label='VIO')
ax[0].plot(gps[0, 1], gps[0, 0], 'go', label='start')
ax[0].set_xlabel('East (m)'); ax[0].set_ylabel('North (m)')
ax[0].axis('equal'); ax[0].grid(True); ax[0].legend(); ax[0].set_title('Trajectory (top view)')

ax[1].plot(t, gps[:, 0], label='GPS North'); ax[1].plot(t, vio[:, 0], '--', label='VIO North')
ax[1].plot(t, gps[:, 1], label='GPS East');  ax[1].plot(t, vio[:, 1], '--', label='VIO East')
ax[1].set_xlabel('time (s)'); ax[1].set_ylabel('position (m)')
ax[1].grid(True); ax[1].legend(); ax[1].set_title('Position vs time')

ax[2].plot(t, err)
ax[2].set_xlabel('time (s)'); ax[2].set_ylabel('error (m)')
ax[2].grid(True); ax[2].set_title(f'Error (RMSE {rmse:.2f} m)')

plt.tight_layout()
plt.savefig('gps_vs_vio.png', dpi=150)
print("saved gps_vs_vio.png")
