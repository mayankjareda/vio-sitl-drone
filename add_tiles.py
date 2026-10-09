import random

P = '/home/chaitanya/ardupilot_gazebo/worlds/iris_vio_world.sdf'
s = open(P).read()
if 'name="tiles"' in s:                       # dobara chalane par double insert na ho
    raise SystemExit("tiles already added")

random.seed(1)                                # same seed = har baar same pattern (repeatable)
vis = []
n = 0
for gx in range(-8, 9):                       # 17 x 17 grid
    for gy in range(-8, 9):
        x = gx * 3 + random.uniform(-0.15, 0.15)   # 3 m cell, thoda jitter
        y = gy * 3 + random.uniform(-0.15, 0.15)
        if x * x + y * y < 2.25:              # drone ke spawn point (0,0) ke paas tile nahi
            continue
        sx, sy = random.uniform(0.8, 1.9), random.uniform(0.8, 1.9)  # random size
        yaw = random.uniform(0, 3.14)         # random rotation
        r, g, b = random.random(), random.random(), random.random()  # random rang
        vis.append(f'''      <visual name="t{n}">
        <pose>{x:.2f} {y:.2f} 0.01 0 0 {yaw:.2f}</pose>
        <geometry><box><size>{sx:.2f} {sy:.2f} 0.02</size></box></geometry>
        <material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse></material>
      </visual>''')
        n += 1

model = ('    <model name="tiles"><static>true</static>\n    <link name="l">\n'
         + '\n'.join(vis) + '\n    </link></model>\n')
i = s.rfind('</world>')                       # world ke band hone se pehle insert
open(P, 'w').write(s[:i] + model + s[i:])
print("added", n, "tiles")
