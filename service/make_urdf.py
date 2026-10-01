"""quadruped.urdf 生成 v3 (12自由度: 腰ロール4 + 脚5節リンク8)

チェーン: base --waist(X軸ロール,駆動)--> hip(脚ユニット)
            --hip1(Y軸,駆動)--> crank1 --knee(受動)--> leg(足)
            --hip2(Y軸,駆動)--> crank2 --rodj(受動)--> rod
ループ閉鎖(rod-leg, 点L)は loop_constraints.json でシミュレータ側にて。
hip1/hip2/knee/rodj の軸は子フレーム定義なので腰ロールに追従する。
"""
import json, numpy as np, os

import config

P = json.load(open(config.LEG_PARAMS))
S = 0.001
FOOT_R = 7.5 * S
WAIST_RANGE = 0.7   # [rad] 腰ロール可動域 ±40deg (ソフト制限)

def box_inertia(m, x, y, z, floor=1e-4):
    return (max(m/12*(y*y+z*z),floor), max(m/12*(x*x+z*z),floor), max(m/12*(x*x+y*y),floor))

def inertial(m, ix, iy, iz):
    return f'''    <inertial>
      <mass value="{m}"/>
      <inertia ixx="{ix:.3e}" ixy="0" ixz="0" iyy="{iy:.3e}" iyz="0" izz="{iz:.3e}"/>
    </inertial>'''

def mesh_visual(fname, offset, color):
    ox, oy, oz = offset
    return f'''    <visual>
      <origin xyz="{ox:.6f} {oy:.6f} {oz:.6f}" rpy="0 0 0"/>
      <geometry><mesh filename="{config.URDF_MESH_PREFIX}/{fname}" scale="{S} {S} {S}"/></geometry>
      <material name="{color}"/>
    </visual>'''

u = ['<?xml version="1.0"?>', '<robot name="linkage_quadruped">']
for name, rgba in [('grey','0.75 0.75 0.78 1'), ('dark','0.25 0.25 0.28 1'),
                   ('orange','0.95 0.55 0.15 1'), ('blue','0.25 0.5 0.9 1'),
                   ('green','0.55 0.58 0.62 1')]:
    u.append(f'  <material name="{name}"><color rgba="{rgba}"/></material>')

bi = box_inertia(0.55, 0.24, 0.13, 0.11)
u.append(f'''  <link name="base_link">
{inertial(0.55, *bi)}
{mesh_visual('body.stl', (0,0,0), 'grey')}
    <collision>
      <origin xyz="0 0 0.1165" rpy="0 0 0"/>
      <geometry><box size="0.238 0.120 0.107"/></geometry>
    </collision>
  </link>''')

loops = {}
for leg, p in P.items():
    A1=np.array(p['A1']); A2=np.array(p['A2'])
    pin1=np.array(p['pin1']); pin2=np.array(p['pin2'])
    L=np.array(p['L']); F=np.array(p['F'])
    y1,y2,yl,yr = p['y_crank1'],p['y_crank2'],p['y_leg'],p['y_link']
    w = p['waist']

    o_w  = np.array([w['x'], w['y'], w['z']]) * S
    o_c1 = np.array([A1[0], y1, A1[1]]) * S
    o_lg = np.array([pin1[0], yl, pin1[1]]) * S
    o_c2 = np.array([A2[0], y2, A2[1]]) * S
    o_rd = np.array([pin2[0], yr, pin2[1]]) * S

    hi = box_inertia(0.12, 0.105, 0.06, 0.05)
    ci = box_inertia(0.012, 0.03, 0.012, 0.03)
    ri = box_inertia(0.020, 0.055, 0.01, 0.055)
    li = box_inertia(0.045, 0.05, 0.01, 0.09)

    u.append(f'''  <link name="{leg}_hip">
{inertial(0.12, *hi)}
{mesh_visual(f'{leg}_hip.stl', tuple(-o_w), 'green')}
  </link>
  <joint name="{leg}_waist" type="revolute">
    <parent link="base_link"/><child link="{leg}_hip"/>
    <origin xyz="{o_w[0]:.6f} {o_w[1]:.6f} {o_w[2]:.6f}" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="{-WAIST_RANGE}" upper="{WAIST_RANGE}" effort="0.52" velocity="10.8"/>
    <dynamics damping="0.002" friction="0.0"/>
  </joint>
  <link name="{leg}_crank1">
{inertial(0.012, *ci)}
{mesh_visual(f'{leg}_crank1.stl', tuple(-o_c1), 'orange')}
  </link>
  <joint name="{leg}_hip1" type="continuous">
    <parent link="{leg}_hip"/><child link="{leg}_crank1"/>
    <origin xyz="{(o_c1-o_w)[0]:.6f} {(o_c1-o_w)[1]:.6f} {(o_c1-o_w)[2]:.6f}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/><dynamics damping="0.001" friction="0.0"/>
  </joint>
  <link name="{leg}_leg">
{inertial(0.045, *li)}
{mesh_visual(f'{leg}_leg.stl', tuple(-o_lg), 'dark')}
    <collision>
      <origin xyz="{(F[0]-pin1[0])*S:.6f} 0 {(F[1]+FOOT_R/S-pin1[1])*S:.6f}" rpy="0 0 0"/>
      <geometry><sphere radius="{FOOT_R}"/></geometry>
    </collision>
  </link>
  <joint name="{leg}_knee" type="continuous">
    <parent link="{leg}_crank1"/><child link="{leg}_leg"/>
    <origin xyz="{(pin1[0]-A1[0])*S:.6f} {(yl-y1)*S:.6f} {(pin1[1]-A1[1])*S:.6f}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/><dynamics damping="0.0005" friction="0.0"/>
  </joint>
  <link name="{leg}_crank2">
{inertial(0.012, *ci)}
{mesh_visual(f'{leg}_crank2.stl', tuple(-o_c2), 'orange')}
  </link>
  <joint name="{leg}_hip2" type="continuous">
    <parent link="{leg}_hip"/><child link="{leg}_crank2"/>
    <origin xyz="{(o_c2-o_w)[0]:.6f} {(o_c2-o_w)[1]:.6f} {(o_c2-o_w)[2]:.6f}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/><dynamics damping="0.001" friction="0.0"/>
  </joint>
  <link name="{leg}_rod">
{inertial(0.020, *ri)}
{mesh_visual(f'{leg}_rod.stl', tuple(-o_rd), 'blue')}
  </link>
  <joint name="{leg}_rodj" type="continuous">
    <parent link="{leg}_crank2"/><child link="{leg}_rod"/>
    <origin xyz="{(pin2[0]-A2[0])*S:.6f} {(yr-y2)*S:.6f} {(pin2[1]-A2[1])*S:.6f}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/><dynamics damping="0.0005" friction="0.0"/>
  </joint>''')

    ymid = (yl + yr) / 2
    W = np.array([L[0], ymid, L[1]]) * S
    loops[leg] = dict(parent=f'{leg}_rod', child=f'{leg}_leg',
                      pivot_in_rod=(W-o_rd).tolist(), pivot_in_leg=(W-o_lg).tolist())

u.append('</robot>')
open(config.ROBOT_URDF, 'w').write('\n'.join(u))
json.dump(loops, open(config.LOOP_CONSTRAINTS, 'w'), indent=1)
print(f'written: {config.ROBOT_URDF}, {config.LOOP_CONSTRAINTS}')
