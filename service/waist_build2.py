"""
v4ビルド: 四足ロボット2.stl (脚を腰XL330位置に取付けた正しい配置) から再構築。

分割は新旧STLの形状マッチング差分で自動判定:
  - 移動した部品(±32mm in Y) = 脚ユニット (駆動XL330×2 + ブラケット×3 + リンク×4)
  - 不動の部品 = ボディコア (ハウジング・腰XL330・タブ含む)
リンク部品は従来通りボディ側サーボ軸へ整列(剛体変換)する。
"""
import trimesh, numpy as np, networkx as nx, json, os
from collections import defaultdict

import config

# 入力STLと出力先は .env で指定 (STL_SRC_V1 / STL_SRC_V2 / BUILD_OUT_DIR)
SRC1 = config.STL_SRC_V1
SRC2 = config.STL_SRC_V2
OUT = config.BUILD_OUT_DIR

m1 = trimesh.load(SRC1); m2 = trimesh.load(SRC2)
c1 = m1.split(only_watertight=False); c2 = m2.split(only_watertight=False)

def sig(c): return tuple(np.round(c.extents,1)) + (c.faces.shape[0],)
def center(c): return c.bounds.mean(axis=0)

# ---- 新旧マッチングで「移動した部品」を特定 ----
by1 = defaultdict(list)
for c in c1: by1[sig(c)].append(c)
moved, static = [], []
for s in set(sig(c) for c in c2):
    lst2 = [c for c in c2 if sig(c)==s]; lst1 = list(by1[s])
    for b in lst2:
        cb = center(b)
        i = int(np.argmin([np.linalg.norm(center(a)-cb) for a in lst1]))
        d = cb - center(lst1.pop(i))
        (moved if np.linalg.norm(d) > 0.5 else static).append(b)
print(f'moved={len(moved)} static={len(static)}')
assert len(moved) == 36

# ---- 検出ユーティリティ ----
def fit_circle(pts):
    A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]
    b = (pts**2).sum(axis=1)
    sol,*_ = np.linalg.lstsq(A,b,rcond=None)
    cx,cy = sol[0],sol[1]
    r = np.sqrt(max(sol[2]+cx**2+cy**2,0))
    return cx,cy,r,np.abs(np.hypot(pts[:,0]-cx,pts[:,1]-cy)-r).mean()

def cylinders(c):
    fn = c.face_normals
    side = np.where(np.abs(fn[:,1])<0.2)[0]; ss=set(side.tolist())
    G = nx.Graph(); G.add_nodes_from(side.tolist())
    for a,b in c.face_adjacency:
        if a in ss and b in ss: G.add_edge(a,b)
    out=[]
    for grp in nx.connected_components(G):
        grp=list(grp)
        if len(grp)<8: continue
        p3 = c.vertices[np.unique(c.faces[grp])]
        cx,cz,r,resid = fit_circle(p3[:,[0,2]])
        if resid<0.15 and 1.0<r<20:
            out.append(dict(x=cx,z=cz,r=r))
    return out

def merge_axes(cyl, tol=1.0):
    axes=[]
    for c in cyl:
        for a in axes:
            if abs(a['x']-c['x'])<tol and abs(a['z']-c['z'])<tol:
                a['items'].append(c); break
        else: axes.append(dict(x=c['x'],z=c['z'],items=[c]))
    for a in axes:
        a['x']=np.mean([i['x'] for i in a['items']])
        a['z']=np.mean([i['z'] for i in a['items']])
        a['rmax']=max(i['r'] for i in a['items'])
    return axes

def rigid2d(a1,a2,b1,b2):
    va=np.array(a2)-np.array(a1); vb=np.array(b2)-np.array(b1)
    phi=np.arctan2(vb[1],vb[0])-np.arctan2(va[1],va[0])
    cp,sp=np.cos(phi),np.sin(phi)
    R=np.array([[cp,0,-sp],[0,1,0],[sp,0,cp]])
    t=np.array([b1[0],0,b1[1]])-R@np.array([a1[0],0,a1[1]])
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=t
    return T

def apply(mesh,T,dy=0.0,dz=0.0):
    mm=mesh.copy(); T2=T.copy(); T2[1,3]+=dy; T2[2,3]+=dz
    mm.apply_transform(T2); return mm

# ---- 部品分類 (移動した部品を役割別に) ----
def role(c):
    e = np.round(c.extents,1)
    if tuple(e)==(41.4,9.0,83.3): return 'leg'
    if tuple(e)==(57.5,15.0,57.5): return 'rod'
    if e[0] in (29.5,26.8) and e[2] in (29.5,31.7): return 'crank'
    if tuple(e)==(20.0,25.0,34.0): return 'servo'
    return 'bracket'

def leg_of(c):
    ct = center(c)
    return ('f' if ct[0]>4.55 else 'r') + ('l' if ct[1]>0 else 'r')

groups = {k: defaultdict(list) for k in ['rr','fr','rl','fl']}
for c in moved:
    groups[leg_of(c)][role(c)].append(c)
for k,g in groups.items():
    print(k, {r:len(v) for r,v in g.items()})

# ---- 理想化ジオメトリ (XZはv3と同一 / Yは新配置の実測から) ----
AX = dict(rear_a1=-49.95, rear_a2=-75.45, front_a1=59.05, front_a2=84.55, z=90.5)
WAIST = dict(y=70.23, z=97.0)
MIRROR = dict(rl='rr', fl='fr', rr=None, fr=None)

def measure(g):
    legm = g['leg'][0]; linkm = g['rod'][0]
    la = [a for a in merge_axes(cylinders(legm))
          if any(abs(i['r']-2.0)<0.3 for i in a['items'])]
    la.sort(key=lambda a:-a['z'])
    U=np.array([la[0]['x'],la[0]['z']]); L=np.array([la[1]['x'],la[1]['z']])
    vi=legm.vertices[:,2].argmin()
    F=np.array([legm.vertices[vi,0],legm.vertices[vi,2]])
    cr=[]
    for cm in g['crank']:
        ax=merge_axes(cylinders(cm))
        horn=[a for a in ax if any(abs(i['r']-2.0)<0.35 for i in a['items'])
              and not any(i['r']>2.6 for i in a['items'])]
        pin =[a for a in ax if any(i['r']>2.6 for i in a['items'])]
        A=np.mean([[h['x'],h['z']] for h in horn],axis=0)
        P=np.array([pin[0]['x'],pin[0]['z']])
        cr.append(dict(mesh=cm,A=A,P=P,r=float(np.linalg.norm(P-A)),
                       yc=float(cm.bounds[:,1].mean())))
    if np.linalg.norm(cr[0]['P']-U)>np.linalg.norm(cr[1]['P']-U): cr=cr[::-1]
    lka=[a for a in merge_axes(cylinders(linkm)) if a['rmax']>1.5]
    lka.sort(key=lambda a:np.hypot(a['x']-cr[1]['P'][0],a['z']-cr[1]['P'][1]))
    RP=np.array([lka[0]['x'],lka[0]['z']]); RL_=np.array([lka[1]['x'],lka[1]['z']])
    # 駆動サーボの出力ボスz実測 (ユニットの上下ずれ補正用)
    dz = 0.0
    for sv in g['servo']:
        for a in merge_axes(cylinders(sv)):
            if 7.5 < a['rmax'] < 8.5:
                dz = AX['z'] - a['z']; break
    return dict(legm=legm,linkm=linkm,c1=cr[0],c2=cr[1],U=U,L=L,F=F,rodP=RP,rodL=RL_,
                y_leg=float(legm.bounds[:,1].mean()),y_link=float(linkm.bounds[:,1].mean()),
                unit_dz=dz)

meas = {k: measure(g) for k,g in groups.items()}

os.makedirs(f'{OUT}/meshes',exist_ok=True)
params={}
for name in ['rr','fr','rl','fl']:
    mv = meas[name]; g = groups[name]
    ref = meas[MIRROR[name]] if MIRROR[name] else mv
    s = +1 if name.endswith('l') else -1
    front = name.startswith('f')
    yp = dict(c1=s*abs(ref['c1']['yc']), c2=s*abs(ref['c2']['yc']),
              leg=s*abs(ref['y_leg']),   link=s*abs(ref['y_link']))
    A1n=np.array([AX['front_a1' if front else 'rear_a1'], AX['z']])
    A2n=np.array([AX['front_a2' if front else 'rear_a2'], AX['z']])
    th1=np.arctan2(*(mv['c1']['P']-mv['c1']['A'])[::-1])
    th2=np.arctan2(*(mv['c2']['P']-mv['c2']['A'])[::-1])
    r1,r2=mv['c1']['r'],mv['c2']['r']
    U_n=A1n+r1*np.array([np.cos(th1),np.sin(th1)])
    P2_n=A2n+r2*np.array([np.cos(th2),np.sin(th2)])
    coupler=np.linalg.norm(mv['L']-mv['U']); rod=np.linalg.norm(mv['rodL']-mv['rodP'])
    dv=P2_n-U_n; dd=np.linalg.norm(dv)
    x=(dd*dd+coupler**2-rod**2)/(2*dd); h=np.sqrt(max(coupler**2-x*x,0))
    e=dv/dd; nv=np.array([-e[1],e[0]])
    L_n=min([U_n+x*e+h*nv, U_n+x*e-h*nv], key=lambda q:np.linalg.norm(q-mv['L']))

    apply(mv['c1']['mesh'], rigid2d(mv['c1']['A'],mv['c1']['P'],A1n,U_n),  yp['c1']-mv['c1']['yc']).export(f'{OUT}/meshes/{name}_crank1.stl')
    apply(mv['c2']['mesh'], rigid2d(mv['c2']['A'],mv['c2']['P'],A2n,P2_n), yp['c2']-mv['c2']['yc']).export(f'{OUT}/meshes/{name}_crank2.stl')
    Tlg=rigid2d(mv['U'],mv['L'],U_n,L_n)
    apply(mv['legm'], Tlg, yp['leg']-mv['y_leg']).export(f'{OUT}/meshes/{name}_leg.stl')
    apply(mv['linkm'], rigid2d(mv['rodP'],mv['rodL'],P2_n,L_n), yp['link']-mv['y_link']).export(f'{OUT}/meshes/{name}_rod.stl')
    F3=Tlg@np.array([mv['F'][0],0,mv['F'][1],1]); F_n=np.array([F3[0],F3[2]])

    hip=trimesh.util.concatenate([c.copy() for c in g['servo']+g['bracket']])
    hip.apply_translation([0,0,mv['unit_dz']])
    hip.export(f'{OUT}/meshes/{name}_hip.stl')

    params[name]=dict(
        A1=A1n.tolist(),A2=A2n.tolist(),r1=r1,r2=r2,
        pin1=U_n.tolist(),pin2=P2_n.tolist(),U=U_n.tolist(),L=L_n.tolist(),F=F_n.tolist(),
        coupler=float(coupler),rod=float(rod),
        y_crank1=yp['c1'],y_crank2=yp['c2'],y_leg=yp['leg'],y_link=yp['link'],
        side=s,
        waist=dict(y=s*WAIST['y'], z=WAIST['z'],
                   x=(A1n[0]+A2n[0])/2))
    print(f"[{name}] y_leg={yp['leg']:+.1f} y_cranks=({yp['c1']:+.1f},{yp['c2']:+.1f}) "
          f"unit_dz={mv['unit_dz']:+.1f} closure={np.linalg.norm(P2_n-L_n)-rod:+.4f}mm")

body = trimesh.util.concatenate(static)
body.export(f'{OUT}/meshes/body.stl')
json.dump(params, open(f'{OUT}/leg_params.json','w'), indent=1)
print(f'written under {OUT} (conf/ と meshes/ へ手動で反映)')
print(f'core: {len(static)} parts. done')
