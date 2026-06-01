#!/usr/bin/env python3
"""
Bag回放验证: 模拟 fusion 节点在 nav_drift_test2 包上的完整行为
================================================================
输出:
  1. 状态转换时间线 (什么时候进入/退出 DEGRADED)
  2. NDT-only vs Fused 轨迹对比 (定位有没有被纠正)
  3. 关键路点的定位误差对比
"""
import sys, struct, math, json, time
from collections import defaultdict
sys.path.insert(0, '/usr/lib/python3/dist-packages')
from mcap.reader import make_reader

BAG = '/home/ubuntu/nav_drift_test2/nav_drift_test2_0.mcap'
WP = '/home/ubuntu/humanoid_ws/data/dynamic_waypoints.json'

# ══════════════════════════════════════════════════════════
# 参数 (与 localization_odom_fusion.py 完全一致)
# ══════════════════════════════════════════════════════════
DEGRADED_ERR = 0.5; HEALTHY_ERR = 0.15
HEALTHY_CONSEC = 3; DEGRADED_CONSEC = 2
MAX_DEGRADED_SEC = 120; MAX_ODOM_DISP_M = 30
TRANSITION_SEC = 2.0

# ══════════════════════════════════════════════════════════
# CDR helpers
# ══════════════════════════════════════════════════════════
def parse_cdr_string(data, offset):
    strlen = struct.unpack_from('<I', data, offset)[0]; offset += 4
    if strlen > 10000: return None, offset
    s = data[offset:offset+strlen].decode('utf-8', errors='replace').rstrip('\x00')
    offset += strlen; offset += (4-(strlen%4))%4
    return s, offset
def skip_rtps(data):
    if len(data)<4: return 0
    e=struct.unpack_from('<H',data,0)[0]; return 4 if e in(0x0001,0x0100) else 0
def quat_to_yaw(qx,qy,qz,qw):
    return math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
def smoothstep(t):
    t=max(0.,min(1.,t)); return t*t*(3-2*t)

# ══════════════════════════════════════════════════════════
# Load waypoints
# ══════════════════════════════════════════════════════════
with open(WP) as f:
    wps_data = json.load(f)
wps = []
for k,v in sorted(wps_data['waypoints']['navigation_target'].items(), key=lambda x:int(x[0])):
    wps.append({'id':v['id'],'name':v['name'],'x':v['position'][0],'y':v['position'][1]})
print(f"路点: {len(wps)}个")

# ══════════════════════════════════════════════════════════
# Extract bag data
# ══════════════════════════════════════════════════════════
print("提取bag数据...")
t0=time.time()
real_poses=[]; status_samples=[]; map_odom=[]; odom_body=[]

with open(BAG,'rb') as f:
    reader=make_reader(f)
    for schema,channel,msg in reader.iter_messages(
        topics=['/robot_realpose','/tf','/status']):
        t=msg.log_time/1e9
        if channel.topic=='/robot_realpose':
            d=msg.data
            if len(d)<8: continue
            try:
                o=skip_rtps(d)
                struct.unpack_from('<i',d,o)[0];o+=4
                struct.unpack_from('<I',d,o)[0];o+=4
                fi,o=parse_cdr_string(d,o)
                if fi is None: continue
                if o+56>len(d): continue
                px=struct.unpack_from('<d',d,o)[0];o+=8
                py=struct.unpack_from('<d',d,o)[0];o+=8
                pz=struct.unpack_from('<d',d,o)[0];o+=8
                ox=struct.unpack_from('<d',d,o)[0];o+=8
                oy=struct.unpack_from('<d',d,o)[0];o+=8
                oz=struct.unpack_from('<d',d,o)[0];o+=8
                ow=struct.unpack_from('<d',d,o)[0];o+=8
                if all(math.isfinite(v) for v in(px,py,ox,oy,oz,ow)):
                    real_poses.append((t,px,py,pz,quat_to_yaw(ox,oy,oz,ow)))
            except: continue
        elif channel.topic=='/tf':
            d=msg.data
            if len(d)<8: continue
            try:
                o=skip_rtps(d)
                n=struct.unpack_from('<I',d,o)[0];o+=4
                for _ in range(min(n,20)):
                    if o+16>len(d): break
                    struct.unpack_from('<i',d,o)[0];o+=4
                    struct.unpack_from('<I',d,o)[0];o+=4
                    fi,o=parse_cdr_string(d,o)
                    ci,o=parse_cdr_string(d,o)
                    if fi is None or ci is None: break
                    if len(fi)>60 or len(ci)>60: break
                    if o+56>len(d): break
                    tx=struct.unpack_from('<d',d,o)[0];o+=8
                    ty=struct.unpack_from('<d',d,o)[0];o+=8
                    tz=struct.unpack_from('<d',d,o)[0];o+=8
                    qx=struct.unpack_from('<d',d,o)[0];o+=8
                    qy=struct.unpack_from('<d',d,o)[0];o+=8
                    qz=struct.unpack_from('<d',d,o)[0];o+=8
                    qw=struct.unpack_from('<d',d,o)[0];o+=8
                    if (fi,ci)==('map','odom'):
                        map_odom.append((t,tx,ty,tz,qx,qy,qz,qw))
                    elif (fi,ci)==('camera_init','body'):
                        odom_body.append((t,tx,ty,tz,qx,qy,qz,qw))
            except: continue
        elif channel.topic=='/status':
            d=msg.data
            try:
                o=skip_rtps(d)
                ci=d.find(b'camera_init',o)
                if ci<0: continue
                sl=ci-4; sv=struct.unpack_from('<I',d,sl)[0]
                if sv>100: continue
                a=ci+sv; a+=(4-(sv%4))%4
                if a+12>len(d): continue
                hc=struct.unpack_from('<I',d,a)[0]!=0
                me=struct.unpack_from('<f',d,a+4)[0]
                inf=struct.unpack_from('<f',d,a+8)[0]
                if math.isfinite(me): status_samples.append((t,me,hc,inf))
            except: continue

print(f"提取完成 ({time.time()-t0:.0f}s): real_pose={len(real_poses)} status={len(status_samples)} mo={len(map_odom)} ob={len(odom_body)}")

# ══════════════════════════════════════════════════════════
# Align data
# ══════════════════════════════════════════════════════════
def find_nearest(samples, target_t, max_dt=1.0):
    if not samples: return None
    lo,hi=0,len(samples)-1; best_idx,best_dt=None,max_dt
    while lo<=hi:
        mid=(lo+hi)//2; dt=abs(samples[mid][0]-target_t)
        if dt<best_dt: best_dt=dt; best_idx=mid
        if samples[mid][0]<target_t: lo=mid+1
        else: hi=mid-1
    return samples[best_idx] if best_idx is not None else None

print("对齐数据...")
aligned=[]
for rp in real_poses:
    t,rx,ry,rz,ryaw=rp
    st=find_nearest(status_samples,t)
    ob=find_nearest(odom_body,t)
    mo=find_nearest(map_odom,t)
    me=st[1] if st else float('nan')
    hc=st[2] if st else False
    aligned.append({'t':t,'rx':rx,'ry':ry,'me':me,'hc':hc,'ob':ob,'mo':mo})
print(f"对齐: {len(aligned)} 帧 ({len(aligned)//10}s @10Hz)")

# ══════════════════════════════════════════════════════════
# Simulate Fusion Node
# ══════════════════════════════════════════════════════════
print("\n"+"="*100)
print("模拟 FUSION 节点状态机")
print("="*100)

state='HEALTHY'
consec_healthy=0; consec_degraded=0
last_healthy_mo=None; last_healthy_ob=None
frozen_mo=None; frozen_ob=None
degraded_start_i=0
state_log=[]  # [(frame_idx, ros_t, old_state, new_state, ndt_error, note)]

for i,d in enumerate(aligned):
    me=d['me']; hc=d['hc']; ob=d['ob']; mo=d['mo']
    if math.isnan(me):
        d['fused_x']=d['rx']; d['fused_y']=d['ry']; d['state']=state
        continue

    is_degraded=(me>DEGRADED_ERR) or (not hc and me>0.1)
    is_healthy=(me<HEALTHY_ERR and hc)
    old_state=state

    if state=='HEALTHY':
        if is_healthy and mo:
            last_healthy_mo=(mo[1],mo[2],mo[3],mo[4],mo[5],mo[6],mo[7])
            last_healthy_ob=(ob[1],ob[2]) if ob else None
        if is_degraded:
            consec_degraded+=1
            if consec_degraded>=DEGRADED_CONSEC and last_healthy_mo:
                state='DEGRADED'
                frozen_mo=last_healthy_mo
                frozen_ob=last_healthy_ob
                degraded_start_i=i
                consec_healthy=0
                state_log.append((i,d['t'],old_state,state,me,
                    f"冻结map->odom=({frozen_mo[0]:.2f},{frozen_mo[1]:.2f})"))
        else: consec_degraded=0

    elif state=='DEGRADED':
        if is_healthy:
            consec_healthy+=1
            if consec_healthy>=HEALTHY_CONSEC:
                state='TRANSITIONING'; transition_start_i=i
                state_log.append((i,d['t'],old_state,state,me,
                    f"NDT恢复, 开始平滑过渡, 连续健康={consec_healthy}帧"))
        else: consec_healthy=0

        elapsed=(i-degraded_start_i)/10.0
        odom_disp=0.0
        if frozen_ob and ob:
            odom_disp=math.hypot(ob[1]-frozen_ob[0],ob[2]-frozen_ob[1])
        if elapsed>MAX_DEGRADED_SEC or odom_disp>MAX_ODOM_DISP_M:
            state='LOST'
            state_log.append((i,d['t'],old_state,state,me,
                f"兜底失败! 耗时{elapsed:.0f}s 位移{odom_disp:.1f}m"))

    elif state=='TRANSITIONING':
        if is_degraded:
            state='DEGRADED'; consec_healthy=0
            state_log.append((i,d['t'],old_state,state,me,"过渡中断, NDT又变差了"))
        else:
            elapsed=(i-transition_start_i)/10.0
            if elapsed>=TRANSITION_SEC:
                state='HEALTHY'; consec_degraded=0
                state_log.append((i,d['t'],old_state,state,me,
                    f"过渡完成, 切回HEALTHY (耗时{elapsed:.1f}s)"))

    elif state=='LOST':
        if is_healthy:
            state='TRANSITIONING'; transition_start_i=i
            state_log.append((i,d['t'],old_state,state,me,"NDT恢复, 从LOST过渡"))

    # Compute fused position
    if frozen_mo and ob and state in ('DEGRADED','TRANSITIONING'):
        fmo_x,fmo_y=frozen_mo[0],frozen_mo[1]
        fmo_yaw=quat_to_yaw(frozen_mo[3],frozen_mo[4],frozen_mo[5],frozen_mo[6])
        c,s=math.cos(fmo_yaw),math.sin(fmo_yaw)
        d['fused_x']=fmo_x+ob[1]*c-ob[2]*s
        d['fused_y']=fmo_y+ob[1]*s+ob[2]*c
    else:
        d['fused_x']=d['rx']; d['fused_y']=d['ry']
    d['state']=state

# ══════════════════════════════════════════════════════════
# Output: State Transitions
# ══════════════════════════════════════════════════════════
print(f"\n状态转换时间线 ({len(state_log)} 次转换):\n")
print(f"{'Frame':<8} {'Bag时间':<12} {'转换':<30} {'NDT err':<10} {'备注':<50}")
print("-"*115)
for fi,t,old,new,me,note in state_log:
    print(f"{fi:<8} {t:<12.1f} {old:>10} → {new:<10} {me:<10.3f} {note:<50}")

# ══════════════════════════════════════════════════════════
# State distribution
# ══════════════════════════════════════════════════════════
state_counts=defaultdict(int)
for d in aligned: state_counts[d['state']]+=1
total=len(aligned)
print(f"\n状态分布 (总共{total}帧, {total/10:.0f}s):")
for s in ['HEALTHY','DEGRADED','TRANSITIONING','LOST']:
    n=state_counts[s]; pct=n/total*100
    bar='█'*int(pct/2)
    print(f"  {s:<16} {n:>6}帧 ({pct:5.1f}%) {bar}")

# ══════════════════════════════════════════════════════════
# Key waypoint comparison
# ══════════════════════════════════════════════════════════
print("\n"+"="*100)
print("关键路点定位对比 (NDT漂移附近的路点)")
print("="*100)
print(f"{'路点':<8} {'Waypoint':<18} {'NDT-only dist':<14} {'FUSED dist':<14} {'改善':<8} {'融合状态':<16}")
print("-"*80)

key_wps=['点位8','点位9','点位10','点位11','点位12','点位13','点位14','点位15',
         '点位16','点位17','点位18','点位19','点位20','点位21','点位22','点位23','点位24']

for wp_name in key_wps:
    wp=next((w for w in wps if w['name']==wp_name),None)
    if not wp: continue
    min_ndt=float('inf'); min_fused=float('inf'); min_fused_state=''
    for d in aligned:
        dn=math.hypot(d['rx']-wp['x'],d['ry']-wp['y'])
        df=math.hypot(d['fused_x']-wp['x'],d['fused_y']-wp['y'])
        if dn<min_ndt: min_ndt=dn
        if df<min_fused: min_fused=df; min_fused_state=d['state']
    imp=min_ndt-min_fused
    icon='✓' if min_ndt<3.0 else '✗'
    f_icon='✓' if min_fused<3.0 else '✗'
    print(f"  {wp_name:<6} ({wp['x']:5.1f},{wp['y']:5.1f})  {icon} {min_ndt:<12.2f}  {f_icon} {min_fused:<12.2f}  {imp:+.2f}m  {min_fused_state:<16}")

# ══════════════════════════════════════════════════════════
# Map->odom stability during drift
# ══════════════════════════════════════════════════════════
print("\n"+"="*100)
print("Map->odom 跳变对比")
print("="*100)

# Count jumps >5m in NDT vs Fused
prev_ndt_x=prev_ndt_y=None; prev_fused_x=prev_fused_y=None
ndt_jumps=0; fused_jumps=0
for d in aligned:
    # NDT map->base
    ndt_x,ndt_y=d['rx'],d['ry']
    fused_x,fused_y=d['fused_x'],d['fused_y']
    if prev_ndt_x is not None:
        d_ndt=math.hypot(ndt_x-prev_ndt_x,ndt_y-prev_ndt_y)
        d_fused=math.hypot(fused_x-prev_fused_x,fused_y-prev_fused_y)
        if d_ndt>5: ndt_jumps+=1
        if d_fused>5: fused_jumps+=1
    prev_ndt_x,prev_ndt_y=ndt_x,ndt_y
    prev_fused_x,prev_fused_y=fused_x,fused_y

print(f"  NDT-only map->base 跳变 (>5m): {ndt_jumps} 次")
print(f"  Fused  map->base 跳变 (>5m): {fused_jumps} 次")
print(f"  改善: {ndt_jumps-fused_jumps} 次跳变被消除" if ndt_jumps>fused_jumps else f"  无改善")

# ══════════════════════════════════════════════════════════
# Show drift period trajectory
# ══════════════════════════════════════════════════════════
print("\n"+"="*100)
print("漂移期间轨迹快照 (每50帧)")
print("="*100)

drift_start=None; drift_end=None
for i,d in enumerate(aligned):
    if d['state']=='DEGRADED' and drift_start is None: drift_start=i
    if d['state']=='DEGRADED': drift_end=i

if drift_start:
    print(f"漂移区间: 帧 {drift_start}-{drift_end} "
          f"(bag时间 {aligned[drift_start]['t']:.1f}-{aligned[drift_end]['t']:.1f})")
    print(f"\n{'Frame':<8} {'State':<16} {'NDT map_base':<28} {'FUSED map_base':<28} {'NDT err':<10}")
    print("-"*95)
    for i in range(max(0,drift_start-10), min(len(aligned),drift_end+20), 50):
        d=aligned[i]
        me_str=f"{d['me']:.3f}" if not math.isnan(d['me']) else "N/A"
        marker='>>>' if d['state']!=aligned[max(0,i-1)]['state'] else '   '
        print(f"  {i:<8} {marker} {d['state']:<13} "
              f"({d['rx']:7.1f},{d['ry']:7.1f})              "
              f"({d['fused_x']:7.1f},{d['fused_y']:7.1f})              "
              f"{me_str:<10}")

# ══════════════════════════════════════════════════════════
# Final verdict
# ══════════════════════════════════════════════════════════
print("\n"+"="*100)
print("验证结论")
print("="*100)

degraded_frames=state_counts['DEGRADED']
transition_frames=state_counts['TRANSITIONING']
lost_frames=state_counts['LOST']
healthy_frames=state_counts['HEALTHY']

print(f"""
  融合节点状态分布:
    HEALTHY:       {healthy_frames} 帧 ({healthy_frames/total*100:.0f}%)
    DEGRADED:      {degraded_frames} 帧 ({degraded_frames/total*100:.0f}%)
    TRANSITIONING: {transition_frames} 帧 ({transition_frames/total*100:.0f}%)
    LOST:          {lost_frames} 帧 ({lost_frames/total*100:.0f}%)

  状态转换次数: {len(state_log)}
  NDT跳变消除:  {ndt_jumps} → {fused_jumps} ({ndt_jumps-fused_jumps}次消除)

  判定:
""")

if degraded_frames>0 and fused_jumps<ndt_jumps:
    print("  ✓ 融合节点成功: 在NDT漂移时冻结了map->odom, 减少了跳变")
elif degraded_frames>0:
    print("  ~ 融合节点部分有效: 进入了DEGRADED但跳变仍在(可能机器人没移动)")
else:
    print("  ✗ 融合节点未触发: 此bag中NDT未达到退化阈值")

if state_log:
    first_d=state_log[0]
    last_d=state_log[-1]
    print(f"  首次退化: 帧{first_d[0]} bag时间{first_d[1]:.1f} NDT error={first_d[4]:.3f}")
    print(f"  最后事件: 帧{last_d[0]} {last_d[2]}→{last_d[3]} {last_d[5]}")

print(f"\n总耗时: {time.time()-t0:.0f}s")
