#!/bin/bash
# Depot 射线可见性分变体实测（2026-08-25 排查收口）。
# 背景：单实例取证显示 Depot 模型已加载但雷达全 miss；疑点=submesh 命名/机制。
# 本脚本对每个碰撞变体：清场(复核) → 起仿真(timeout 兜底) → 探针测距离分布 → 清场。
# 变体：
#   base      原 assets/model.sdf（submesh 名 WALLS 等）
#   libsuffix submesh 名全部加 -lib 后缀（对照 DAE geometry id="WALLS-lib"）
#   wholemesh 删除 collision 内 submesh，用整个 DAE 做碰撞
# 用法：bash test_depot_raycast.sh [base|libsuffix|wholemesh|all]
set -u
GO2=$HOME/claude/raicom/go2-scan
ASSETS=$GO2/scenes/depot/model/assets
VAR_ROOT=/tmp/depot_variants
PROBE_MIN_DIST=0.5   # 中位水平距离达标线

clean() {
  bash $GO2/simulation/kill_all_sim.sh >/dev/null 2>&1
  for i in $(seq 1 15); do
    pgrep -f 'gzserver|rosmaster|roslaunch' >/dev/null || return 0
    sleep 2
  done
  echo "[test] FATAL: 清场后仍有残留"; pgrep -af 'gzserver|rosmaster|roslaunch'; exit 9
}

make_variant() {
  local v=$1
  rm -rf $VAR_ROOT/$v; mkdir -p $VAR_ROOT/$v/Depot
  cp $ASSETS/model.config $VAR_ROOT/$v/Depot/
  ln -sfn $ASSETS/meshes $VAR_ROOT/$v/Depot/meshes
  ln -sfn $ASSETS/materials $VAR_ROOT/$v/Depot/materials
  case $v in
    base)
      cp $ASSETS/model.sdf $VAR_ROOT/$v/Depot/model.sdf ;;
    libsuffix)
      /usr/bin/python3 - << 'EOF'
import re
src='/home/pumpkin-db/claude/raicom/go2-scan/scenes/depot/model/assets/model.sdf'
dst='/tmp/depot_variants/libsuffix/Depot/model.sdf'
t=open(src).read()
def fix(m):
    inner=m.group(1)
    # 只在 <name>X</name> 位于 submesh 块内时加后缀
    return '<submesh><name>%s-lib</name></submesh>' % inner.strip() if False else m.group(0)
t2=re.sub(r'<submesh>\s*<name>([^<]+)</name>', lambda m: '<submesh>\n        <name>%s-lib</name>' % m.group(1), t)
open(dst,'w').write(t2)
print('libsuffix 变体生成')
EOF
      ;;
    wholemesh)
      /usr/bin/python3 $GO2/scenes/gtools/make_wholemesh_collision.py \
          $ASSETS/model.sdf $VAR_ROOT/$v/Depot/model.sdf ;;
  esac
}

probe() {
  # 等雷达出帧，输出一行结论 JSON
  source /opt/ros/noetic/setup.bash
  timeout 90 /usr/bin/python3 - << 'EOF'
import rospy, numpy as np, sys
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
rospy.init_node('variant_probe', anonymous=True)
cap={'c':[],'p':None}
rospy.Subscriber('/mid360_points', PointCloud2, lambda m: cap['c'].append(m), queue_size=2)
rospy.Subscriber('/quad_0/lidar_pose', Odometry, lambda m: cap.__setitem__('p',m), queue_size=2)
t0=rospy.get_rostime()
while (rospy.get_rostime()-t0).to_sec()<60 and len(cap['c'])<12:
    rospy.sleep(0.3)
if not cap['c'] or not cap['p']:
    print('RESULT {"frames":0}'); sys.exit(0)
msg=cap['c'][-1]
dt=np.dtype({'names':[f.name for f in msg.fields],'formats':[np.float32]*len(msg.fields),'offsets':[f.offset for f in msg.fields],'itemsize':msg.point_step})
arr=np.frombuffer(bytes(msg.data),dtype=dt)
xy=np.stack([arr['x'],arr['y']],axis=-1).astype(np.float64)
sp=np.array([cap['p'].pose.pose.position.x,cap['p'].pose.pose.position.y])
d=np.linalg.norm(xy-sp,axis=1)
uniq=len(np.unique(xy.round(2),axis=0))
print('RESULT {"frames":%d,"median":%.3f,"max":%.3f,"unique":%d}' % (len(cap['c']), float(np.median(d)), float(d.max()), uniq))
EOF
}

run_variant() {
  local v=$1
  echo "===== 变体 [$v] ====="
  clean
  make_variant $v
  # 关键：env.sh 会把原 model/ 路径插到变体路径前面，必须临时摘掉原 Depot 链接，
  # 否则 gazebo 永远先解析到原模型，变体无效
  mv $GO2/scenes/depot/model/Depot $GO2/scenes/depot/model/Depot.bak_test 2>/dev/null
  cd $GO2
  timeout 240 env GAZEBO_MODEL_PATH=$VAR_ROOT/$v:${GAZEBO_MODEL_PATH:-} \
      bash simulation/launch_gazebo_sim.sh global_planner:=ariadne scene:=depot gui:=false rviz:=false \
      > /tmp/var_$v.log 2>&1 &
  local launcher=$!
  # 等探针结果（probe 自带 60s 等帧），最多等 200s
  local out=$(probe)
  echo "$out" | grep -o 'RESULT .*'
  kill $launcher 2>/dev/null
  clean
  mv $GO2/scenes/depot/model/Depot.bak_test $GO2/scenes/depot/model/Depot 2>/dev/null
}

V=${1:-all}
if [ "$V" = "all" ]; then
  run_variant base
  run_variant libsuffix
  run_variant wholemesh
else
  run_variant $V
fi
echo "===== 全部完成 ====="
