#!/bin/bash
# B 验收一键脚本：terrain_follow 遥控上台阶（方向④阶段B）+ stair_detector 几何检出（阶段C）
# 流程：清场 → none 底座+remote_drive 起 Depot 仿真 → spawn 健康门 → stair_detector +
#       双记录器 → 五段遥控绕行推进（出生→南通道→东隔间→楼梯 entry→二层 exit）→ 摘要
# 航点依据：GT 直查三道隔断墙（x≈1.4-2.7/5.3-6.5/9.1-10.3，y 全宽）仅南通道 y∈[-7.5,-1.5]
#       可过（2026-08-26 逐段验证）。2026-08-27 注册表方向仲裁后改道：接地端在
#       y=3.2 侧，爬升沿 -y，故段3/4 改走楼梯北侧（y=3.5）进场。
# 产物目录：/tmp/stair_b/{sim.log,pos.log,stairs.log,stair_det.log}
# 用法：bash simulation/test_stair_climb.sh
set -u

GO2=$HOME/claude/raicom/go2-scan
PY=/usr/bin/python3
D=/tmp/stair_b
mkdir -p "$D"

echo "[1/6] 清场..."
bash "$GO2/simulation/kill_all_sim.sh" >/dev/null 2>&1 || true
sleep 3
LEFT=$(ps -eo comm | grep -cwE "gzserver|gzclient|terrainAnalysis|terrainAnalysisExt|scan_planner_no" || true)
if [ "${LEFT:-0}" -gt 0 ]; then
  echo "FATAL: 残留 $LEFT 个进程，kill_all_sim.sh 杀不干净（已知问题），请手动补杀后重跑："
  ps -eo pid,comm | grep -E "gzserver|terrainAnal|scan_planner"
  exit 42
fi

echo "[2/6] 起 none 底座仿真（remote_drive，无头）..."
nohup bash "$GO2/simulation/launch_gazebo_sim.sh" global_planner:=none scene:=depot gui:=false rviz:=false \
      remote_drive:=true > "$D/sim.log" 2>&1 &
sleep 45

echo "[3/6] spawn + 点云健康门..."
source /opt/ros/noetic/setup.bash
source "$GO2/algorithms/local_planning/scan_planner/devel/setup.bash"
# shellcheck disable=SC1091
source "$GO2/scenes/depot/env.sh"
CMU=$GO2/simulation/cmu_env
export ROS_PACKAGE_PATH=$CMU/src/velodyne_simulator:$CMU/src:$ROS_PACKAGE_PATH
export ROS_PACKAGE_PATH=$GO2/integration:$ROS_PACKAGE_PATH   # go2_bridge（stair_detector 需要）
$PY "$GO2/simulation/spawn_go2.py" --x "$SPAWN_X" --y "$SPAWN_Y" --z "$SPAWN_Z" --yaw "${SPAWN_YAW:-0}"
RC=$?
if [ $RC -ne 0 ]; then echo "FATAL: spawn/健康门失败 rc=$RC"; exit 43; fi

echo "[4/6] 起 stair_detector（几何检测分支）..."
# 必须 /usr/bin/python3 全路径直起：rosrun 走 shebang env python3 落 miniconda 无 numpy
# （续9/续11 教训两次复现）。_registry:= 私有参数 rospy 命令行重映射仍生效。
$PY "$GO2/integration/go2_bridge/scripts/stair_detector.py" \
    _registry:="$GO2/scenes/depot/scene.yaml" > "$D/stair_det.log" 2>&1 &
sleep 3

echo "[5/6] 记录器启动（PYTHONUNBUFFERED 防 timeout 杀进程丢缓冲，520s 窗口）..."
# rostopic echo 是 Python 进程，stdbuf 只管 libc 层 → 必须 PYTHONUNBUFFERED=1
PYTHONUNBUFFERED=1 timeout 520 rostopic echo /quad_0/body_pose/pose/pose/position > "$D/pos.log" 2>&1 &
PYTHONUNBUFFERED=1 timeout 520 rostopic echo /stairs_detected > "$D/stairs.log" 2>&1 &
sleep 2

echo "[6/6] 五段遥控绕行推进..."
# 航点已按 2026-08-27 注册表方向仲裁修正：接地端在 y=3.2（非 0.4），爬升沿 -y。
# 路线：南通道→东隔间北上至 y=3.5（避开楼梯结构带）→ 楼梯接地端前 → 沿 -y 爬升。
$PY "$GO2/tools/drive_go2.py" --target -11.0 -5.5 --max-v 0.40 --tol 0.45 --timeout 150
echo "--- 段1(南下入通道) rc=$? ---"
$PY "$GO2/tools/drive_go2.py" --target 10.5 -5.5 --max-v 0.40 --tol 0.50 --timeout 200
echo "--- 段2(南通道东行 21.5m) rc=$? ---"
$PY "$GO2/tools/drive_go2.py" --target 11.2 3.50 --max-v 0.30 --tol 0.45 --timeout 150
echo "--- 段3(北上东隔间) rc=$? ---"
$PY "$GO2/tools/drive_go2.py" --target 12.85 3.55 --max-v 0.20 --tol 0.35 --timeout 120
echo "--- 段4(entry 接地端前) rc=$? ---"
$PY "$GO2/tools/drive_go2.py" --target 12.87 0.15 --max-v 0.13 --tol 0.60 --timeout 300
echo "--- 段5(沿 -y 爬梯至二层) rc=$? ---"

sleep 5
echo ""
echo "================ B 验收结果摘要 ================ "
$PY - <<'EOF'
import re

# rostopic echo 输出是单行「x: 7.518」；截断的科学计数法（如 1.23e）float 会抛
# ValueError，跳过即可
xs, ys, zs = [], [], []
pat = re.compile(r'^(x|y|z):\s*(-?[\d.eE+]+)\s*$')
cur = {}
for line in open('/tmp/stair_b/pos.log', errors='ignore'):
    m = pat.match(line.strip())
    if not m:
        continue
    try:
        cur[m.group(1)] = float(m.group(2))
    except ValueError:
        continue
    if len(cur) == 3:
        xs.append(cur['x']); ys.append(cur['y']); zs.append(cur['z']); cur = {}
if zs:
    print('采样 %d 帧 | z: min=%.3f max=%.3f 终值=%.3f' % (len(zs), min(zs), max(zs), zs[-1]))
    print('终点 (x=%.2f, y=%.2f)  出发点 (x=%.2f, y=%.2f)' % (xs[-1], ys[-1], xs[0], ys[0]))
    hi = [z for z in zs if z > 1.0]
    print('z>1.0m 样本: %d 帧%s' % (len(hi), ('，最高 %.2fm —— 地形跟随生效 ✅' % max(hi)) if hi else ' —— 未观察到爬升 ❌'))
else:
    print('pos.log 无数据 ❌')

txt = open('/tmp/stair_b/stairs.log', errors='ignore').read()
if 'geometry' in txt:
    print('/stairs_detected: 含 geometry 几何检出 ✅')
elif 'registry' in txt:
    print('/stairs_detected: 仅 registry 注册表兜底（几何未检出）⚠️')
else:
    print('/stairs_detected: 无输出 ❌')
EOF
echo "日志: $D/"
