# EE3033 中文实验流程

建图 / 复用地图 → 规划选点 → 单目标 → 有界搜索

用途与边界
----------
这是供团队阅读、现场参考的流程文档，不是脚本或一键启动程序。
下列命令由现场操作员逐段执行；标明「运动」的命令需现场明确授权。
主体导航流程不包含 YOLO 闭环、目标位置报告、自动返航、balance_gui 或点击导航目标。
2026-11-04 补充了独立、无运动的 YOLO 角度输出验收，见下方记录；
它不是自动搜索遇目标后停止、定位、接近目标的完整闭环。
不要把 /mission/start_mapping 当成真实建图启动：它只切换任务管理器状态。

证据基线：2026-11-03，ROS 2 Humble。
FAGIO 分支 robot-validation-hardening，提交
e17ec86c75e07508dc3b4fb836e0461cf6ef3422。
当天报告的小车提交为 74ecd91175236eba555e669b52f2eb07f083c6a2；
两者差异仅 README.md。下次仍需核对实际部署，不能假定自动同步。

当天较早场地的历史输出（不是以下新场地测试结果）：
  - 构建、pytest：操作员报告通过，详细测试数量未提供。
  - 保存地图基础栈、定位、激光和动态 TF：已检查。
  - dry-run：96 个候选，候选 40，tiny_mlp，路径 0.73 m。
  - 单目标：SINGLE_GOAL_TEST_COMPLETE，覆盖率 0.16110581159591675。
  - 操作员补充报告手动验证过运动中取消，但本会话未收齐对应 UUID/终态日志。
  - 已保存覆盖并准备 20 次上限；本会话没有 20 次运行完成的结果。
上述记录不是下次实验的自动验收，不把「准备完成」写成「运行通过」。

2026-11-03 晚间新小场地测试（时间为 UTC+08:00；Pi SHA 仍为上述 74ecd91）：
  - 建图：课程 SLAM 整合入口 + 操作员自己的操纵方式。首次地图有重影，
    未保存，现场重启整套建图栈后重新建图；慢右转平衡漂移未修复。
  - 地图：map_test_1103.pgm/.yaml 保存成功，44 x 61 cells、0.05 m/cell，
    origin=[-0.64,-2.58,0]；范围矩形约 2.20 x 3.05 m，不等于可通行面积。
  - Pi 构建：1 package finished [2.25s]；pytest：96 passed in 0.53s。
  - 保存地图栈：map_server/amcl/planner_server/bt_navigator 均 active，
    两个 Action 有服务器，无 continuous_waypoint_loop 输出。
  - 激光：SLAM 用 /scan_fixed；保存地图栈实测用 /scan，LiDAR 发布者，
    AMCL 和两个 costmap 订阅，QoS 兼容；五次试跑前延迟 1-2 ms。
  - Nav2：local/global footprint=[]、robot_radius=0.12 m、padding 约 0.01 m；
    操作员确认现场空间。动态 base TF 最大净延迟实测约 15.4 ms。
  - 单目标：候选 35，tiny_mlp，path=0.24 m；操作员回传 F 状态 4，
    SINGLE_GOAL_TEST_COMPLETE；最终目标朝向 -90 度，朝墙观察效果暂未处理。
  - 运动中取消：本次 UUID 06a446c9-6d1a-4939-8822-a15dc4aa5936，
    Nav2 状态 2 -> 3 -> 5；随后 OPERATOR_STOPPED、FINISHED，
    操作员确认已停且未继续运动。取消请求与终态分别有实际输出。
  - FAGIO 多次卡死后不再运行 RViz；Pi 栈继续存在，旧 search_manager
    经 PID 核实后用 SIGTERM 退出。新节点未确认加载覆盖快照。
  - 新起点 dry-run：候选 34，tiny_mlp，path=0.64 m，
    DRY_RUN_SELECTION_READY；TF 约 (1.05-1.08,-0.12 至 -0.14) m、149 度，
    操作员确认现场定位及空间。
  - 五次试跑：明确授权 True/True/5/False，从零覆盖开始；
    操作员报告完成五次选点和导航，终态 SINGLE_GOAL_TEST_COMPLETE，
    /search/coverage_fraction=0.6899682283401489，即模型覆盖率 69.00%。
  - 尚未收齐五次逐目标 UUID/路径/终态账本、最后目标终态与现场停止确认；
    最终 save_memory 响应、持久归档、模型 checksum/推理时间：未回传或未测量。
  - 本轮为 physical robot motion（有界导航），没有相机/YOLO 观察验收，
    没有目标定位报告；不能称为目标检测闭环或 69% 实际图像覆盖。


## 2026-11-04 补充：静止实机 YOLO 角度输出已验收

类别：**non-actuating robot integration（静止、无运动实机整合）**。
本轮未发送导航目标；操作员在 2026-11-04 14:19（UTC+08:00）确认验收通过。
Pi 已回传的代码基线为 `robot-validation-hardening`、
`9c4a641f50a6ae9d7a316b846371999b7993b2f6`。
完整接口、证据边界、启动命令和驱动回退方式见
[YOLO adapter 交接记录](YOLO_FORMAL_ADAPTER_HANDOFF.md#2026-11-04-stationary-pi-acceptance)。

本轮复用了现有 Pi 本地 YOLOv8n NCNN，不重新部署模型，不改检测脚本：

```text
/camera/image_raw + /camera_info（已核对原始图像标定）
  -> 现有 yolo_ncnn -> /yolo/detections（String JSON）
  -> yolo_detection_adapter（精确匹配原图时间戳、类别过滤、多帧确认）
  -> /target/observation（原始 header、episode、bbox、水平 bearing_rad）
```

- 实测发现 usb_cam 0.8.1 的 epoch/monotonic 换算将 `tv_usec` 错除以 1000。
  经批准，在独立工作区基于官方 0.8.1 应用官方单行修复并构建；
  未覆盖 `/opt/ros` 或课程核心包。运行相机时需显式 source 该 overlay，
  不是仓库更新后自动生效。
- 修复前 IMAGE 年龄约 832 ms；修复后最后一次 6 秒采样保留的最近
  60 条 IMAGE 年龄为 min/median/max = 37.93/39.04/40.62 ms，
  JSON 为 58.36/59.11/79.01 ms，低于原有 500 ms 门限。
  这是消息年龄，不是曝光到结果的完整延迟，也不是长期延迟上界。
- 右侧 bbox 中心 `(432.0,176.5)`：`+0.2412075729 rad`，约 `+13.82°`；
  左侧 `(151.25,192.75)`：`-0.2300178216 rad`，约 `-13.18°`。
  两条样本均为 `bottle`、class 39、640x480、`confirmed=true`。
- 操作员另行确认新 episode 配置和重新确认已通过。
  会话贴出的左右样本本身是同一 episode；新 UUID/服务响应未归档，
  因此不把这两条样本当作独立 episode 的直接证据。
- 角度基于去畸变的 bbox 中心，光学 +z 为零、画面右侧为正；
  主点 x 约 288.46，不是图像中心 320。不是底盘或地图 yaw。

测试通过范围：现有 NCNN -> JSON -> 正式 observation、短时新鲜度、
左右符号，以及操作员确认的配置/多帧流程。
未测：独立物理角度真值误差、近零角检查、完整视场有效性、光学外参、
同一物体持续关联、自动停止搜索、多视角地图定位、定位后安全接近。
`tracking_id` 为空，不能宣称已具备可靠的多目标实例关联。

测试只使用 `/bearing_test/mission_state`，不调用 `/mission/start_search`。
当前 adapter 不发布 `/target_found`，不发送导航目标，也不发布 `/cmd_vel`。
后续先审计实际 TF/定位接口并设计可验证的停止握手；
主动移动定位和导航接近必须另行完成安全检查、测试并获得现场授权。

## 一、机器、终端和路径

| 环境 | 工作区 | 仓库 |
| --- | --- | --- |
| FAGIO 本地 | — | `/home/acyg/Map_Search/ee3033-search-policy` |
| Pi 宿主机 | `/home/balance/ros2_ws` | `/home/balance/ros2_ws/src/ee3033-search-policy` |
| Pi ROS Docker | `/home/ros2_ws` | `/home/ros2_ws/src/ee3033-search-policy` |

已知容器名 ros2_humble；宿主机工作区绑定挂载到 Docker。
FAGIO 的代码或地图修改不会自动进入小车。

每个新的 Pi ROS 终端，先在 **Pi 宿主机**执行：

```bash
ros2-start
```

**进入 Pi ROS Docker 后**：

```bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
```

建议固定终端角色：
  A：SLAM 或保存地图基础栈；两种模式不要同时运行。
  B：mission_manager。
  C：search_manager。
  D：控制服务；始终可输入取消命令。
  E：/search/status 监控。
  F：Nav2 UUID / 状态监控。
  G：日志记录。
下列 Pi Docker 命令都先完整 source，不依赖另一终端的环境。

本次后半程不运行 FAGIO RViz（操作员报告运行期间电脑多次卡死，原因未诊断）。
控制、状态、Nav2 UUID 监控都在 Pi Docker，终端窗口消失不表示 Pi 进程退出。
不因电脑卡死重建地图或重复启动底盘。机器人被移动、底盘重启或定位异常时，
需要重新核对真实 TF、传感器和现场位置；规划成功不代替定位确认。
无 RViz 也不能跳过此关口，无法确认时保持导航关闭。

FAGIO RViz（仅保留为可选参考，不是本次五次试跑必须执行的命令）：
  先确认它实际在宿主机还是 Docker，复用已确认能看到 ROS 图的环境。
  仅当 FAGIO 宿主机已确认安装 Humble，且 DDS 环境已与小车核对时：

```bash
    source /opt/ros/humble/setup.bash
    printenv | grep -E '^(ROS_DISTRO|ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_LOCALHOST_ONLY)='
    rviz2
```

  若使用 FAGIO Docker，须先核实该容器入口和 overlay 的实际路径；不要套用
  Pi 的 /home/ros2_ws/install/setup.bash，也不要自动修改 DDS。
  RViz Fixed Frame 使用现场已验证的 map；显示地图、激光、TF，
  可添加 /search/selected_goal 的 Pose 显示。不要用 RViz 点击派发目标。


## 二、实验开始只做一次的准备关口

目的：明确部署、是否已有节点、地图选择和现场断电方法。
除非发生重启、掉线、节点退出、地图变化或定位异常，不重复已通过的检查。

[Pi 宿主机；只读，不需要 ROS source]

```bash
  cd /home/balance/ros2_ws/src/ee3033-search-policy
  git status --short --branch
  git rev-parse HEAD
```

保护未跟踪地图和预期删除，不自动恢复、pull、切分支或安装软件。
代码变更时，先在 FAGIO 验证；经批准在 Pi 同步明确的提交或指定文件，
再核实 Pi SHA / diff 并构建。不可假定 FAGIO 编辑已经部署。
WORKFLOW.md 当前只写入 FAGIO，亦遵循这一规则。

[Pi ROS Docker；现有检查终端 D]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  printenv | grep -E '^(ROS_DISTRO|ROS_DOMAIN_ID|RMW_IMPLEMENTATION|ROS_LOCALHOST_ONLY)='
  ros2 node list
```

当天已观察 ROS_DOMAIN_ID=15、ROS_LOCALHOST_ONLY=0；下次核实而非自动 export。
如果需要重新构建，仅在代码同步 / 构建产物变化后，经批准执行一次：

```bash
  source /opt/ros/humble/setup.bash
  cd /home/ros2_ws
  colcon build --packages-select ee3033_search_policy --symlink-install
  source /home/ros2_ws/install/setup.bash
  python3 -m pytest /home/ros2_ws/src/ee3033-search-policy/test -q
```

依赖缺失就报告，不自动安装；保留真实输出，不编造通过数量。


## 三、地图分支：首次建图，或直接复用

### 3A. 已有可信地图

跳过本节建图和保存，直接进入第四节。

本次新小场地地图：

```text
  Pi 宿主机：
    /home/balance/ros2_ws/src/ee3033-search-policy/maps/map_test_1103.yaml
    /home/balance/ros2_ws/src/ee3033-search-policy/maps/map_test_1103.pgm
  Pi Docker：
    /home/ros2_ws/src/ee3033-search-policy/maps/map_test_1103.yaml
    /home/ros2_ws/src/ee3033-search-policy/maps/map_test_1103.pgm
  PGM SHA256：
    3010472a448eb5d7c281bc6e9954ae57f029e9e23dcebedd79f5613550eb97c6
  YAML：image=map_test_1103.pgm，resolution=0.05，
    origin=[-0.64,-2.58,0]，mode=trinary，negate=0，
    occupied_thresh=0.65，free_thresh=0.25。
```

新地图未同步到 FAGIO；不能在那里验证或重新生成小车这张地图。
下次只在地图选择或文件变化时检查文件内容、引用关系和 checksum。

### 3B. 首次建图

2026-11-03 已依据课程 Lab4_report.pdf 第 5-6、10-11 页及操作员回传的
Pi install 下 launch 内容核实以下流程；操作员已确认采用自己的操纵方式，
不使用键盘遥控。本记录是流程确认，不是本次新场地建图已通过的证据。
当前代码也没有实现自主探索和建图完成的硬件通知接口。

机器人先放到新场地起点，确认没有重复运行的底盘、LiDAR 或 SLAM。
经现场明确授权启动真实底盘后，使用以下整合入口。

[Pi ROS Docker；建图终端 A，启用真实底盘，不是只读检查]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 launch balance_bringup balance_slam_bringup.launch.py
```

该入口依次启动 balance_robot_base、N10P LiDAR、包内 scan_resampler.py
和异步 SLAM Toolbox；重采样配置为 /scan -> /scan_fixed、450 bins。
不要再分别启动同一组组件；它的顶层不启动 Nav2 或 explore_lite。
不要用 /mission/start_mapping 代替真实 SLAM。
另一个 balance_slam_nav_explore_bringup.launch.py 会按定时器无条件启动
Nav2 和 explore_lite，没有关闭探索的 launch 参数，不能用作本次人工建图入口，
也不能假定 navigation_enabled:=false 能阻止该教师入口自动运动。

[FAGIO 已确认 ROS 图可见的宿主机或 Docker；独立可视化终端]

```bash
  rviz2
```

按第一节核对环境；Map、LaserScan、TF、Odometry 使用现场实测 Topic/帧名。
课程中的 base_link、n10_lidar_link 等名称是示例，不替代小车实际配置与 TF。
先确认地图更新、激光与墙体对齐，再由现场操作员使用自己的既有操纵方式移动。
本文件不猜测该操纵接口，不提供键盘遥控或 /cmd_vel 命令。
平稳、低速覆盖完整小场地，保持安全间距，避免急转和碰撞；
最后回到起点附近帮助闭环。未实测的速度数值不写作已验收参数。

机器人停止移动后，保持 SLAM 运行，先保存并检查地图，最后才关闭建图进程。
地图应墙体连续、少重影、障碍位置合理，起终点附近一致；
RViz 确认地图覆盖和激光对齐。新场地不复用旧地图或旧覆盖快照。

[Pi ROS Docker；检查终端 D，先确认保存工具的实际参数]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 run nav2_map_server map_saver_cli --help
```

以下是 Humble map_saver_cli 标准保存方式，须以小车 help 和实际 /map 为准，
不是本会话已执行的保存结果；新前缀不得覆盖已有文件：

```bash
  MAP_PREFIX=/home/ros2_ws/src/ee3033-search-policy/maps/map_test_1103
  if [ ! -d /home/ros2_ws/src/ee3033-search-policy/maps ]; then
    printf 'STOP: map directory does not exist\n'
  elif [ -e "${MAP_PREFIX}.yaml" ] || [ -e "${MAP_PREFIX}.pgm" ]; then
    printf 'STOP: choose a new map filename\n'
  else
    ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX" -t /map --fmt pgm
  fi
```

本次预期生成 map_test_1103.yaml 和 map_test_1103.pgm，位于上述 Pi Docker
仓库 maps 目录，不保存到课程原示例的 balance_mapping/maps。
同名前缀已存在或保存失败就停止，不检查旧文件并把它们当作本次保存成功。
保存成功后，在同一检查终端执行：

```bash
  if test -s "${MAP_PREFIX}.yaml" && test -s "${MAP_PREFIX}.pgm"; then
    cat "${MAP_PREFIX}.yaml" &&
    sha256sum "${MAP_PREFIX}.pgm"
  else
    printf 'STOP: map files are missing or empty\n'
  fi
```

检查命令任一失败就停止；检查 YAML 的 image 是否指向匹配的真实文件。
本次 YAML 末尾没有换行，cat 输出的 free_thresh: 0.25 紧接 SHA256，
不表示 free_thresh 值被改变。
通过保存与地图检查、确定机器人停止后，只关闭自己启动的建图进程，
再进入保存地图模式。关闭终端或 Ctrl+C 本身不是物理停止证明。
不要同时启动两个底盘或两个地图提供者。


## 四、启动保存地图基础栈

节点已正常运行就跳过启动，不重复 launch。只在确实重启后重验定位/实时性。
没有地图切换就不重新建图或反复检查 checksum。

[Pi ROS Docker；基础栈终端 A]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 launch balance_bringup balance_known_map_waypoint_bringup.launch.py \
    map:=/home/ros2_ws/src/ee3033-search-policy/maps/map_test_1103.yaml \
    start_waypoints:=false
```

使用新地图时只替换 map 参数为新 YAML 的真实 Docker 路径。
此启动底盘/传感器/定位/Nav2，不是纯软件模拟，仍需现场监护。
已知该 launch 无条件打印 Starting continuous waypoint loop；
不能仅凭这条提示判断航点循环在跑。Nav2 waypoint_follower 也不是循环派发器。

[Pi ROS Docker；检查终端 D，启动后的合并关口]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  for node in map_server amcl planner_server bt_navigator; do
    printf '\n===== %s =====\n' "$node"
    ros2 lifecycle get "/$node"
  done
  ros2 node list | grep continuous_waypoint_loop
  ros2 action info /compute_path_to_pose
  ros2 action info /navigate_to_pose
  timeout 6s ros2 run tf2_ros tf2_echo map base_footprint
  timeout 12s ros2 topic delay /scan
  timeout 15s ros2 run tf2_ros tf2_monitor odom_combined base_footprint
```

验收：四节点 active；无航点循环；两 Action 有服务器；真实 TF 可查询；
位置、朝向、激光与现场吻合；机器人静止。初次切换时本轮通过 RViz 确认；
后半程不再运行 RViz，移动后由实际 TF 与现场/已知地图核对，无法核对就不运动。
当天暂定激光、base_footprint 动态 TF 最大延迟 <=0.5 秒，仅是暂定门限。
不要把静态 TF 或 All Broadcasters 的累计时间当作动态链路延迟。
tf2 工具启动初期等待后能获取变换不自动判失败；timeout 结束信号是预期。
不能仅凭两分钟前的静止 AMCL 保留消息断定定位不实时，也不能凭有 TF
就断定定位准确。base_footprint / odom_combined 是当天实测名称，不是通用常量。
候选 clearance=0.15 m 已由操作员确认为当前机器人安全阈值；它仍不替代
Nav2 footprint、默认 inflation 和碰撞检查。
本次保存地图栈没有 /scan_fixed，用 /scan。若某 Topic 最初提示未发布，
应看随后是否收到数据，不能忽略一直没有样本的情况，也不盲目启动重采样。


## 五、策略启动：默认无运动，先看当前位姿的选点

[Pi ROS Docker；任务终端 B]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 run ee3033_search_policy mission_manager
```

[Pi ROS Docker；策略终端 C]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 run ee3033_search_policy search_manager --ros-args \
    --params-file /home/ros2_ws/src/ee3033-search-policy/config/search_policy.yaml \
    -p navigation_enabled:=false \
    -p single_goal_mode:=true \
    -p max_navigation_goals:=1 \
    -p return_home_enabled:=false
```

READY 只代表候选就绪，不等于 /mission/state 为 IDLE。
新建 search_manager 不自动加载 /tmp 的覆盖文件。当天选择从零开始。
复用覆盖的方法见第八节；若采用保存覆盖，不能在加载之后 mission/reset。

[Pi ROS Docker；控制终端 D]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  for p in navigation_enabled single_goal_mode max_navigation_goals return_home_enabled; do
    printf '\n===== %s =====\n' "$p"
    ros2 param get /search_manager "$p"
  done
  timeout 10s ros2 topic echo /mission/state \
    --qos-reliability reliable --qos-durability transient_local --once
```

确认 False / True / 1 / False，任务 IDLE 后：

```bash
  ros2 service call /mission/start_mapping std_srvs/srv/Trigger '{}'
  ros2 service call /mission/finish_mapping std_srvs/srv/Trigger '{}'
  ros2 service call /mission/start_search std_srvs/srv/Trigger '{}'
```

服务逐条执行，每条必须 success=True，否则不继续。
前两条只推进 IDLE -> MAPPING -> WAIT_FOR_TARGET，不重新建图。
最后一条推进 SEARCHING；此时 navigation_enabled=false，所以仅规划选点。
导航启用之后同一个 start_search 会真的移动，不要混淆。

查看状态和目标：

```bash
  timeout 30s ros2 topic echo /search/status \
    --qos-reliability reliable --qos-durability transient_local
  timeout 10s ros2 topic echo /search/selected_goal \
    --qos-reliability reliable --qos-durability transient_local --once
```

验收：DRY_RUN_SELECTION_READY；日志 Selected ... by tiny_mlp 或明确 fallback。
查看新目标的位置、朝向和路径，现场确认通道安全；不能沿用另一定位下的目标。
规划不是运动到达，也不会更新覆盖。
无需另跑 YOLO、相机、target_checker 或 target_localizer。


## 六、一次性设置监控和日志，之后复用

终端丢失时只恢复缺少的监控。若策略终端丢失但节点仍在，先确认旧目标终态、
OPERATOR_STOPPED 和现场停止，再识别实际进程；不要启动第二个 search_manager。

[Pi ROS Docker；检查终端 D，只读识别]

```bash
  ps -eo pid,ppid,args | grep '[s]earch_manager'
  timeout 5s ros2 topic echo /search/status \
    --qos-reliability reliable --qos-durability transient_local --once
```

需要保留覆盖时先按第八节保存并检查响应。现场批准后只对刚核实的实际节点
PID 使用 kill -TERM；ros2 run 包装进程不是实际节点。不复用历史 PID，
不用 pkill/killall。确认进程和节点均退出后才开新的 C；不杀底盘或 Nav2。
本轮曾核实 PID 1682 并确认退出，仅是历史记录，不是下次可直接使用的 PID。
如果退出发生在活动导航中，新 search_manager 不能被假定能接管旧目标，
重启不能作为停止证明。

[Pi ROS Docker；状态终端 E]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 topic echo /search/status \
    --qos-reliability reliable --qos-durability transient_local
```

[Pi ROS Docker；Nav2 状态终端 F]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 topic echo /navigate_to_pose/_action/status action_msgs/msg/GoalStatusArray \
    --qos-reliability reliable --qos-durability transient_local
```

[Pi ROS Docker；日志终端 G]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  LOG_FILE="/tmp/ee3033_search_$(date +%Y%m%dT%H%M%S).log"
  printf 'Log: %s\n' "$LOG_FILE"
  ros2 topic echo /rosout rcl_interfaces/msg/Log | tee -a "$LOG_FILE"
```

新终端先按第一节在 Pi 宿主机执行 ros2-start。
日志记录 Selected / Candidate ... failed / 状态错误；Nav2 按新 UUID 区分每次
动作。启动时历史 UUID 不算新尝试。取消锁存、服务响应和动作终态分别记录。
严格逐目标账本仍需后续工具，尤其拒绝的目标可能不出现在服务器状态列表中。
实验结束把 /tmp 日志、快照另存到双方约定的持久位置；/tmp 不是长期归档。


## 七、单目标运动及取消

必要前提：现场操作员明确授权，目标/路径安全，断电方法就绪。
首次到新场地或部署变化后，不直接跳到连续运动；先验单目标及停止行为。

从零覆盖的 dry-run 退出：
[Pi ROS Docker；控制终端 D]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 service call /mission/reset std_srvs/srv/Trigger '{}'
  timeout 10s ros2 topic echo /mission/state \
    --qos-reliability reliable --qos-durability transient_local --once
```

确认 IDLE 且没有活动导航。reset 会清零覆盖、计数和锁存；
如果有要保留的真实覆盖，不走这个 reset 分支，改用第八节保存/FINISHED/恢复。

[Pi ROS Docker；原策略终端 C]
  只对 search_manager 按 Ctrl+C，等 shell 返回；其他终端保持运行。

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 run ee3033_search_policy search_manager --ros-args \
    --params-file /home/ros2_ws/src/ee3033-search-policy/config/search_policy.yaml \
    -p navigation_enabled:=true \
    -p single_goal_mode:=true \
    -p max_navigation_goals:=1 \
    -p return_home_enabled:=false
```

合并确认 True / True / 1 / False，任务 IDLE，监控和取消终端准备好。
这些参数初始化时读入，不用运行中 ros2 param set 代替重启。

[Pi ROS Docker；控制终端 D，逐条服务成功后再继续]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 service call /mission/start_mapping std_srvs/srv/Trigger '{}'
  ros2 service call /mission/finish_mapping std_srvs/srv/Trigger '{}'
  ros2 service call /mission/start_search std_srvs/srv/Trigger '{}'
```

最后一条是【真实运动】，只执行一次。

运动中取消命令：

```bash
  ros2 service call /search/cancel_navigation std_srvs/srv/Trigger '{}'
```

通过必须同时满足：
  本次新 UUID 状态 5 CANCELED；
  /search/status 为 OPERATOR_STOPPED；
  现场确认停止；停止后不再派发目标。
状态 2 是 EXECUTING，4 是 SUCCEEDED，5 是 CANCELED，6 是 ABORTED。
No active navigation goal、服务 success、请求超时、OPERATOR_STOP_UNVERIFIED
都不能单独作为运动中取消通过证据；已经到达不算取消通过。
本实现 OPERATOR_STOPPED 也可能来自 SUCCEEDED 或无活动目标，因此必须核对 UUID。
失败或到达过快时不要自动补发；先明确下一次单目标授权。
任何异常运动使用物理断电开关并扶稳，不依靠终端 Ctrl+C 证明机器人已停。

正常到达测试：SUCCEEDED -> OBSERVING -> SINGLE_GOAL_TEST_COMPLETE。
观察时间默认为 0.25 秒，随后才更新覆盖；不是发送目标就增加覆盖。
当前节点依赖 Nav2 成功并等待观察时间，未单独验证真实 heading 达标；
时间到也不等于相机确实完成观察。今天无相机测试，覆盖是视角模型的记忆更新。


## 八、准备最多五次新的导航尝试

只在新场地单目标到达、运动中取消与现场停止验收通过后进入本节。
基础栈、mission_manager 和监控保持运行；不重建地图，不运行 FAGIO RViz。
下面两条路线二选一，不把保存覆盖与清零覆盖混用。

### 8A. 本轮实测路线：从零覆盖，先规划再启用五次预算

确认没有活动目标，上一任务终结且现场停止；操作员明确选择新任务清零。
本轮未回传旧覆盖保存/加载成功证据，不声称五次试跑保留了此前单目标覆盖。

[Pi ROS Docker；终端 D]

```bash
  ros2 service call /mission/reset std_srvs/srv/Trigger '{}'
```

必须 success=True；reset 清零覆盖、计数与锁存，不删除地图。
只停止已终结的 search_manager（终端丢失见第六节），等 shell 返回。

[Pi ROS Docker；终端 C，保持运行]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 run ee3033_search_policy search_manager --ros-args \
    --params-file /home/ros2_ws/src/ee3033-search-policy/config/search_policy.yaml \
    -p navigation_enabled:=false \
    -p single_goal_mode:=true \
    -p max_navigation_goals:=5 \
    -p return_home_enabled:=false
```

候选就绪后，终端 D 按第五节逐条推进 start_mapping、finish_mapping、
start_search，每条 success=True 才继续；此时只是无运动规划。
确认 False/True/5/False、DRY_RUN_SELECTION_READY 和 Selected 日志。

机器人位置变化时，在 Pi Docker 终端 D 读取新位姿和传感器，不沿用旧路径：

```bash
  timeout 8s ros2 run tf2_ros tf2_echo map base_footprint
  timeout 8s ros2 topic delay /scan
  timeout 5s ros2 topic echo /search/selected_goal \
    --qos-reliability reliable --qos-durability transient_local --once
```

现场将坐标/朝向与已知地图对应，确认路径与终点转向空间；无法确认不运动。
scan 必须有实际延迟样本，暂定最大 <=0.5 s；TF 初始化等待不是最终结论。
重启后的实际运动任务会重新规划，不保证重用 dry-run 的同一候选或路径。

现场授权本轮最多五次导航后，恢复 E/F 监控，终端 D 再调用 mission/reset，
必须 success=True。仅重启终端 C 的策略，使用：

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 run ee3033_search_policy search_manager --ros-args \
    --params-file /home/ros2_ws/src/ee3033-search-policy/config/search_policy.yaml \
    -p navigation_enabled:=true \
    -p single_goal_mode:=true \
    -p max_navigation_goals:=5 \
    -p return_home_enabled:=false
```

候选就绪后，终端 D 按第五节核对参数为 True/True/5/False。
逐条执行 start_mapping、finish_mapping，每条 success=True 才继续；
最后一次 start_search 是【真实连续运动入口】，只调用一次。
节点自动重复选点/规划/导航/计时观察，不写 shell 循环派发五次 start_search。
取消方法见第七节；结束后按第九节读取覆盖和确认终态。
五次是请求尝试上限，检测尚未接入，不是五次完整目标识别任务。

### 8B. 可选路线：保存并恢复覆盖，最多五次新的请求

只在已确认上一活动目标终结、现场停止后执行，不在 SEARCHING/PAUSED 盲目重置。
保持基础栈、mission_manager 和监控，不需要重建地图或重新构建。

[Pi ROS Docker；控制终端 D]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  timeout 10s ros2 topic echo /search/status \
    --qos-reliability reliable --qos-durability transient_local --once
  timeout 10s ros2 topic echo /search/coverage_fraction \
    --qos-reliability reliable --qos-durability transient_local --once
```

记下本次覆盖值。下面备份旧默认快照，避免保存时覆盖旧记录；
若 memory_path 使用其他路径，必须同步替换保存/备份/重启路径。

```bash
  if [ -f /tmp/ee3033_search_memory.npz ]; then
    backup=$(mktemp /tmp/ee3033_previous_memory_XXXXXX.npz) &&
    cp /tmp/ee3033_search_memory.npz "$backup" &&
    printf 'Previous snapshot backup: %s\n' "$backup"
  fi
  ros2 service call /search/save_memory std_srvs/srv/Trigger '{}'
  ros2 service call /mission/finish std_srvs/srv/Trigger '{}'
```

逐条检查成功；任何备份/服务失败不继续。
FINISHED 不清零覆盖；mission/reset 会清零，所以这里不用 reset。

[Pi ROS Docker；原策略终端 C]
  只 Ctrl+C 停止 search_manager，等返回 shell 后：

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  ros2 run ee3033_search_policy search_manager --ros-args \
    --params-file /home/ros2_ws/src/ee3033-search-policy/config/search_policy.yaml \
    -p navigation_enabled:=true \
    -p single_goal_mode:=true \
    -p max_navigation_goals:=5 \
    -p return_home_enabled:=false \
    -p memory_path:=/tmp/ee3033_search_memory.npz
```

等待生成候选并接收到任务 FINISHED。不重启 mission_manager。
single_goal_mode=true 是启用计数上限，并不是永远只能走一个点；
max_navigation_goals=5 配合它提供当前所需有界循环。

[Pi ROS Docker；控制终端 D]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  for p in navigation_enabled single_goal_mode max_navigation_goals return_home_enabled; do
    printf '\n===== %s =====\n' "$p"
    ros2 param get /search_manager "$p"
  done
  ros2 service call /search/load_memory std_srvs/srv/Trigger '{}'
  timeout 10s ros2 topic echo /search/coverage_fraction \
    --qos-reliability reliable --qos-durability transient_local --once
```

门控必须 True / True / 5 / False；load 必须 success=True；
覆盖必须与保存前一致，不能默认为重启会自动恢复。
当前 load 校验 observable_mask 和候选数量，不是完整地图 metadata/hash 校验；
地图改变时不要复用旧快照。失败 blacklist 当前不保存/恢复。
新节点重新初始化取消锁存和计数；之后从 WAIT_FOR_TARGET 进入 SEARCHING
又会按新任务设置计数。不要用 pause/resume 绕过已用目标数。

保留恢复的覆盖，推进任务；每条 success=True 才继续：

```bash
  ros2 service call /mission/start_mapping std_srvs/srv/Trigger '{}'
  ros2 service call /mission/finish_mapping std_srvs/srv/Trigger '{}'
```

确认现场授权本轮有界连续运动、断电开关就绪，再执行【真实运动】：

```bash
  ros2 service call /mission/start_search std_srvs/srv/Trigger '{}'
```

只调用一次，代码自动选下一点，不写 shell for 循环调用五次 start_search。
默认 coverage_target=0.995，正增益/可达候选耗尽时允许提前结束。
每次到达后等待观察并更新覆盖；导航失败增加 blacklist 失败数并可继续选点。
5 是新的导航请求尝试预算，不是五次成功、五次规划或总计五分钟。
navigation_goal_count 在 navigate_to_pose 返回 ActionRequest 后递增；
拒绝、超时等已建立的请求也消耗预算，提交前抛异常不消耗。
因此内部次数不必等于 Nav2 服务器接受的 UUID 数，失败日志须同时保留。
程序没有总体实验超时：持续等 planner 等情况不保证在有限分钟内结束。

停止方法仍为第七节 cancel_navigation，并验证终态/现场停止。
不要在运行中 Ctrl+C 底盘栈，也不要把 mission/finish 当成紧急停止证明。
终止或预算耗尽后不要再次 start_search；那会形成新的任务预算。


## 九、结束、保存与实验记录

可能结束状态：
  SINGLE_GOAL_TEST_COMPLETE：预算耗尽且最后一次完成到达/观察；
  SINGLE_GOAL_TEST_FAILED：预算耗尽且最后一次导航失败；
  SINGLE_GOAL_LIMIT_REACHED：达到派发上限；
  COVERAGE_TARGET_REACHED、NO_REMAINING_GAIN、NO_REACHABLE_GAIN：提前结束。
这些名称沿用现有代码，不表示五次模式只执行了一次。
自然结束的失败/超时状态本身也不能证明远端动作与机器人已停止。

先核实最后活动 UUID 的终态、现场停止、没有新目标，再执行：
[Pi ROS Docker；控制终端 D]

```bash
  source /opt/ros/humble/setup.bash
  source /home/ros2_ws/install/setup.bash
  timeout 10s ros2 topic echo /search/status \
    --qos-reliability reliable --qos-durability transient_local --once
  timeout 10s ros2 topic echo /search/coverage_fraction \
    --qos-reliability reliable --qos-durability transient_local --once
  ros2 service call /search/save_memory std_srvs/srv/Trigger '{}'
  ros2 service call /mission/finish std_srvs/srv/Trigger '{}'
```

保留日志与快照，记录：
  日期/时区、分支完整 SHA、地图路径/hash、模型 checksum；
  实际参数、起止覆盖、每次请求/UUID/接受拒绝/终态/失败原因；
  候选/选点/规划/推理时间、路径距离、停止确认、提前结束原因。
覆盖值乘以 100% 得到百分比，本次实测为 69.00%，是模型覆盖记忆，
不代表 YOLO 检测覆盖或真实图像已完成观察。
未测量字段写 N/A，不根据预算宣称已经跑满五次。
实验结束经现场同意才按进程关闭，关闭监控不是关闭机器人。
本文件不能作为物理机器人验收报告；只有真实输出和现场确认才能验收。


## 十、如何做到「一个命令」，以及目前的缺口

可以做到：一条命令启动编排器，合并检查、日志、进程管理和有界搜索；
不能把它理解为无人确认地从建图直接进入运动。

优先分两种入口，而不是每次把全部流程重跑：
  A. 已有地图：复用/启动基础栈 -> 合并就绪检查 -> 定位确认 ->
     启动策略/恢复覆盖 -> 现场授权 -> 最多 N 次尝试 -> 验证停止/保存。
  B. 新建地图：已核实 SLAM 入口 + 操作员操纵 -> 人工确认地图完成 -> 保存并验证 ->
     安全切换保存地图模式 -> 接 A 的定位和运动授权关口。
一个父程序可以在同一次运行中等待这些事件；不需要开十几个终端确认同一事实。
下一次指导应按「准备批次、运动授权、结束验收」给命令，不逐个节点反复询问。

建议后续最小实现（本次没有实现，下列是设计而非现有命令）：
  - 仓库内新增仅启动 mission_manager/search_manager 的 no-target launch。
    参数明确暴露 map/导航开关/N/返航关闭/memory_path；导航默认关闭。
  - 包装器管理自己创建的进程；已有节点则核对后复用，有冲突就报错。
    不无条件重启、不按进程名批量 kill、不写教师包、不自动拉代码或安装。
  - 自动 source 和环境审计，保存版本、地图/model hash、结构化逐目标日志。
  - 用就绪状态和 Action result 推进，不用固定 sleep 或 shell 成功码冒充验收。
    ros2 service call 进程正常退出也不意味着 Trigger_Response.success=True。
  - 中断/超时/传感器失效时先锁存禁止派发，再取消并等远端终态；
    不能只 Ctrl+C 父进程或发送 cancel 就宣称机器人已停。
  - 覆盖恢复校验完整地图签名、参数/候选一致性和暂停状态；区分新任务与恢复。
  - 单命令仍保留现场定位/路径/断电确认及明确运动授权，不自动发送测试探针目标。
  - 实现总体实验时间预算，逐目标账本包含接受、拒绝、成功、失败、取消和超时；
    预算不会因暂停/掉线/重启意外重置。结束后自动归档并等待现场停止确认。

当前实现不能直接当作可靠一键端到端程序：
  - full_runtime.launch.py 同时启动 target_checker/target_localizer，没有无目标模式；
    没有启动教师基础栈、没有地图切换、没有 return_home_enabled launch 参数。
    YAML 默认 return_home_enabled=true，因此本流程显式覆盖 false。
    不直接把 full_runtime navigation_enabled=true 当作本次无识别流程的替代品。
  - 人工建图的 SLAM 入口已核实，见 3B；自主探索的安全启动/停止、自动完成通知
    和阶段切换尚待集成，不能把定时启动 explore_lite 当作就绪验收。
  - 当前 _robot_pose 查询最新可用 TF，没有自动的 TF/scan 新鲜度拒动门控。
    本文的延迟采样只是人工关口，不是持续运行的安全保障。
  - 观察依赖 Nav2 success + 计时，没有独立 heading / 真实观察完成确认。
  - 本地 Nav2 超时和远端终结不是同一件事，需要验证未知终态时的禁止后续派发。
  - 快照仅校验 observable_mask/候选数，且不保存失败 blacklist/尝试预算。
  - 没有统一的就绪、逐目标统计、总时限、信号清理和持久归档编排器。

结论：复用保存地图的「一命令启动 + 一次现场运动授权」最容易先落地；
包含建图的同一入口也可实现，但仍需补齐建图完成、操纵停止和阶段切换接口。
先做无运动启动/停止/日志编排，再验证故障门控，最后才接入有界运动。
不要为了减少命令数量去掉终态验证，应该自动化它并消除重复人工检查。

依据：AGENTS.md；config/search_policy.yaml；launch/full_runtime.launch.py；
ee3033_search_policy/{mission_manager_node,search_manager_node,coverage_memory,
nav2_client,target_stop_guard}.py；2026-11-03 本次操作员输出；
课程 Lab4_report.pdf 第 5-6、10-11 页（仅用于 3B 建图流程，不包含凭据）。


## 十一、下一阶段：自动 YOLO 接入（待开发，尚未验收）

本次五次导航流程不包含目标检测。下一阶段先保持 navigation_enabled=false，
不启动旧的 navigation_enabled=true 策略，不直接使用 full_runtime 一键运动。
已经完成的测试不等于摄像头、YOLO、确认或自动停止测试通过。

README 当前要求自动 bottle 类别检测，不需要参考照片或人工点击；
现场仍需确认目标类别、实际模型类别表及运行机器。
现有 target_checker 消费 /detector/status 和 balance_interface/msg/TargetInfo，
检查 TRACKING、similarity 和 bbox，再做连续帧确认。这里 similarity 是外观
相似度，不是 YOLO 类别置信度；教师参考匹配检测器不能通过一个参数变成类别检测。
target_localizer 也使用旧 TargetInfo，新增适配器时必须保持直接依赖调用一致。

第一批仅发现摄像头/检测包、实际 Topic/消息/QoS/时间戳和模型能力，
不安装依赖、不启动摄像头或检测器、不写教师包。发现接口后才确定最小实现：
类别/置信度筛选、bbox 与图像尺寸、源时间戳/新鲜度、重复帧和多帧确认、
/target_found 锁存及重置。无目标、错误类别、低置信度、陈旧/重复帧和无效 bbox
不能确认目标；测试从无运动开始。
确认目标后停止派发、请求取消，并验证 Nav2 终态和现场停止；
不得把取消请求成功当作停止。坐标定位是后续独立验收，不假定 LiDAR 背景墙
距离就是目标距离。新的运动测试需要重新明确授权。
