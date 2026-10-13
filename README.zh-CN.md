# Spark 赛道：导航 + 抓取交接

[English](README.md) | 中文

`course_nav.py` 让 NXROBO Spark 完成赛道：

1. 从起点线出发，绕过赛道中间的纸箱。
2. 到终点线后切换为摄像头引导，**在蓝色或黄色物块前方稍远处停下**，距离足够机械臂抓取，且不会推动物块。
3. 返回起点线。

它本身**不做抓取**。它通过两个 ROS 话题把控制权交给抓取/放下脚本，见[交接协议](#交接协议)。

本文说明这个脚本如何建立在 Spark 代码之上，以及如何完全复现我们的运行。

| 文件 | 说明 |
|---|---|
| `course_nav.py` | 导航脚本。唯一需要运行的代码。 |
| `course_nav_start.json` | 赛道起点在 `map/` 地图中的位姿，由 `course_nav.py set-start` 写入。 |
| `map/test_map.yaml`、`map/test_map.pgm` | 我们运行时使用的 SLAM 地图（10 月 13 日 17:31 建图）。 |
| `camera_stream.py` | 可选：在浏览器中查看机器人的摄像头画面。 |

---

## 与 Spark 代码的关系

我们**没有修改或替换任何 Spark 代码**。`course_nav.py` 是一个普通的 ROS 节点，运行在原版 Spark 导航系统之上：

```
course_nav.py   （本仓库）   赛道逻辑：去哪里、绕行记忆、摄像头接近、
      |                      提前停车规则、安全检查、抓取交接话题
      |  发送 move_base 目标，短距离移动时发 /cmd_vel，读取 /scan /odom /tf /map /camera
      v
spark_navigation/amcl_demo_lidar.launch   （Spark）
      |-- map_server        提供保存好的 SLAM 地图
      |-- amcl              用激光雷达在地图上定位
      |-- move_base         规划绕障路径（DWA 局部规划器、代价地图）
      |-- 速度平滑器         (follower_velocity_smoother)
      `-- spark_bringup/driver_bringup.launch
            |-- spark_base        底盘驱动（/dev/sparkBase），/odom
            |-- ydlidar_g2        激光雷达驱动，/scan
            |-- astra 摄像头      /camera/rgb/image_raw
            `-- 机器人模型与 TF
```

| Spark 部分 | 用途 | 方式 |
|---|---|---|
| `spark_slam/2d_slam_teleop.launch`（OneKey 选项 **4**） | 建图 | gmapping + 键盘遥控，然后用 `map_server map_saver` 保存 |
| `spark_navigation/amcl_demo_lidar.launch`（OneKey 选项 **6** 是同一套加 RViz） | 驱动、定位、路径规划 | 启动一次；`course_nav.py` 向它发送目标 |
| Spark 配置文件（`spark_navigation/param/*.yaml`） | 代价地图、DWA 规划器、AMCL | 原样使用。`course_nav.py` 运行时通过 `dynamic_reconfigure` 提高速度上限，不修改任何文件 |
| `move2grasp/scripts/grasp_pro_new.py`、`spark_sagittarius_carry/config/hsv.yaml` | 蓝色/黄色的 HSV 颜色范围 | 数值复制到 `course_nav.py` 中，不调用这些代码 |
| `task1`、`move2grasp` 的 `move.py`、SwiftPro 机械臂 | 不使用 | 未使用。它们**不能**同时运行（见[重要规则](#重要规则)） |

### 我们使用的版本

- **机器人电脑：** Ubuntu 20.04.5，ROS Noetic。
- **机器人与传感器：** Spark，配 YDLIDAR G2 激光雷达、Astra 摄像头和 SwiftPro 机械臂（本脚本不使用机械臂）。
- **Spark 工作空间：** `~/spark_noetic`，克隆自 `https://github.com/nxrobo/spark_noetic`，提交 `a3669fe`，用 `catkin_make` 编译。
  - 唯一与导航有关的本地改动是 `spark_navigation/param/dwa_local_planner_params.yaml` 中的 `max_vel_x`：上游为 0.3，我们机器人上是 0.1。
  - `course_nav.py` 运行时无论如何都会把它设为 0.3，所以用原版代码效果相同。
- **Python 依赖：** 只用 Spark 上 ROS Noetic 自带的：`rospy`、`actionlib`、`tf`、`cv_bridge`、OpenCV、NumPy 和 `dynamic_reconfigure`。

---

## 逐步复现我们的运行

以下所有操作都**在机器人上**以 `spark` 用户执行，可以通过 SSH（`ssh spark@<机器人IP>`）或在机器人屏幕上的终端里执行。我们的机器人 IP 是 `192.168.161.145`。

### 0. 把代码放到机器人上（只需一次）

```bash
cd ~
git clone https://github.com/RAY2L/xrobot.git
cd ~/xrobot
```

在这个文件夹里运行 `course_nav.py`。它会在自己旁边读写 `course_nav_start.json`（如果示教了停车线，还有 `course_nav_reach.json`）。

### 1. 地图：使用我们的，或重新建图

- **与我们相同的房间和赛道：** 使用 `~/xrobot/map/test_map.yaml` 和随附的 `course_nav_start.json`，然后直接进入第 2 步。
- **其他地方**，或家具位置变化很大，就重新建图：
  1. **布置赛道。** 把纸箱和物块放到比赛时的位置。
  2. **开始建图。** 运行 `cd ~/spark_noetic && ./onekey.sh`，选择 **4**（激光雷达 2D 建图），再选 **1**（gmapping）。它执行的是：
     ```bash
     roslaunch spark_slam 2d_slam_teleop.launch slam_methods_tel:=gmapping camera_type_tel:=astra lidar_type_tel:=ydlidar_g2
     ```
  3. **用键盘慢慢开**（`w` 前进，`s` 后退，`a`/`d` 转向）。
     - 从起点线沿赛道出发，从纸箱两侧都绕一下，开到终点线之后，再开回来。
     - 在两端各原地转一圈，让周围的墙和家具都被建进地图。
  4. **保存地图**，建图仍在运行时，在第二个终端中执行：
     ```bash
     source ~/spark_noetic/devel/setup.bash
     rosrun map_server map_saver -f ~/xrobot/map/test_map
     ```
     Spark 自带的 `spark_slam/scripts/cmd_save_map.sh` 会保存到 `spark_slam/scripts/test_map`，两个位置都可以。
  5. **结束建图**，按 Ctrl-C。新地图一定要重新执行 `set-start`（第 3 步）。

### 2. 启动导航系统：只能启动一份

```bash
source ~/spark_noetic/devel/setup.bash
roslaunch spark_navigation amcl_demo_lidar.launch camera_type_tel:=astra lidar_type_tel:=ydlidar_g2 \
    start_camera:=true map_file:=$HOME/xrobot/map/test_map.yaml
```

保持运行。然后在第二个终端中关闭 `move_base` 的原地旋转恢复行为并重启 `move_base`。定位丢失时，这个行为曾让机器人疯狂打转，而且这些参数只在启动时读取：

```bash
source ~/spark_noetic/devel/setup.bash
rosparam set /move_base/recovery_behavior_enabled false
rosparam set /move_base/clearing_rotation_allowed false
rosnode kill /move_base
rosrun move_base move_base cmd_vel:=/navigation_velocity_smoother/raw_cmd_vel odom:=odom scan:=scan __name:=move_base
```

这个也保持运行。

可选：在机器人屏幕上打开 RViz，查看地图、机器人和规划路径：

```bash
rviz -d ~/spark_noetic/src/spark_app/spark_navigation/rviz/amcl_camera_astra.rviz
```

也可以用 OneKey 选项 **6** 代替上面的 `roslaunch`，它是同一套系统加 RViz。但它总是加载 `spark_slam/scripts/test_map.yaml`，所以用它时要把地图保存到那里。

**运行期间，绝不要再启动第二套导航、建图、遥控或其他导航脚本。** 同一个 ROS master 上的两个 launch 会互相杀掉对方的节点，我们因此不止一次丢失了激光雷达和底盘。

### 3. 告诉脚本机器人在哪里

把机器人**放在起点线中间，朝向赛道**。下面两个命令都不会让机器人移动。

- **新建地图后**，记录一次起点：
  ```bash
  cd ~/xrobot
  python3 course_nav.py set-start --min-match 0.45
  ```
  它在整张地图上搜索与激光雷达最匹配的位姿，并保存到 `course_nav_start.json`。赛道方向就是机器人当前朝向。`--min-match 0.45` 表示接受 45% 的匹配度，地图比激光雷达看到的范围小时需要它。
- **每次（重新）启动导航系统后**，或机器人被人手动搬动后：
  ```bash
  python3 course_nav.py localize
  ```
  它在赛道附近找到机器人并告诉 AMCL，并打印匹配百分比：我们看到的是 75–85%。

### 4. 先检查，再运行

```bash
python3 course_nav.py --dry-run --targets any,any     # 检查地图、目标点和定位；不移动
python3 course_nav.py round-trip --targets any,any    # 我们最后一次运行的完整命令
```

`round-trip --targets any,any` 会跑**两趟**。每一趟：
1. 在地图上导航到赛道 2.3 m 处，越过纸箱。
2. 用摄像头接近看到的蓝色或黄色物块，并在它前方稍远处停下。
3. 后退 0.2 m，掉头，导航回起点线。

我们最后一次运行两趟共用 **2 分 42 秒**，机器人回到起点的误差在 9 cm 以内。

其他常用命令：

```bash
python3 course_nav.py round-trip --targets blue,yellow            # 第一趟蓝色，第二趟黄色
python3 course_nav.py round-trip --targets blue,yellow --handoff  # 每端等待抓取脚本
python3 course_nav.py to-end --targets blue                       # 单程，在蓝色前停下
python3 course_nav.py to-start                                    # 从当前位置返回起点
python3 course_nav.py round-trip                                  # 只用地图：开到 2.75 m 再返回，不用摄像头
python3 course_nav.py --help                                      # 所有选项
```

随时可以按 **Ctrl-C** 停止，机器人会停下。

### 5. 在笔记本上观察（可选）

- **摄像头：** 在机器人上运行 `python3 ~/xrobot/camera_stream.py`，然后在浏览器打开 `http://<机器人IP>:8080/`。画面会标出蓝色/黄色检测框和红色停车线。它不会让任何东西移动。
- **RViz：** 它运行在机器人自己的屏幕上。我们的机器人还在 5900 端口运行 VNC 服务。

---

## 运行原理

- **起点位姿：** 来自 `course_nav_start.json`。其他所有位置都以它为基准：x 沿赛道向前，y 向左。
- **地图阶段。**
  - `move_base` 开到赛道 2.3 m 处（`--handover-distance`）。纸箱在保存的地图里，实时激光雷达也能看到它，所以规划器会绕开它。
  - 第一次经过时，脚本会记住向外绕行的那个点，与纸箱至少保持 0.5 m。之后每一段（包括返回）都经过这个点，所以回程与去程对称。
  - `--side left|right` 可以强制绕行方向。
- **摄像头接近。**
  - 先沿直线搜索颜色，再朝它转向。
  - 当物块在图像中的**下边缘**到达停车线时停下，默认是 480 行图像中的第 400 行（`--stop-row`）。停车时物块仍在画面中。
  - 绝不盲开。如果看不到物块了，就停下并最多等待 3 秒让它重新出现（`--lost-timeout`）。激光雷达在前方 25 cm 内看到任何东西也会停车。
- **返回。**
  - 先后退 0.2 m（`--backoff`），这样转身时不会扫到物块。如果后方 35 cm 内有东西，就停止后退。
  - 然后经过记住的绕行点导航回起点，并转回朝向赛道。
- **本次运行的速度**（启动时设置，不写入任何配置文件）：

  | 项目 | 速度 |
  |---|---|
  | `move_base` | 最高 0.3 m/s（`--nav-speed`；我们机器人上的 Spark 默认值是 0.1） |
  | 接近阶段，离停车线较远时 | 0.12 m/s |
  | 接近阶段，靠近停车线时 | 0.04 m/s |
  | 后退 | 0.1 m/s |

### 示教停车线（推荐）

第 400 行只是一个通用的估计值。要让机器人正好停在**你们的**机械臂够得到的地方：
1. 把机器人放在机械臂能抓到物块的位置，且物块仍在摄像头画面中。
2. 运行（不会移动）：
   ```bash
   python3 course_nav.py teach-reach --targets blue
   ```
3. 黄色同样操作一次。

它会把该物块下边缘和中心在图像中的位置保存到 `course_nav_reach.json`。之后接近时会停在那里，并转向让物块保持在那一列。每次摄像头角度被调整后都要重新示教。

如果只想测试接近阶段，把机器人放在物块前方 1–1.5 m 处，运行 `python3 course_nav.py approach --targets blue`。

### 重要规则

- **一套导航，一个驾驶者。**
  - `course_nav.py` 运行期间，只有它能控制底盘。
  - 先停止 OneKey 遥控、建图、`task1` 以及 `move2grasp` 的 `move.py`/`grasp_pro_new.py`。有一次运行中，`move2grasp` 发出的目标在半路接管了 `move_base`。
  - 抓取脚本必须使用[交接协议](#交接协议)，绝不能发布 `/cmd_vel` 或 `move_base` 目标。
- **起点位姿只属于一张地图。** 重新建图就要重新运行 `set-start`。
- **机器人被搬动或碰撞过，就运行 `localize`。** 如果在 AMCL 位姿上匹配地图的激光点少于 50%，脚本会拒绝开车。这救了我们好几次。
- **如果电池没电，所有东西都要重新启动：** 第 2、3 步。开机后不会自动启动任何东西。

---

## 交接协议

两个话题都是 `std_msgs/String`。状态消息是 JSON，并且是 latched，所以后订阅的节点也能收到最后一条状态。

| `course_nav.py` | 抓取脚本 |
|---|---|
| 控制底盘：`/cmd_vel`、`move_base` 目标 | 控制机械臂：`/position_write_topic`、`/pump_topic` |
| 发布 `/course_nav/status` | 订阅 `/course_nav/status` |
| 订阅 `/course_nav/continue` | 发布 `/course_nav/continue` |

使用 `--targets blue,yellow --handoff` 时，状态顺序是：

```
TO_END        {"trip": 1, "target": "blue"}
AT_END        {"trip": 1}
APPROACHING   {"color": "blue"}
IN_REACH      {"color": "blue", "pixel": [u, v], "bottom_row": 401, "stop_row": 400, "taught": true, "stopped_by": "camera", "pose": [x, y, yaw], "travelled_m": 0.6}
   （或 NOT_IN_REACH {"reason": "not_found" | "lost" | "blocked" | "too_far", ...}）
WAITING       {"waiting_for": "grasp"}      <- 现在抓取，然后发布 /course_nav/continue
TO_START      {"trip": 1}
AT_START      {"trip": 1}
WAITING       {"waiting_for": "release"}    <- 现在放下，然后发布 /course_nav/continue
TO_END        {"trip": 2, "target": "yellow"}
... 黄色同上 ...
DONE
```

- 要继续，就在 `/course_nav/continue` 上发布任意字符串，例如 `"done"`。
- 要中止，就发布 `"abort"`。机器人原地停下，脚本退出。
- 如果 `--wait-timeout`（600 秒）内没有收到消息，机器人原地不动，脚本退出。
- 如果任何环节失败，机器人停下，发布 `FAILED {"error": "..."}`，然后脚本退出。

`IN_REACH` 中：
- `pixel` 是停车时物块颜色区域的中心在 640×480 图像 `/camera/rgb/image_raw` 中的位置。对蓝色瓶子来说，那是**瓶顶的蓝色盖子**，不是瓶底。停车时物块仍在画面中，所以抓取脚本可以再看一次。
- `stopped_by` 是 `camera`（到达停车线）或 `lidar`（前方 25 cm 内有东西）。

### 最简抓取脚本框架

```python
#!/usr/bin/env python3
import json, rospy
from std_msgs.msg import String
from swiftpro.msg import position, status

rospy.init_node('course_grasp')
arm = rospy.Publisher('/position_write_topic', position, queue_size=1)
pump = rospy.Publisher('/pump_topic', status, queue_size=1)
go_on = rospy.Publisher('/course_nav/continue', String, queue_size=1)
last_in_reach = {}

def grasp(info):
    # info = IN_REACH 记录：颜色、像素 [u, v]、机器人位姿……
    # TODO：像素 -> 机械臂 x, y（需要针对当前摄像头位置的标定），然后
    # 悬停、下降、开吸泵、抬到不挡摄像头/激光雷达的搬运姿态。
    pass

def release():
    # TODO：在起点线放低、关吸泵、机械臂回到搬运/初始姿态。
    pass

def on_status(msg):
    s = json.loads(msg.data)
    if s['state'] in ('IN_REACH', 'NOT_IN_REACH'):
        last_in_reach.clear(); last_in_reach.update(s)
    elif s['state'] == 'WAITING':
        if s['waiting_for'] == 'grasp':
            if last_in_reach.get('state') != 'IN_REACH':
                go_on.publish('abort'); return
            grasp(last_in_reach)
        else:
            release()
        go_on.publish('done')

rospy.Subscriber('/course_nav/status', String, on_status, queue_size=10)
rospy.spin()
```

先启动机械臂驱动（`roslaunch swiftpro pro_control_nomoveit.launch`），在单独的终端运行抓取脚本，然后运行 `course_nav.py round-trip --targets blue,yellow --handoff`。

## 抓取端的机械臂说明

- **机械臂：** SwiftPro，接在 `/dev/ttyACM0`；用 `roslaunch swiftpro pro_control_nomoveit.launch` 启动驱动。
- **话题。**
  - `/position_write_topic` 接收 `swiftpro/position`：机械臂坐标系下的 x、y、z，单位 mm。
  - `/pump_topic` 接收 `swiftpro/status`：`1` 开吸泵，`0` 关。
  - 机械臂的状态只是回显命令，不能确认是否吸住，所以抬起后要用摄像头检查。
- **参考动作序列：** `~/spark_noetic/src/task1/scripts/mission.py` 中的 `pick()` / `release()`：

  | 步骤 | 机械臂位置 |
  |---|---|
  | 悬停 | z = 20 |
  | 下降 | z = -50 |
  | 开吸泵，然后搬运 | (20, 150, 130) |
  | 放下 | (200, 0, -50) |
- **我们这次调试期间摄像头被重新调整过角度**，所以旧的像素到机械臂标定已经不适用。包括 `task1/config/arm_calibration.txt` 和 `~/thefile.txt`；后者是未标定的 `0 227.5 0 105`。
  - 格式是 `a b c d`，即 `arm_x = a*v + b`，`arm_y = c*u + d`。
  - 要重新标定，用 OneKey 选项 **8**（摄像头-机械臂标定），它会写入 `~/thefile.txt`。
  - 更简单的方法：在固定的机械臂位置抓取，也就是机器人停在示教停车线时物块所在的位置。
- **不同物块的抓取高度不同。** 蓝色瓶子比黄色方块高，而且蓝色检测到的是瓶顶。
