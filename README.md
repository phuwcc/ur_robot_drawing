# UR Robot Drawing

Project ROS 2 Humble mô phỏng robot Universal Robots trong Gazebo và điều
khiển TCP để vẽ chữ **P** bằng MoveIt 2.

## 1. Cấu trúc repository

Project được chia thành hai phần độc lập:

```text
ur_robot_drawing/
├── Universal_Robots_ROS2_GZ_Simulation/
│   └── ur_simulation_gz/        # repo gốc của Universal Robots
└── ur_gz/
    └── src/ur_drawing/          # package riêng để vẽ chữ P
```

`ur_gz` **không chứa** submodule, bản sao hoặc package của
`Universal_Robots_ROS2_GZ_Simulation`. Repo Universal Robots cung cấp Gazebo,
robot description, controller, TF và MoveIt; package `ur_drawing` chỉ kết nối
vào các service/action đó để tạo và thực thi quỹ đạo.

## 2. Yêu cầu hệ thống

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- `rosdep`
- `vcstool` (`vcs`)
- Kết nối Internet để tải các dependency ROS
- Gazebo và các package MoveIt/Universal Robots được khai báo bởi repo mô phỏng

## 3. Lấy source code

```bash
git clone https://github.com/phuwcc/ur_robot_drawing.git
cd ur_robot_drawing
```

Repo mô phỏng Universal Robots phải được clone riêng tại đúng thư mục:

```bash
git clone -b humble \
  https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git \
  Universal_Robots_ROS2_GZ_Simulation
```

## 4. Build repo Universal Robots & ur_drawing

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash

# Tải các repository phụ thuộc được khai báo cho Humble.
vcs import src < ur_simulation_gz.humble.repos

rosdep update
rosdep install --ignore-src --from-paths src -r -y

colcon build --symlink-install
source install/setup.bash
```

## 5. Build package `ur_drawing`

rosdep update
rosdep install --ignore-src --from-paths src -r -y

colcon build --packages-select ur_drawing --symlink-install
source install/setup.bash
```

Kiểm tra package:

```bash
colcon list
ros2 pkg prefix ur_drawing
```

`colcon list` phải hiển thị:

```text
ur_drawing    src/ur_drawing    (ros.ament_cmake)
```

## 6. Chạy mô phỏng, MoveIt và RViz

Để robot chắc chắn hiển thị trong đúng RViz của project và không tạo hai
Gazebo, dùng ba terminal. Không chạy `ur_sim_moveit.launch.py`, vì launch đó
tự mở RViz mặc định của MoveIt.

### Terminal 1: khởi động Gazebo và controller

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  ur_type:=ur3e launch_rviz:=false gazebo_gui:=true
```

Launch này khởi động:

- một Gazebo;
- robot state publisher;
- controller;
- TF và `/robot_description`.

Giữ terminal này đang chạy.

### Terminal 2: khởi động MoveIt

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e use_sim_time:=true launch_rviz:=false
```

Không chạy `ur_sim_moveit.launch.py` ở terminal này vì nó sẽ tự gọi lại
control launch và có thể tạo Gazebo thứ hai. Cần bảo đảm MoveIt cung cấp:

- `/compute_cartesian_path`
- `/execute_trajectory`
- `/move_action`
- TF từ `base_link` tới `tool0`

### Terminal 3: chạy package vẽ và RViz riêng

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_drawing draw_p.launch.py launch_rviz:=true
```

RViz này dùng cấu hình [`draw_p.rviz`](ur_gz/src/ur_drawing/config/draw_p.rviz),
trong đó `RobotModel` đọc `/robot_description` và Fixed Frame là `base_link`.

## 7. Chạy tính năng vẽ chữ P

Launch của `ur_drawing` chỉ khởi động node `draw_p` và RViz riêng; nó không
khởi động Gazebo, controller, robot description hoặc MoveIt.

## 8. Chạy thử không điều khiển robot

Để lập quỹ đạo và kiểm tra khả năng tính toán nhưng không gửi lệnh thực thi:

```bash
ros2 launch ur_drawing draw_p.launch.py execute:=false
```

## 9. Các tham số vẽ

Ví dụ đầy đủ:

```bash
ros2 launch ur_drawing draw_p.launch.py \
  x:=0.30 \
  y:=-0.08 \
  z:=0.18 \
  width:=0.06 \
  height:=0.12 \
  step:=0.004 \
  speed_scale:=0.125
```

| Tham số | Mặc định | Ý nghĩa |
| --- | ---: | --- |
| `x` | `0.30` | tọa độ X của chân chữ trong `base_link`, mét |
| `y` | `-0.08` | tọa độ Y của chân chữ trong `base_link`, mét |
| `z` | `0.18` | tọa độ Z của chân chữ trong `base_link`, mét |
| `width` | `0.06` | chiều rộng chữ P, mét |
| `height` | `0.12` | chiều cao chữ P, mét |
| `step` | `0.004` | khoảng cách giữa các Cartesian waypoint, mét |
| `speed_scale` | `0.125` | hệ số tốc độ, trong khoảng lớn hơn 0 và nhỏ hơn hoặc bằng 1 |
| `execute` | `true` | có gửi trajectory tới robot hay không |
| `launch_rviz` | `false` | có mở RViz riêng của `ur_drawing` hay không |

Node giữ orientation hiện tại của TCP trong suốt nét vẽ. Đường chữ nằm trong
mặt phẳng Y-Z của `base_link`.

## 10. Topic và frame

Package xuất bản các topic để quan sát trong RViz:

```text
/draw_p/target_path
/draw_p/actual_path
```

Các frame mặc định:

```text
frame_id: base_link
ee_link: tool0
group_name: ur_manipulator
```

RViz có thể hiển thị:

- robot state;
- đường chữ mục tiêu màu xanh;
- đường TCP thực tế màu cam;
- lưới mặt đất và mặt phẳng viết;
- TF frames khi cần kiểm tra.

