# UR Robot Drawing

Project ROS 2 Humble mô phỏng robot Universal Robots trong Gazebo và điều
khiển TCP để vẽ chữ **P** hoặc **hình tròn** bằng MoveIt 2.

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

Nạp ROS 2 trong mỗi terminal làm việc:

```bash
source /opt/ros/humble/setup.bash
```

## 3. Lấy source code

Nếu đã có project:

```bash
cd /home/phuc/ur_robot_drawing
```

Nếu clone lại từ Git:

```bash
git clone <URL_REPOSITORY_UR_ROBOT_DRAWING> ur_robot_drawing
cd ur_robot_drawing
```

Repo mô phỏng Universal Robots phải được clone riêng tại đúng thư mục:

```bash
git clone -b humble \
  https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git \
  Universal_Robots_ROS2_GZ_Simulation
```

Nếu thư mục repo đã tồn tại, kiểm tra branch:

```bash
cd Universal_Robots_ROS2_GZ_Simulation
git status
git branch --show-current
cd ..
```

Kết quả branch nên là `humble`.

## 4. Build repo Universal Robots

Repo Universal Robots là một ROS workspace độc lập. Không build package
`ur_drawing` trong workspace này.

```bash
cd /home/phuc/ur_robot_drawing/Universal_Robots_ROS2_GZ_Simulation
source /opt/ros/humble/setup.bash

# Tải các repository phụ thuộc được khai báo cho Humble.
vcs import src < ur_simulation_gz.humble.repos

rosdep update
rosdep install --ignore-src --from-paths src -r -y

colcon build --symlink-install
source install/setup.bash
```

Kiểm tra package mô phỏng đã được đăng ký:

```bash
ros2 pkg prefix ur_simulation_gz
```

Kết quả phải trỏ tới thư mục `install` của
`Universal_Robots_ROS2_GZ_Simulation`.

## 5. Build package `ur_drawing`

Mở một terminal mới, hoặc tiếp tục terminal sau khi source ROS 2:

```bash
cd /home/phuc/ur_robot_drawing/ur_gz
source /opt/ros/humble/setup.bash

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
cd /home/phuc/ur_robot_drawing/Universal_Robots_ROS2_GZ_Simulation
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  ur_type:=ur3 launch_rviz:=false gazebo_gui:=true
```

Launch này khởi động:

- một Gazebo;
- robot state publisher;
- controller;
- TF và `/robot_description`.

Có thể thay `ur3` bằng loại robot được repo Universal Robots hỗ trợ:

```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  ur_type:=ur3e launch_rviz:=false gazebo_gui:=true
```

Giữ terminal này đang chạy.

### Terminal 2: khởi động MoveIt

```bash
cd /home/phuc/ur_robot_drawing/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3 use_sim_time:=true launch_rviz:=false
```

Không chạy `ur_sim_moveit.launch.py` ở terminal này vì nó sẽ tự gọi lại
control launch và có thể tạo Gazebo thứ hai. Cần bảo đảm MoveIt cung cấp:

- `/compute_cartesian_path`
- `/execute_trajectory`
- `/move_action`
- TF từ `base_link` tới `tool0`

### Terminal 3: chạy package vẽ và RViz riêng

```bash
cd /home/phuc/ur_robot_drawing/ur_gz
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

Để xem hình học đường mục tiêu (không lập quỹ đạo MoveIt, không di chuyển robot):

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
| `x` | `0.30` | tọa độ X của chân chữ hoặc tâm tròn trong `base_link`, mét |
| `y` | `-0.08` | tọa độ Y của chân chữ hoặc tâm tròn trong `base_link`, mét |
| `z` | `0.18` | tọa độ Z của chân chữ hoặc tâm tròn trong `base_link`, mét |
| `shape` | `p` | hình cần vẽ: `p` hoặc `circle` |
| `radius` | `0.04` | bán kính hình tròn, mét |
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

## 11. Xử lý lỗi thường gặp

### `Package 'ur_drawing' not found`

Build và source đúng workspace:

```bash
cd /home/phuc/ur_robot_drawing/ur_gz
source /opt/ros/humble/setup.bash
colcon build --packages-select ur_drawing --symlink-install
source install/setup.bash
ros2 pkg prefix ur_drawing
```

Không chỉ source `install/setup.bash` của repo Universal Robots; package
`ur_drawing` nằm trong workspace `ur_gz`.

### Không tìm thấy `ur_simulation_gz`

Kiểm tra đang source đúng workspace Universal Robots:

```bash
cd /home/phuc/ur_robot_drawing/Universal_Robots_ROS2_GZ_Simulation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix ur_simulation_gz
```

### Không tìm thấy service MoveIt hoặc TF

Phải chạy cả hai launch của repo Universal Robots trước khi chạy
`ur_drawing`. Kiểm tra nhanh:

```bash
ros2 service list | grep compute_cartesian_path
ros2 action list | grep execute_trajectory
ros2 run tf2_ros tf2_echo base_link tool0
```

### `Incomplete path`

MoveIt không lập được toàn bộ Cartesian path. Thử:

- giảm `width` hoặc `height`;
- thay đổi `x`, `y`, `z` để đưa chữ vào workspace;
- giảm `step`;
- dùng `execute:=false` để xem vị trí hình (không kiểm tra khả năng thực thi);
- đưa robot về tư thế khởi đầu ổn định.

### Gazebo hoặc RViz chạy chậm

Chạy Gazebo không giao diện nếu launch của repo Universal Robots hỗ trợ:

```bash
ros2 launch ur_simulation_gz ur_sim_control.launch.py \
  ur_type:=ur3 launch_rviz:=false gazebo_gui:=true
```

Launch tổng hợp của repo Universal Robots sẽ mở một Gazebo và một RViz. Nếu
cần chạy không giao diện Gazebo, hãy xem các launch argument được repo đó cung
cấp cho đúng branch đang dùng; không sửa trực tiếp repo gốc trong project này.

## 12. Tài liệu liên quan

- Hướng dẫn riêng của package: [`ur_gz/README.md`](ur_gz/README.md)
- Mô tả hình học và RViz: [`ur_gz/src/ur_drawing/DRAW_P.md`](ur_gz/src/ur_drawing/DRAW_P.md)
- Repo mô phỏng Universal Robots:
  [`Universal_Robots_ROS2_GZ_Simulation/`](Universal_Robots_ROS2_GZ_Simulation/)

## Vẽ hình tròn và cách tiếp cận điểm đầu

```bash
ros2 launch ur_drawing draw_p.launch.py shape:=circle radius:=0.04 launch_rviz:=true
```

`shape:=p` (mặc định) vẽ chữ P; `shape:=circle` vẽ hình tròn bán kính
`radius` (mặc định 0.04 m). Cả hai nằm trong mặt phẳng Y-Z.
Với chữ P, `(x, y, z)` là chân chữ. Với hình tròn, đó là tâm;
điểm bắt đầu là `(x, y, z - radius)` và đường kết thúc tại chính điểm đầu.

Luồng chạy: tạo waypoint → MoveIt lập đường tới waypoint đầu → giữ hướng
TCP vừa đạt được → lập và thực thi đường Cartesian của hình.
MoveIt tự chọn hướng TCP khi tiếp cận, không cần cấu hình khớp cố định của UR3.
Đoạn tiếp cận có kiểm tra va chạm nhưng không bắt buộc là đường thẳng;
nó không được ghi vào nét vẽ màu cam. Nếu hướng TCP đạt được không cho phép
vẽ toàn bộ hình, node dừng và báo lỗi, không thực thi nét vẽ dở dang.
Đoạn tiếp cận có thể đã hoàn thành trước khi phát hiện lỗi này.

Đây là vẽ đường TCP trong mô phỏng, chưa có thao tác nhấc/hạ bút hoặc tiếp xúc
mặt giấy. Tên launch và topic `/draw_p/*` được giữ để dùng cấu hình RViz cũ.
