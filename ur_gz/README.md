# UR Robot Drawing

ROS 2 Humble workspace chạy mô phỏng Universal Robots trong Gazebo và dùng
MoveIt 2 để vẽ chữ **P** bằng TCP của robot.

## Yêu cầu

- Ubuntu 22.04
- ROS 2 Humble
- `colcon`
- `vcstool` (`vcs`)
- Kết nối Internet để tải các repository phụ thuộc

## Tải mã nguồn

Clone repository cùng với submodule mô phỏng UR:

```bash
git clone --recurse-submodules -b main \
  https://github.com/phuwcc/ur_robot_drawing.git
cd ur_robot_drawing
```

Nếu đã clone mà chưa lấy submodule:

```bash
git submodule update --init --recursive
```

Submodule [`src/ur_simulation_gz`](src/ur_simulation_gz) sử dụng branch
`humble` của `Universal_Robots_ROS2_GZ_Simulation`.

## Cài dependency và build

Workspace ROS nằm trong thư mục `ur_gz`:

```bash
cd ur_gz
source /opt/ros/humble/setup.bash

# Tải các repository phụ thuộc được khai báo bởi mô phỏng UR.
vcs import src < src/ur_simulation_gz/ur_simulation_gz.humble.repos

rosdep update
rosdep install --ignore-src --from-paths src -r -y

colcon build --symlink-install
source install/setup.bash
```

Nếu chỉ muốn build các package chính của project sau khi dependency đã được
cài đặt:

```bash
colcon build --packages-select ur_simulation_gz ur_drawing --symlink-install
source install/setup.bash
```

## Chạy mô phỏng và vẽ chữ P

Từ thư mục `ur_gz`, chạy một launch duy nhất:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_drawing draw_p.launch.py
```

Launch này khởi động:

1. Gazebo với robot UR3;
2. `ur_control` và các controller mô phỏng;
3. MoveIt 2;
4. node `draw_p`;
5. RViz với cấu hình hiển thị robot, chữ P mục tiêu và đường đi thực tế.

Node sẽ chờ các service của MoveIt và TF cần thiết trước khi lập và thực thi
quỹ đạo. Sau khi vẽ xong, node vẫn giữ hoạt động để đường đi còn hiển thị
trong RViz.

## Các tùy chọn thường dùng

Chạy với UR3e:

```bash
ros2 launch ur_drawing draw_p.launch.py ur_type:=ur3e
```

Chạy không mở giao diện Gazebo:

```bash
ros2 launch ur_drawing draw_p.launch.py gazebo_gui:=false
```

Chỉ lập quỹ đạo và không gửi lệnh chạy robot:

```bash
ros2 launch ur_drawing draw_p.launch.py execute:=false
```

Điều chỉnh vị trí, kích thước và tốc độ chữ:

```bash
ros2 launch ur_drawing draw_p.launch.py \
  x:=0.30 y:=-0.08 z:=0.18 \
  width:=0.06 height:=0.12 \
  step:=0.004 speed_scale:=0.125
```

Các tham số vị trí tính theo `base_link`, đơn vị mét. `speed_scale` là hệ số
tốc độ vẽ; giá trị mặc định là `0.125` (12.5%).

## Xử lý lỗi

- **Không tìm thấy package hoặc launch file:** kiểm tra đã source cả
  `/opt/ros/humble/setup.bash` và `install/setup.bash` trong đúng terminal
  chưa.
- **Thiếu package khi build:** chạy lại `rosdep install --ignore-src
  --from-paths src -r -y` sau khi đã chạy `vcs import`.
- **`Incomplete path`:** vị trí hoặc kích thước chữ có thể nằm ngoài vùng làm
  việc của robot hoặc gây collision. Thử giảm `width`/`height`, thay đổi
  `x`, `y`, `z`, hoặc dùng `execute:=false` để kiểm tra trước.
- **Gazebo/RViz chạy chậm:** dùng `gazebo_gui:=false` hoặc giảm tải các display
  không cần thiết trong RViz.

Tài liệu mô tả chi tiết hơn về hình học chữ P và các topic RViz nằm tại
[`src/ur_drawing/DRAW_P.md`](src/ur_drawing/DRAW_P.md).
