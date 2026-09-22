# UR Robot Drawing

Package ROS 2 Humble chứa tính năng vẽ chữ **P** hoặc **hình tròn** bằng MoveIt trên TCP của
robot. Thư mục này chỉ chứa mã nguồn riêng của tính năng vẽ; không chứa mã
nguồn Universal Robots, Gazebo hoặc MoveIt.

## Build

```bash
cd /home/phuc/ur_robot_drawing/ur_gz
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --ignore-src --from-paths src -r -y
colcon build --symlink-install
source install/setup.bash
```

Repository Universal Robots riêng phải được build và source trước để cung cấp
robot, TF, controller và các service MoveIt cần thiết. `ur_gz` không tải,
chứa hoặc quản lý repository đó.

## Chạy tính năng vẽ chữ P

Sau khi robot stack và MoveIt đã chạy, từ thư mục `ur_gz` chạy:

```bash
ros2 launch ur_drawing draw_p.launch.py
```

Launch này chỉ khởi động node `draw_p` và tùy chọn RViz; nó không khởi động
Gazebo, controller hoặc MoveIt. Mặc định `launch_rviz` là `false` để không
mở thêm RViz nếu launch MoveIt đã mở RViz.

Để dùng RViz với cấu hình hiển thị robot của package này, chạy:

```bash
ros2 launch ur_drawing draw_p.launch.py launch_rviz:=true
```

Khi dùng cách này, hãy khởi động Gazebo bằng `ur_sim_control.launch.py
launch_rviz:=false`, khởi động MoveIt bằng `ur_moveit_config` với
`launch_rviz:=false`, rồi mới chạy launch trên. Không dùng
`ur_sim_moveit.launch.py` trong quy trình này vì launch đó tự gọi lại control
launch và mở RViz mặc định.

Chỉ xem hình học đường mục tiêu; không lập quỹ đạo MoveIt hoặc di chuyển robot:

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
tốc độ vẽ, mặc định là `0.125` (12.5%).

## Xử lý lỗi

- **Không tìm thấy package hoặc launch file:** source cả
  `/opt/ros/humble/setup.bash` và `install/setup.bash`.
- **Không tìm thấy service MoveIt hoặc TF:** khởi động robot stack và MoveIt
  từ repository Universal Robots riêng trước khi chạy launch này.
- **`Incomplete path`:** vị trí hoặc kích thước chữ có thể nằm ngoài vùng làm
  việc của robot hoặc gây collision. Thử giảm `width`/`height`, thay đổi
  `x`, `y`, `z`, hoặc đưa robot về tư thế khởi đầu khác.

Tài liệu chi tiết về hình học chữ P và các topic RViz nằm tại
[`src/ur_drawing/DRAW_P.md`](src/ur_drawing/DRAW_P.md).

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
