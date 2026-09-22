# Bài tập ROS 2: UR3/UR3e viết chữ P

Package sử dụng chữ `P`, là chữ cái đầu của tên Phúc. Chữ được tạo thủ công
bằng các Cartesian waypoint trong mặt phẳng Y-Z của `base_link`. MoveIt 2 giải
IK, kiểm tra collision/self-collision, tạo joint trajectory và gửi trajectory
đến controller của robot mô phỏng.

Build package riêng từ thư mục `ur_gz`:

```bash
cd ~/ur_robot_drawing/ur_gz
source /opt/ros/humble/setup.bash
colcon build --packages-select ur_drawing --symlink-install
source install/setup.bash
```

Robot simulation, controller và MoveIt phải được khởi động từ repository
Universal Robots riêng trước. Sau đó chạy node vẽ và RViz:

```bash
ros2 launch ur_drawing draw_p.launch.py
```

Để dùng UR3e, đặt `ur_type:=ur3e` trong launch mô phỏng và MoveIt.

Node chờ MoveIt và TF sẵn sàng. Node chạy một lần và giữ hoạt động để RViz vẫn
nhận được đường đi. Cửa sổ RViz
mới hiển thị robot, chữ P mục tiêu màu xanh và vệt `tool0` thực tế màu cam.
Chuyển động tiếp cận được thực hiện trước và không được ghi vào nét chữ.

RViz có hai lưới: mặt đất XY và mặt phẳng viết YZ gần TCP tại `x=0.28`. Trục tọa độ
gắn với `tool0` giúp quan sát hướng đầu công tác. Trong panel `Views`, có thể
chọn `Close letter view` để nhìn gần nét chữ hoặc `Robot overview` để xem toàn
bộ chuyển động robot. Display `TF frames` được chuẩn bị sẵn và có thể bật khi
cần kiểm tra các frame.

Điều khiển camera trong RViz:

- Chọn công cụ `Move Camera` trên thanh công cụ, giữ chuột trái và kéo để xoay.
- Lăn con lăn chuột để zoom in/out.
- Giữ nút chuột giữa và kéo để dịch chuyển góc nhìn.
- Chọn `Focus Camera`, sau đó bấm vào robot hoặc đường chữ để lấy điểm đó làm
  tâm xoay và zoom.
- Panel `Views` cho phép chuyển nhanh giữa `Close letter view` và
  `Robot overview`.

Chữ nằm trong mặt phẳng Y–Z của `base_link`, chân chữ cố định tại
`(0.30, -0.08, 0.18)` m, rộng 0.06 m và cao 0.12 m. Robot đi trực tiếp tới chân chữ bằng MoveIt và giữ orientation đạt được
trong toàn bộ nét chữ. Robot đi từ chân
chữ lên đỉnh, sau đó vẽ nửa ellipse về giữa thân chữ. Đây là đường TCP trong
không gian, không tạo nét mực hoặc tiếp xúc với bề mặt trong Gazebo.

Đoạn tiếp cận điểm đầu được giới hạn ở 15% tốc độ để Gazebo ổn định tại goal;
`speed_scale` điều khiển tốc độ vẽ Cartesian và mặc định là 12.5%.

```bash
ros2 launch ur_drawing draw_p.launch.py x:=0.30 y:=-0.08 z:=0.18 width:=0.06 height:=0.12 speed_scale:=0.125
```

Để xem trước hình học đường mục tiêu, không lập quỹ đạo hoặc chạy robot:

```bash
ros2 launch ur_drawing draw_p.launch.py execute:=false
```

Nếu dùng cửa sổ RViz có sẵn, thêm hai display loại `Path`, chọn topic
`/draw_p/target_path` và `/draw_p/actual_path`, đặt Durability thành
`Transient Local`, rồi chạy với `launch_rviz:=false`.

Node dùng `/compute_cartesian_path` và `/execute_trajectory` của MoveIt,
bật kiểm tra va chạm và từ chối chạy khi quỹ đạo không đầy đủ. Vị trí mặc định
không được đảm bảo khả thi với mọi tư thế ban đầu: nếu báo `Incomplete path`,
chỉnh vị trí/kích thước chữ hoặc dùng MoveIt đưa robot gần chân chữ trước.
Node yêu cầu `use_sim_time:=true`; launch đã đặt sẵn tham số này.
