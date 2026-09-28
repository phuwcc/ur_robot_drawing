# UR3e Natural-Language Skill Control

Package ROS 2 Humble cho **Bài thực hành 02 – Điều khiển UR3 bằng LLM và
Skill-based Planning**. Package này độc lập với `ur_drawing` và không thay đổi
bài thực hành vẽ chữ P.

Hệ thống thực hiện đúng ranh giới an toàn:

```text
Natural-language command
        -> 9Router / LLM
        -> JSON plan
        -> strict validator
        -> home / pick / place
        -> MoveIt 2
        -> UR3e in Gazebo
```

LLM chỉ chọn và sắp xếp skill. Pose, joint goal, tốc độ, collision object và
quỹ đạo đều do chương trình xác định trước; chúng không bao giờ được lấy từ
phản hồi LLM.

## 1. Thành phần

- `worlds/pick_place.sdf`: bàn, ba cube và ba zone.
- `urdf/ur3e_robotiq_2f85.urdf.xacro`: UR3e ghép với URDF/Xacro và mesh gốc
  của Robotiq 2F-85 trong package ROS 2 `ros2_robotiq_gripper/robotiq_description`.
- `config/world.yaml`: pose, kích thước và tham số chuyển động.
- `planner.py`: gọi endpoint OpenAI-compatible của 9Router.
- `validator.py`: whitelist skill/object/zone và kiểm tra thứ tự logic.
- `skills.py`: triển khai `home()`, `pick(object)` và `place(object, zone)`.
- `moveit_client.py`: gửi goal và quản lý Planning Scene.
- `moveit_cartesian_client.py`: chuyển động thẳng ngắn khi hạ/nâng gripper.
- `physical_gripper.py`: điều khiển sáu khớp ngón qua action `FollowJointTrajectory`, sau đó attach/detach vật trong Gazebo.
- `node.py`: nhận lệnh, lập kế hoạch và thực thi tuần tự.

Gripper dùng mesh 2F-85, pad nguyên bản và coupling 11 mm từ repo ROS 2 mới,
được gắn vào `tool0`. Các khớp ngón được điều khiển bởi
`joint_trajectory_controller/JointTrajectoryController` qua action
`/robotiq_gripper_controller/follow_joint_trajectory`; các góc mở/đóng lấy từ
`config/world.yaml`. MoveIt cho phép vật đang gắp chạm các link của gripper
được khai báo trong `gripper.touch_links`. Sau khi hai ngón khép, plugin
`DetachableJoint` tạo fixed joint giữa robot và cube để mô phỏng lực giữ chắc;
khi thả, ngón mở và joint được xóa. Không có timer teleport cube.


## 2. Lựa chọn gripper

Package dùng **Robotiq 2F-85** vì hành trình 85 mm phù hợp với cube 50 mm và đây là dòng gripper cộng tác phổ biến cho robot UR. Cả Gazebo và MoveIt đều nạp cùng mô tả robot ghép. MoveIt lập kế hoạch cho nhóm `ur_manipulator`, còn gripper dùng ROS 2 trajectory controller riêng cho sáu khớp ngón.

Tham khảo: [Robotiq product catalogue](https://blog.robotiq.com/hubfs/Robotiq%20Kits%20for%20Partners/UR%20Robotiq%20Kit%20for%20Partners/Robotiq%20Essentials/Catalogue/Robotiq%20Product%20Catalogue_2021_10.pdf), [Robotiq ROS packages](https://github.com/robotiq/ros), [Gazebo DetachableJoint](https://gazebosim.org/api/sim/9/detachablejoints.html).

## 3. Yêu cầu

- Ubuntu 22.04.
- ROS 2 Humble.
- Gazebo Sim và `ros_gz`.
- MoveIt 2.
- Source của `Universal_Robots_ROS2_GZ_Simulation` và `ros2_robotiq_gripper` đã có trong workspace.
- 9Router đang chạy và đã kết nối ít nhất một model.

## 4. Build

Từ thư mục repository:

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash

rosdep install --ignore-src --from-paths . -r -y
colcon build --packages-select robotiq_description ur_llm_control --symlink-install
source install/setup.bash
```

Chạy unit test của validator:

```bash
colcon test --packages-select ur_llm_control
colcon test-result --verbose
```

## 5. Kiểm tra robot trước, chưa dùng LLM

Chế độ này chỉ dùng để smoke test Gazebo, MoveIt và các skill. Parser mock
không được dùng trong phần demo chính thức.

### Terminal 1 – Gazebo, UR3e, MoveIt và RViz

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_llm_control simulation.launch.py
```

Chờ robot, controller, `/move_action` và RViz khởi động xong.

### Terminal 2 – skill node ở chế độ mock

Thay thông tin sinh viên trước khi chạy:

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_llm_control llm_control.launch.py \
  student_name:="Dinh Van Phuc" \
  student_id:="23020756" \
  mock_llm:=true
```

### Terminal 3 – gửi lệnh

```bash
source /opt/ros/humble/setup.bash
source /home/phuc/ur_robot_drawing/install/setup.bash

ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Hãy đưa khối màu đỏ vào vùng B'}"
```

Node phải sinh kế hoạch tương đương:

```json
{
  "plan": [
    {"skill": "pick", "object": "red_cube"},
    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
    {"skill": "home"}
  ]
}
```

## 6. Chạy với 9Router

Khởi động 9Router theo cấu hình của máy. Endpoint local mặc định mà package sử
dụng là:

```text
http://127.0.0.1:20128/v1
```

Lấy API key trong dashboard của 9Router và xem tên model đang có:

```bash
export NINE_ROUTER_API_KEY="YOUR_9ROUTER_KEY"

curl -s http://127.0.0.1:20128/v1/models \
  -H "Authorization: Bearer ${NINE_ROUTER_API_KEY}"
```

Giữ Terminal 1 chạy simulation. Trong Terminal 2, chạy planner thật:

```bash
cd /home/phuc/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash
export NINE_ROUTER_API_KEY="YOUR_9ROUTER_KEY"

ros2 launch ur_llm_control llm_control.launch.py \
  student_name:="YOUR NAME" \
  student_id:="YOUR STUDENT ID" \
  router_model:="MODEL_NAME_FROM_9ROUTER" \
  mock_llm:=false
```

Thử nhiều cách diễn đạt:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa vật màu đỏ sang vùng B'}"

ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Hãy lấy khối màu vàng và đặt nó vào ô A'}"

ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Move the blue cube to zone C'}"
```

## 7. Topic quan sát

```text
/user_command          std_msgs/msg/String   câu lệnh đầu vào
/llm/raw_response      std_msgs/msg/String   phản hồi nguyên bản của LLM
/llm/validated_plan    std_msgs/msg/String   plan đã qua validator
/execution_status      std_msgs/msg/String   trạng thái từng bước
```

Theo dõi trạng thái:

```bash
ros2 topic echo /execution_status
ros2 topic echo /llm/validated_plan
```

Các trạng thái quan trọng:

```text
SUCCESS
INVALID_OBJECT
INVALID_ZONE
INVALID_STATE
INVALID_PLAN
PLANNING_FAILED
EXECUTION_FAILED
GRASP_FAILED
LLM_FAILED
BUSY
```

Khi plan không hợp lệ, node phát `INVALID_PLAN` và không gọi bất kỳ robot skill
nào. Executor cũng dừng ngay tại skill đầu tiên thất bại.

## 8. Cá nhân hóa bằng MSSV

Node lấy hai chữ số cuối của `student_id`, tính `P = XX mod 6` và in mapping
ngay khi khởi động. Ví dụ MSSV `23020123`:

```text
23 mod 6 = 5
zone_a = Blue
zone_b = Yellow
zone_c = Red
```

Phải truyền đúng họ tên và MSSV thật khi demo. Hai giá trị mặc định chỉ là
placeholder và node sẽ in cảnh báo nếu chưa thay.

## 9. Điều chỉnh pose khi planning thất bại

Tất cả pose nằm trong `config/world.yaml`. Nếu robot của máy không đạt được một
pose:

1. Giảm khoảng cách X của cube/zone về phía robot.
2. Giữ cube center cao hơn mặt bàn một nửa kích thước cube.
3. Không tăng `velocity_scale` trước khi toàn bộ ba vật chạy ổn định.
4. Kiểm tra Planning Scene trong RViz trước khi chạy lại.

Không sửa prompt để LLM sinh tọa độ thay thế. Việc đó vi phạm yêu cầu đề bài.
