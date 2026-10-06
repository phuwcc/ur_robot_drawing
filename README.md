# UR3e Camera-Grounded Natural-Language Skill Control

ROS 2 Humble package điều khiển UR3e + Robotiq 2F-85 bằng lệnh tự nhiên và
camera RGB trong Gazebo Fortress. Gemini chỉ tạo kế hoạch `pick/place/home`;
camera cung cấp tọa độ vật, validator kiểm tra kế hoạch trước khi MoveIt và
Gazebo thực thi. LLM không được điều khiển tọa độ hoặc khớp trực tiếp.

```text
/user_command → Gemini → validator → xử lý occupancy → skills → MoveIt → Gazebo
                       ↑                                      ↓
                 trạng thái bàn ← nhận diện màu ← /camera/image
```

## Build

```bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths ur_llm_camera -r -y
colcon build --packages-select ur_llm_camera --symlink-install
source install/setup.bash
```

## Gemini API key

Tạo key tại [Google AI Studio](https://aistudio.google.com/app/apikey) và export
trong terminal chạy node. Không lưu API key trong source hoặc Git.

```bash
export GEMINI_API_KEY='YOUR_KEY'
```

## Chạy

Terminal 1 — simulation, MoveIt và RViz:

```bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_llm_camera simulation.launch.py
```

Chờ Gazebo, controller và MoveIt khởi động xong.

Terminal 2 — Gemini, camera perception và skill executor:

```bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash
export GEMINI_API_KEY='YOUR_KEY'

ros2 launch ur_llm_camera llm_control.launch.py \
  student_name:="Dinh Van Phuc" \
  student_id:="23020756" \
  gemini_model:="gemini-2.5-flash"
```

Terminal 3 — gửi lệnh:

```bash
source /opt/ros/humble/setup.bash
source ~/ur_robot_drawing/install/setup.bash

ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Arrange all objects according to my student ID.'}"
```

Ví dụ điều khiển một vật:

```bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Put the red cube in Zone B.'}"
```

## Cá nhân hóa theo MSSV

Hai số cuối của `student_id` được dùng để tính `P = XX mod 6`, từ đó xác định
màu cần đặt vào Zone A, B và C. Mapping được in khi node khởi động.

Với `student_id=23020756`:

```text
56 mod 6 = 2
Zone A → Yellow
Zone B → Red
Zone C → Blue
```

Lệnh `Arrange all objects according to my student ID.` yêu cầu robot sắp xếp
ba cube theo mapping này. Kế hoạch có nhiều cặp `pick/place` liên tiếp và chỉ
thực hiện `home` một lần sau bước cuối cùng.

## Xử lý zone bị chiếm

World ban đầu đặt `blue_cube` trong Zone B. Nếu cần đưa `red_cube` vào Zone B,
lớp xử lý scene sẽ tìm một vị trí trống, kiểm tra IK và collision, rồi mở rộng
kế hoạch:

```text
pick blue_cube → place temporary_N
pick red_cube  → place zone_b
home
```

`temporary_N` được tính từ ảnh camera, biên bàn, kích thước cube và khoảng hở;
đây không phải tọa độ cố định. Nếu không có vị trí trống và reachable, robot
dừng trước khi gắp.

## Camera và vật bị che

- Camera RGB 800×800, 10 Hz; OpenCV nhận diện năm cube theo màu HSV.
- Tọa độ cube được chiếu từ pixel xuống mặt bàn trong frame `base_link`.
- Hệ thống không đọc model pose từ Gazebo để lập kế hoạch.
- Ảnh phải mới, ổn định và không có nhiều ứng viên cùng màu.
- Khi tay robot che vật vừa thả hoặc vật sắp gắp, hệ thống tạm dùng vị trí
  camera xác nhận gần nhất. Vật sẽ được xác nhận lại khi hiện ra trong ảnh.
- Sau `home` cuối cùng, mọi placement còn chờ đều phải được camera xác nhận.

Xem ảnh detection:

```bash
ros2 run rqt_image_view rqt_image_view /camera/detections
```

## Cấu hình

- `config/world.yaml`: bàn, cube, zone, camera, gripper và giới hạn chuyển động.
- `planner.py`: gọi Gemini REST API và yêu cầu structured JSON.
- `validator.py`: chỉ chấp nhận skill, object và zone nằm trong whitelist.
- `scene.py`: kiểm tra occupancy và tìm vị trí tạm khi zone bị chiếm.
- `perception.py`, `camera_state.py`: nhận diện màu và kiểm tra frame ổn định.
- `skills.py`: thực thi `pick`, `place`, `home` và xác nhận placement.
- `moveit_*.py`: planning scene, Cartesian motion và fallback tránh va chạm.
- `physical_gripper.py`: Robotiq controller và Gazebo attach/detach.

## Topics

| Topic | Nội dung |
|---|---|
| `/user_command` | Lệnh tự nhiên |
| `/camera/image` | Ảnh RGB từ Gazebo |
| `/camera/detections` | Ảnh có kết quả nhận diện |
| `/perception/world_state` | Tọa độ cube và occupancy của các zone |
| `/llm/raw_response` | JSON từ Gemini |
| `/llm/validated_plan` | Kế hoạch đã kiểm tra và mở rộng |
| `/execution_status` | Trạng thái planner và robot |

Theo dõi trạng thái:

```bash
ros2 topic echo /perception/world_state
ros2 topic echo /llm/validated_plan
ros2 topic echo /execution_status
```

Các trạng thái và lỗi chính:

- `PERCEPTION_READY`: camera đã cung cấp trạng thái bàn hợp lệ.
- `PLAN_VALID`: kế hoạch đã qua validator và xử lý occupancy.
- `PLACEMENT_VERIFIED`: camera xác nhận cube ở đúng vị trí thả.
- `LLM_FAILED`: API key, mạng, model hoặc quota Gemini.
- `INVALID_PLAN`: JSON không qua validator.
- `PLANNING_FAILED`: MoveIt không tìm được đường đi an toàn.
- `EXECUTION_FAILED`: camera, controller, gripper hoặc chuyển động thất bại.
- `BUSY`: robot đang thực thi lệnh khác.
- `SUCCESS`: toàn bộ kế hoạch đã hoàn tất.

Nếu Cartesian path không đạt 100%, node tự chuyển sang collision-aware pose
planning. Dòng `retrying with collision-aware pose planning` là fallback bình
thường, không phải lỗi kết thúc. Nếu execution thất bại sau khi robot đã di
chuyển vật, hãy khởi động lại cả simulation và node trước lần chạy tiếp theo.
