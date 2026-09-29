# UR3e Natural-Language Skill Control

ROS 2 Humble package điều khiển UR3e + Robotiq 2F-85 bằng lệnh tự nhiên.
Gemini chỉ tạo kế hoạch `pick/place/home`; validator kiểm tra kế hoạch trước khi
MoveIt và Gazebo thực thi. LLM không được điều khiển tọa độ hoặc khớp trực tiếp.

```text
/user_command → Gemini → validator → skills → MoveIt → Gazebo
```

## Build

```bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths . -r -y
colcon build --packages-select robotiq_description ur_llm_control --symlink-install
source install/setup.bash
```

## Gemini API key

Tạo key tại [Google AI Studio](https://aistudio.google.com/app/apikey) và export
trong terminal chạy node. Không lưu API key trong source hoặc Git.

```bash
export GEMINI_API_KEY='YOUR_KEY'
```

## Chạy

Terminal 1 — simulation:

```bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_llm_control simulation.launch.py
```

Chờ Gazebo, controller và MoveIt khởi động xong.

Terminal 2 — Gemini và skill executor:

```bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash
export GEMINI_API_KEY='YOUR_KEY'

ros2 launch ur_llm_control llm_control.launch.py \
  student_name:="Dinh Van Phuc" \
  student_id:="23020756" \
  gemini_model:="gemini-2.5-flash"
```

Terminal 3 — gửi lệnh:

```bash
source /opt/ros/humble/setup.bash
source ~/ur_robot_drawing/install/setup.bash

ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa vật màu đỏ sang vùng B'}"
```

## Cấu hình

- `config/world.yaml`: bàn, vật, vùng đặt, pose và giới hạn chuyển động.
- `planner.py`: gọi Gemini REST API và yêu cầu structured JSON.
- `validator.py`: chỉ chấp nhận skill, object và zone nằm trong whitelist.
- `skills.py`: thực thi `pick`, `place`, `home`.
- `moveit_*.py`: planning scene, Cartesian motion và fallback an toàn.
- `physical_gripper.py`: Robotiq controller và Gazebo attach/detach.

Hai số cuối của `student_id` được dùng để tính ánh xạ màu cho zone. Mapping
được in khi node khởi động.

## Topics

| Topic | Nội dung |
|---|---|
| `/user_command` | Lệnh tự nhiên |
| `/llm/raw_response` | JSON từ Gemini |
| `/llm/validated_plan` | Kế hoạch đã kiểm tra |
| `/execution_status` | Trạng thái planner và robot |

Theo dõi trạng thái:

```bash
ros2 topic echo /execution_status
```

Các lỗi chính:

- `LLM_FAILED`: API key, mạng, model hoặc quota Gemini.
- `INVALID_PLAN`: JSON không qua validator.
- `PLANNING_FAILED`: MoveIt không tìm được đường đi an toàn.
- `EXECUTION_FAILED`: controller hoặc gripper thất bại.
- `BUSY`: robot đang thực thi lệnh khác.
- `SUCCESS`: kế hoạch hoàn tất.

Nếu Cartesian path không đạt 100%, node tự chuyển sang collision-aware pose
planning. Dòng `retrying with collision-aware pose planning` là fallback bình
thường, không phải lỗi kết thúc.
