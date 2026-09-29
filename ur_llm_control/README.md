# UR3e Natural-Language Skill Control

ROS 2 Humble package điều khiển UR3e + Robotiq 2F-85 bằng lệnh tiếng Việt hoặc
tiếng Anh. Gemini chỉ tạo kế hoạch symbolic; validator và code robot quyết định
mọi chuyển động.

## Kiến trúc

~~~text
/user_command
  → Gemini GenerateContent
  → strict JSON validator
  → pick / place / home
  → MoveIt 2 + Gazebo
~~~

Ví dụ kế hoạch hợp lệ:

~~~json
{"plan":[
  {"skill":"pick","object":"red_cube"},
  {"skill":"place","object":"red_cube","zone":"zone_b"},
  {"skill":"home"}
]}
~~~

LLM không được sinh tọa độ, góc khớp, quỹ đạo hoặc lệnh ROS. Pose và giới hạn
chuyển động nằm trong config/world.yaml.

Các file chính:

- planner.py: gọi trực tiếp Gemini REST API và yêu cầu structured JSON.
- validator.py: kiểm tra schema, whitelist và thứ tự skill.
- skills.py: thực thi pick, place, home.
- node.py: nối topic ROS, planner, validator và robot.
- simulation.launch.py: Gazebo, MoveIt, controller và RViz.
- llm_control.launch.py: node điều khiển bằng ngôn ngữ tự nhiên.

## 1. Build và test

~~~bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash

rosdep install --ignore-src --from-paths . -r -y
colcon build --packages-select robotiq_description ur_llm_control --symlink-install
source install/setup.bash

PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -q -p no:cacheprovider ur_llm_control/test
~~~

Sau mỗi lần sửa code, build và source lại install/setup.bash.

## 2. Cấu hình Gemini API

Tạo API key tại [Google AI Studio](https://aistudio.google.com/app/apikey), rồi
export trong chính terminal dùng để chạy ROS:

~~~bash
export GEMINI_API_KEY='YOUR_KEY'

test -n "${GEMINI_API_KEY:-}" \
  && echo "Gemini API key loaded" \
  || echo "ERROR: GEMINI_API_KEY missing"
~~~

Không lưu key trong source, YAML, launch file hoặc Git.

Model mặc định là gemini-2.5-flash. Có thể đổi bằng --model khi test hoặc
gemini_model:=... khi launch.

Kiểm tra Gemini mà không khởi động robot:

~~~bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run ur_llm_control test_llm \
  "Hãy đưa khối màu đỏ vào vùng B"
~~~

Kết quả phải có RAW RESPONSE và VALIDATED PLAN.

Tham số CLI:

~~~text
--model          gemini-2.5-flash
--api-key-env    GEMINI_API_KEY
--timeout        30
--student-name   YOUR_NAME
--student-id     00000000
--world-config   đường dẫn world.yaml tùy chọn
~~~

## 3. Chạy simulation

Terminal 1:

~~~bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_llm_control simulation.launch.py
~~~

Chờ robot, controller, MoveIt và RViz khởi động xong.

## 4. Test robot bằng mock

Mock không gọi Gemini, dùng để tách lỗi robot khỏi lỗi mạng/API.

Terminal 2:

~~~bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur_llm_control llm_control.launch.py \
  student_name:="Dinh Van Phuc" \
  student_id:="23020756" \
  mock_llm:=true
~~~

Terminal 3:

~~~bash
source /opt/ros/humble/setup.bash
source ~/ur_robot_drawing/install/setup.bash

ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Hãy đưa khối màu đỏ vào vùng B'}"
~~~

## 5. Chạy với Gemini

Giữ simulation ở Terminal 1, dừng node mock, rồi chạy:

~~~bash
cd ~/ur_robot_drawing
source /opt/ros/humble/setup.bash
source install/setup.bash
export GEMINI_API_KEY='YOUR_KEY'

ros2 launch ur_llm_control llm_control.launch.py \
  student_name:="Dinh Van Phuc" \
  student_id:="23020756" \
  gemini_model:="gemini-2.5-flash" \
  mock_llm:=false
~~~

Gửi lệnh:

~~~bash
ros2 topic pub --once /user_command std_msgs/msg/String \
  "{data: 'Đưa vật màu đỏ sang vùng B'}"
~~~

Không chạy đồng thời node mock và node Gemini.

## 6. Topic và trạng thái

| Topic | Nội dung |
|---|---|
| /user_command | Lệnh tự nhiên |
| /llm/raw_response | JSON thô từ Gemini |
| /llm/validated_plan | Kế hoạch đã qua validator |
| /execution_status | Trạng thái planner và robot |

~~~bash
ros2 topic echo /execution_status
~~~

| Status | Ý nghĩa |
|---|---|
| LLM_FAILED | Lỗi key, mạng, model, quota hoặc response Gemini |
| INVALID_PLAN | JSON không qua validator |
| PLANNING_FAILED | MoveIt không tìm được quỹ đạo |
| EXECUTION_FAILED | Controller hoặc gripper thất bại |
| BUSY | Robot đang chạy kế hoạch khác |
| SUCCESS | Hoàn tất |

## 7. Troubleshooting

### GEMINI_API_KEY is not set

Export key trong cùng terminal chạy test_llm hoặc ros2 launch.

### HTTP 400

Kiểm tra tên model và thử model mặc định:

~~~bash
ros2 run ur_llm_control test_llm "home" --model gemini-2.5-flash
~~~

### HTTP 401 hoặc 403

API key sai, bị hạn chế hoặc project chưa được phép dùng Gemini API. Tạo/copy
lại key từ Google AI Studio.

### HTTP 429

Project đã chạm rate limit hoặc quota. Kiểm tra quota trong Google AI Studio.

### INVALID_PLAN

~~~bash
ros2 topic echo /llm/raw_response
~~~

Plan chỉ được dùng pick, place, home; object/zone phải có trong
config/world.yaml; bước cuối phải là home.

### PLANNING_FAILED

Kiểm tra Planning Scene trong RViz, frame_id, ee_link và pose trong
config/world.yaml. Không sửa prompt để LLM tự sinh tọa độ.

## 8. Cá nhân hóa MSSV

Node lấy hai số cuối của student_id, tính P = XX mod 6 và ánh xạ màu cho ba
zone. Mapping được in khi node khởi động. Hãy thay student_name và student_id
trước khi demo.

## Tài liệu Gemini

- [Gemini API text generation](https://ai.google.dev/gemini-api/docs/text-generation)
- [Gemini structured outputs](https://ai.google.dev/gemini-api/docs/structured-output)
